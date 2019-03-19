#include "SemanticMapper.h"
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace ORB_SLAM2
{

SemanticMapper::SemanticMapper(System *pSystem, Tracking *pTracking, const string &strSettingPath) : mpTracker(
        pTracking), mpSystem(pSystem), mnFrameHeight(0), mnFrameWidth(0), mbNeedNewNode(false), mCurrentReferenceNode(
        nullptr), mCurrentFrame(nullptr), mbPointCloudUdate(false), mbPointStatusUpdate(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fps = fSettings["Camera.fps"];
    if (fps < 1)
        fps = 30;
    mT = 1e3 / fps;

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
    mPointSize = fSettings["Viewer.PointSize"];

}

void SemanticMapper::UpdateFrame(Frame *pFrame)
{
    if (mbNeedNewNode) {
        pFrame->MakePointCloud();
        CloudPointNode *curNode = new CloudPointNode(this, pFrame->mPointCloud, pFrame->mTcw, pFrame->mK, mnFrameHeight,
                                                     mnFrameWidth);
        if (mCurrentReferenceNode != nullptr) {
            mCurrentReferenceNode->UpdatePointCloudStatus();
            curNode->PassInformationFromNode(mCurrentReferenceNode);
        }
        CloudPointNode *lastNode = mCurrentReferenceNode;
        delete lastNode;
        mCurrentReferenceNode = curNode;
        mbNeedNewNode = false;
    } else {
        if (pFrame->mnId == 0) {
            //初始化
            mnFrameWidth = pFrame->mImDisparityLeft.cols;
            mnFrameHeight = pFrame->mImDisparityLeft.rows;
            pFrame->MakePointCloud();
            CloudPointNode *curNode = new CloudPointNode(this, pFrame->mPointCloud, pFrame->mTcw, pFrame->mK,
                                                         mnFrameHeight,
                                                         mnFrameWidth);
            mCurrentReferenceNode = curNode;
        } else {
            pFrame->MakePointCloud();
            UpdateReferencePointCloud(pFrame);
        }


    }


    {
        std::unique_lock<mutex> lock(mMutexCamera);
        mCameraPose = pFrame->mTcw;
    }
    {
        std::unique_lock<mutex> lock(mMutexDisparity);
        mImDisparityOrigin = pFrame->mImDisparityLeft;
    }
    {
        std::unique_lock<mutex> lock(mMutexSegment);
        mImSegmentOrign = pFrame->mImSegment;
    }
//    {
//        std::unique_lock<mutex> lock(mMutexCurrentFrame);
//        mCurrentFrame = pFrame;
//        mbPointCloudUdate = true;
//    }

}


void SemanticMapper::Run()
{

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer", 1024, 768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
//    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuPause("menu.Pause", false, true);
    bool bPause = false;
    cv::namedWindow("DISPARITY: origin", 0);
    while (pangolin::ShouldQuit() == false) {

        //如果设置暂停就会暂停tack
        if (menuPause && !bPause) {
            mpSystem->SetPause(true);
            bPause = true;
            SavePointCloud();
        } else if (!menuPause && bPause) {
            mpSystem->SetPause(false);
            bPause = false;
        }

        cv::Mat disparityPole;
        {
            std::unique_lock<mutex> lock(mMutexDisparity);
            if (!mImDisparityOrigin.empty())
                disparityPole = mImDisparityOrigin.clone();
        }

        if (!disparityPole.empty()) {
//            disparityPole.setTo(0, mImSegmentOrign != POLE);
            cv::Mat disparity = DrawColorMap(disparityPole);
            cv::imshow("DISPARITY: origin", disparity);
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        //当前帧的位置
        DrawCurrentCamera();


        bool bPointUpdate = false;
        {
            std::unique_lock<mutex> lock(mMutexPointUpdate);
            bPointUpdate = mbPointStatusUpdate;
            if (mbPointStatusUpdate)
                mbPointStatusUpdate = false;
        }


        if (bPointUpdate) {
            std::unique_lock<mutex> lock(mMutexSemanticMapPoints);
            cout << "now have semantic map point : " << mvSemanticMapPoints.size() << " for drawing." << endl;
        }
        DrawAllSemanticPoint();

        pangolin::FinishFrame();
        cv::waitKey(mT);

    }
}

void SemanticMapper::DrawAllSemanticPoint()
{
    std::unique_lock<mutex> lock(mMutexSemanticMapPoints);
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    for (auto &p: mvSemanticMapPoints) {
        int flag = p.label;
        switch (flag) {
            case ROAD:
                glColor3f(128.0 / 255.0, 64.0 / 255.0, 128.0 / 255.0);
                break;
            case SIDEWAILK:
                glColor3f(244.0 / 255.0, 35.0 / 255.0, 232.0 / 255.0);
                break;

            case POLE:
                glColor3f(70.0 / 255.0, 70.0 / 255.0, 70.0 / 255.0);
                break;
            case CAR:
                glColor3f(0.0, 0.0, 142.0 / 255.0);
                break;
            default:
                break;
        }
        glVertex3f(p.pt3d.at<float>(0, 0), p.pt3d.at<float>(1, 0), p.pt3d.at<float>(2, 0));
    }
    glEnd();
}

void SemanticMapper::SetPointUpdateStatus(bool status)
{
    std::unique_lock<mutex> lock(mMutexPointUpdate);
    mbPointStatusUpdate = status;
}

void SemanticMapper::DrawCurrentCamera()
{
    pangolin::OpenGlMatrix M;
    M.SetIdentity();
    cv::Mat cameraPose;
    {
        std::unique_lock<mutex> lock(mMutexCamera);
        cameraPose = mCameraPose.clone();

    }
    if (!cameraPose.empty()) {
        cv::Mat Rwc(3, 3, CV_32F);
        cv::Mat twc(3, 1, CV_32F);
        Rwc = cameraPose.rowRange(0, 3).colRange(0, 3).t();
        twc = -Rwc * cameraPose.rowRange(0, 3).col(3);

        M.m[0] = Rwc.at<float>(0, 0);
        M.m[1] = Rwc.at<float>(1, 0);
        M.m[2] = Rwc.at<float>(2, 0);
        M.m[3] = 0.0;

        M.m[4] = Rwc.at<float>(0, 1);
        M.m[5] = Rwc.at<float>(1, 1);
        M.m[6] = Rwc.at<float>(2, 1);
        M.m[7] = 0.0;

        M.m[8] = Rwc.at<float>(0, 2);
        M.m[9] = Rwc.at<float>(1, 2);
        M.m[10] = Rwc.at<float>(2, 2);
        M.m[11] = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15] = 1.0;
    } else
        M.SetIdentity();
    const float mCameraSize = 0.15;
    const float mCameraLineWidth = 2;
    const float &w = mCameraSize;
    const float h = w * 0.75;
    const float z = w * 0.6;

    glPushMatrix();

#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(M.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);

    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);

    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);
    glEnd();

    glPopMatrix();
}

cv::Mat SemanticMapper::DrawColorMap(cv::Mat &input)
{
    if (input.empty())
        return cv::Mat(480, 480, CV_8UC3, cv::Scalar::all(0));
    cv::Mat mImDisparity = input.clone();

    cv::Mat disparityVize = cv::Mat(input.size(), CV_8UC1, cv::Scalar(0));
    float maxVal = 0;
    for (int i = 0; i < mImDisparity.rows; ++i) {
        for (int j = 0; j < mImDisparity.cols; ++j) {
            if (maxVal < mImDisparity.at<float>(i, j)) {
                maxVal = mImDisparity.at<float>(i, j);
            }
        }
    }
    for (int i = 0; i < mImDisparity.rows; ++i) {
        for (int j = 0; j < mImDisparity.cols; ++j) {
            float valf = 255.0 * mImDisparity.at<float>(i, j) / maxVal;

            //只看梯度比较大的点的视差
//            if (mImDebugMaxGrad.at<float>(i, j) > minUsedGrad) {
//                if (mImDebugSegment.at<uchar>(i, j) == ROAD ||
//                    mImDebugSegment.at<uchar>(i, j) == SIDEWAILK ||
//                    mImDebugSegment.at<uchar>(i, j) == POLE)
            disparityVize.at<uchar>(i, j) = uchar(valf);

        }
    }

//将灰度转为伪色图
    cv::Mat imDisparitycolor;
    cv::applyColorMap(disparityVize, imDisparitycolor, cv::COLORMAP_JET);
    return imDisparitycolor;
}

void SemanticMapper::DrawCurrentFramePointCloud()
{

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    cv::Mat Rwc(3, 3, CV_32F);
    cv::Mat twc(3, 1, CV_32F);
    {
        std::unique_lock<mutex> lock(mMutexCurrentFrame);
        if (mbPointCloudUdate) {
            mCurrentPointCloud = mCurrentFrame->mPointCloud;
            mbPointCloudUdate = false;
        }
        cv::Mat Twc = mCurrentFrame->mTcw.inv();
        Rwc = Twc.rowRange(0, 3).colRange(0, 3);
        twc = Twc.rowRange(0, 3).col(3);
    }
    cout << "Semantic Mapper: current frame have point cloud: " << mCurrentPointCloud.size() << endl;

    for (size_t i = 0; i < mCurrentPointCloud.size(); i++) {
        if (mCurrentPointCloud[i].label == -1)
            continue;
        int flag = mCurrentPointCloud[i].label;
        switch (flag) {
            case ROAD:
                glColor3f(128.0 / 255.0, 64.0 / 255.0, 128.0 / 255.0);
                cout << "road: ";
                break;
            case SIDEWAILK:
                glColor3f(244.0 / 255.0, 35.0 / 255.0, 232.0 / 255.0);
                cout << "sidewalk: ";
                break;

            case POLE:
                glColor3f(70.0 / 255.0, 70.0 / 255.0, 70.0 / 255.0);
                cout << "pole: ";
                break;
//            case CAR:
//                glColor3f(0.0, 0.0, 142.0 / 255.0);
//                cout<<"car: ";
//                break;
            default:
                cout << "should not have happen";
                break;
        }
        cv::Mat pt3dc = mCurrentPointCloud[i].pt3d;
        cv::Mat pt3dw = Rwc * pt3dc + twc;
//        cout<<flag <<" "<< pt3dw.at<float>(0, 0)<<" "<< " "<<pt3dw.at<float>(1, 0)<<" "<<pt3dw.at<float>(2, 0)<<endl;
        glVertex3f(pt3dw.at<float>(0, 0), pt3dw.at<float>(1, 0), pt3dw.at<float>(2, 0));
        glEnd();
    }
}

void SemanticMapper::UpdateReferencePointCloud(Frame *pFrame)
{
    cv::Mat curTcw = pFrame->mTcw.clone();
    cv::Mat K = pFrame->mK.clone();
    cv::Mat refTcw = mCurrentReferenceNode->Tcw.clone();
//    cv::Mat curToRef = refTcw ;
    cv::Mat curToRef = refTcw * curTcw.inv();
    cv::Mat rotationCurToRef = curToRef.rowRange(0, 3).colRange(0, 3);
    cv::Mat translationCurToRef = curToRef.rowRange(0, 3).col(3);

    int outRanger = 0;
    int totalCur = 0;
    int holeLabel = 0;
    int unmatchLabel = 0;
    int trackedClose = 0;
    int out40 = 0;

    std::vector<SemanticMapPoint*>& currentRefPointCloud = mCurrentReferenceNode->mvpMapPoints;
    for (int i = 0; i < pFrame->mPointCloud.size(); i++) {
        //将当前帧对应的点云的通过坐标变换投影到参考帧，找到对应的点
        LabelPoint curLabelPoint = pFrame->mPointCloud[i];
        if (curLabelPoint.label == -1)
            continue;
        cv::Mat curPointxyz = curLabelPoint.pt3d;
        uchar curLabel = curLabelPoint.label;
        cv::Mat curPointInRefxyz = rotationCurToRef * curPointxyz + translationCurToRef;
        cv::Mat curPointInRefuv = K * curPointInRefxyz;
//        cout<<"before: ";
//        cout<<curPointInRefuv.at<float>(0,0)<<" "<<curPointInRefuv.at<float>(1,0)<<" "<<curPointInRefuv.at<float>(2,0)<<endl;
        curPointInRefuv /= curPointInRefuv.at<float>(2, 0);
//        cout<<"after: ";
//        cout<<curPointInRefuv.at<float>(0,0)<<" "<<curPointInRefuv.at<float>(1,0)<<" "<<curPointInRefuv.at<float>(2,0)<<endl;

        //距离大于40米的不要
        totalCur++;
        if (curPointInRefxyz.at<float>(2, 0) > 40) {
            out40++;
            continue;
        }
        int refIndex = std::round(curPointInRefuv.at<float>(1, 0)) * mnFrameWidth +
                       std::round(curPointInRefuv.at<float>(0, 0));

        //上面的转换不安全，所以在这里限制一下
        if (refIndex > (mnFrameWidth - 1) * (mnFrameHeight - 1) || refIndex < 0) {
            outRanger++;
            continue;
        }

        if (curPointInRefxyz.at<float>(2, 0) < mpTracker->GetDepthThreshold())
            trackedClose++;
        if (currentRefPointCloud[refIndex] == nullptr) {
            holeLabel++;
            SemanticMapPoint *pSMapPoint = new SemanticMapPoint(curPointInRefxyz, curLabel);
            currentRefPointCloud[refIndex] == pSMapPoint;
        } else if(currentRefPointCloud[refIndex]->mLabel != curLabel){
                unmatchLabel++;
                delete currentRefPointCloud[refIndex];
                currentRefPointCloud[refIndex] = nullptr;
                continue;

        }else
            currentRefPointCloud[refIndex]->AssertObservation(curPointInRefxyz, curLabel);
    }
    float outLabelRatio = holeLabel / float(totalCur);
    float trackedCloseRatio = trackedClose / float(totalCur);
    float unmatchLabelRatio = unmatchLabel / float(totalCur);
    if (trackedCloseRatio < 0.7) {
        mbNeedNewNode = true;
    }
    cout << "Frame " << pFrame->mnId << " out of range point count: " << outRanger << " total valid: " << totalCur
         << endl <<
         " \t holeLabelRatio: " << " (" << holeLabel << ")" << outLabelRatio << " unmatchLabelRatio: " << " ("
         << unmatchLabel << ") " << unmatchLabelRatio << endl <<
         " \t trackedCloseRatio: " << " (" << trackedClose << ")" << trackedCloseRatio << endl;
    printf("over 40m points count: %d ,%f \n", out40, float(out40) / totalCur);
//    printf("close distance is : %f \n", mpTracker->GetDepthThreshold());
    //是否还需要通过远点近点的比例来进行判断是否插入新的参考node？

}

void SemanticMapper::AddMapPoint(LabelPoint &lPoint)
{
    std::unique_lock<mutex> lock(mMutexSemanticMapPoints);
    mvSemanticMapPoints.push_back(lPoint);
}

void SemanticMapper::SavePointCloud()
{
//    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
//    std::unique_lock<mutex> lock(mMutexPoints);
//    for (auto &p: mvSemiDensePoints) {
//        uchar flag = p.label;
//        pcl::PointXYZRGB point3dw;
//        bool bUse = false;
//        switch (flag) {
//            case ROAD:
//                point3dw.r = 128;
//                point3dw.g = 64;
//                point3dw.b = 128;
//                bUse = true;
//                break;
//            case SIDEWAILK:
//                point3dw.r = 244;
//                point3dw.g = 35;
//                point3dw.b = 232;
//                bUse = true;
//                break;
//
//            case POLE:
//                point3dw.r = 70;
//                point3dw.g = 70;
//                point3dw.b = 70;
//                bUse = true;
//                break;
////            case CAR:
////                glColor3f(0.0, 0.0, 142.0 / 255.0);
////                break;
//            default:
//                break;
//        }
//        if (bUse) {
//            point3dw.x = p.pt3d.at<float>(0, 0);
//            point3dw.y = p.pt3d.at<float>(1, 0);
//            point3dw.z = p.pt3d.at<float>(2, 0);
//            pointCloud.push_back(point3dw);
//        }
//    }
////    pcl::PCDWriter writer;
//    cout << "Point Clound size: " << pointCloud.points.size() << endl;
////    writer.write<pcl::PointXYZRGB> ("map.pcd", pointCloud, false);
//    pcl::io::savePCDFileASCII("/home/SENSETIME/liwenqiang/Desktop/test_pcd.pcd", pointCloud);
//    return;
}


//void SemanticMapper::SegmentRefine()
//{
//    cv::Mat poleDisOrigin = mImDisparityOrigin.clone();
//    cv::Mat poleSeg = mImSegmentOrign.clone();
//    int width = poleSeg.cols;
//    int height = poleSeg.rows;
//    for (int v = 1; v < height - 1; v++) {
//        for (int u = 1; u < width - 1; u++) {
//            uchar flag = poleSeg.at<uchar>(v, u);
//            if (flag != POLE)
//                poleDisOrigin.at<float>(v, u) = 0;
//        }
//    }
////    poleSeg.setTo(0, Segment != POLE);
////    poleDisOrigin.setTo(0, poleSeg == 0);
//    cv::Mat poleAverageDis = cv::Mat(poleDisOrigin.size(), CV_32F, cv::Scalar::all(0));
//    difference = cv::Mat(poleDisOrigin.size(), CV_32F, cv::Scalar::all(0));
//    for (int v = 1; v < height - 1; v++) {
//        for (int u = 1; u < width - 1; u++) {
//            if (poleDisOrigin.at<float>(v, u) == 0) continue;
//            int count = 0;
//            float sum = 0;
//            int step = 1;
//            while (count < 8) {
//                sum += getAverage(poleDisOrigin, step, u, v, count);
//                step++;
//            }
//            poleAverageDis.at<float>(v, u) = sum / count;
//        }
//    }
////    for (int v = 1; v < height - 1; v++) {
////        for (int u = 1; u < width - 1; u++) {
////            float disO = poleDisOrigin.at<float>(v,u);
////            float disA = poleAverageDis.at<float>(v,u);
////
////        }
////    }
//    mImSegmentRefined = poleAverageDis;
//    difference = poleDisOrigin - poleAverageDis;
//}

float SemanticMapper::getAverage(const cv::Mat &img, int step, int u, int v, int &count)
{
    int left = u - step;
    int right = u + step;
    int up = v + step;
    int down = v - step;

    if (left < 0) left = 0;
    if (right > img.cols) right = img.cols;
    if (up > img.rows) up = img.rows;
    if (down < 0) down = 0;

    float sum = 0;
    if (img.at<float>(down, left) != 0) {
        sum += img.at<float>(down, left);
        count++;
    }
    if (img.at<float>(down, right) != 0) {
        sum += img.at<float>(down, right);
        count++;
    }
    if (img.at<float>(up, left) != 0) {
        sum += img.at<float>(up, left);
        count++;
    }
    if (img.at<float>(up, right) != 0) {
        sum += img.at<float>(up, right);
        count++;
    }
    if (img.at<float>(v, left) != 0) {
        sum += img.at<float>(v, left);
        count++;
    }
    if (img.at<float>(v, right) != 0) {
        sum += img.at<float>(v, right);
        count++;
    }
    if (img.at<float>(down, u) != 0) {
        sum += img.at<float>(down, u);
        count++;
    }
    if (img.at<float>(up, u) != 0) {
        sum += img.at<float>(up, u);
        count++;
    }
    return sum;
}


CloudPointNode::CloudPointNode(SemanticMapper *mapper,
                               const std::vector<LabelPoint> &labelPoints,
                               const cv::Mat &Tcw_,
                               const cv::Mat &k_,
                               const int &h,
                               const int &w) :
        Tcw(Tcw_.clone()), K(k_.clone()), mnFrameHeight(h), mnFrameWidth(w), mpSemanticMapper(mapper)
{

    mvpMapPoints.resize(labelPoints.size(), nullptr);
    for (auto &p: labelPoints) {
        if (p.label == -1) {
            continue;
        }
        SemanticMapPoint *pSMapPoint = new SemanticMapPoint(p.pt3d, p.label);// 默认状态为UNDETERMINE
        mvpMapPoints[p.id] = pSMapPoint;
    }
}

//语义地图点状态更新，位置更新
void CloudPointNode::UpdatePointCloudStatus()
{

    const float distanceThreshold = 0.2;
    const int minObserveTime = 4;
    const float halfDistanceThresh = 0.5f * distanceThreshold;

    //统计数据
    int total = 0;
    int observingCount = 0;
    int multiObserver = 0;
    int unStableDistance = 0;
    int stableCount = 0;
    int eraseCount = 0;
    int unStableLabel = 0;
    int behindCount = 0;

    cv::Mat Twc = Tcw.inv();
    cv::Mat r = Twc.rowRange(0, 3).colRange(0, 3);
    cv::Mat t = Twc.rowRange(0, 3).col(3);
    for (auto &sPoint : mvpMapPoints) {
        if (sPoint == nullptr)
            continue;
        total++;

        //统计观测数量
        //观测次数小于阈值，则继续观测。
        int observeTime = sPoint->GetObervetime();
        if (observeTime < minObserveTime) {
            sPoint->SetStatus(eSPointStatus::OBSERVING);
            observingCount++;
            continue;
        }

        //多次观测到的点的处理如下
        //语义一致性检测：对观测的语义进行统计，如果出现次数最大的类占总数小于0.8，则认为是不稳定的地图点
        multiObserver++;
        int nMax = 0;
        int nLabelMax = -1;
        for (map<int, int>::iterator mit = sPoint->mLabelObservation.begin();
             mit != sPoint->mLabelObservation.end(); mit++) {
            if (mit->second > nMax) {
                nLabelMax = mit->first;
                nMax = mit->second;
            }
        }
        float ratio = nMax / float(observeTime);
//        cout << "label max : " << nLabelMax << " time: " << nMax << " ratio : " << ratio << endl;

        //删除语义跳变幅度大的地图点
        if (ratio < 0.8) {
            delete sPoint;
            sPoint = nullptr;
            eraseCount++;
            unStableLabel++;
            continue;
        }

        vector<cv::Mat> vlocation = sPoint->mLocationObservation[nLabelMax];
        //踢掉z<0的点
        bool isBehind = false;
        for (auto location: vlocation) {
            if (location.at<float>(2, 0) < 0) {
                isBehind = true;
                behindCount++;
                break;
            }
        }
        if (isBehind) {
            delete sPoint;
            sPoint = nullptr;
            eraseCount++;
            unStableLabel++;
            break;
        }


        //对观测的距离进行统计检查
        vector<float> vdistance;
//        cout << "disttance： ";
        for (auto pt:vlocation) {
            float dist = sqrt(pt.dot(pt));
            vdistance.push_back(dist);
//            cout << dist << ", ";
        }
//        cout << endl;

        vector<float> orderedDistance = vdistance;
        std::sort(orderedDistance.begin(), orderedDistance.end());
        float difference = *(orderedDistance.rbegin()) - *(orderedDistance.begin());
//        cout << "  difference: " << difference << endl;
        int num = orderedDistance.size();

        if (difference > distanceThreshold) {
            unStableDistance++;
            delete sPoint;
            sPoint = nullptr;
            eraseCount++;
            continue;
        }
        float finalDis = orderedDistance[num / 2];

//        cout<<"final distance: "<<finalDis<<endl;
        for (int i = 0; i < vdistance.size(); i++) {
            if (fabs(finalDis - vdistance[i]) < halfDistanceThresh * 0.5) {
                //更新点的状态和距离,这里小于一米的随便取了一个，可以进行更精细的处理。
                sPoint->mLocation = vlocation[i];
                sPoint->mLabel = nLabelMax;
                sPoint->mSatus = STABLE;
                stableCount++;
                cv::Mat worldCorrdinate = r * vlocation[i] + t;
                LabelPoint lpoint;
                lpoint.pt3d = worldCorrdinate.clone();
                lpoint.label = nLabelMax;
                mpSemanticMapper->AddMapPoint(lpoint);
//            printf("A map point mature , distance: %f,lable %d",finalDis,nLabelMax);
                break;
            }
        }
    }
    mpSemanticMapper->SetPointUpdateStatus(true);
    cout << "semantic map point total: " << total;
    printf(" small osbserve : %d, multiObserver: %d \n \t"
           "stableCount: %d ,unStableLabel: %d ,unStableDistance: %d, eraseCount: %d, point behind count: %d \n",
           observingCount, multiObserver, stableCount, unStableLabel, unStableDistance, eraseCount, behindCount);

}

void CloudPointNode::PassInformationFromNode(CloudPointNode *lastNode)
{
    int outRanger = 0;
    int behindCount = 0;
    cv::Mat TLastToCur = Tcw * lastNode->Tcw.inv();
    cv::Mat rotationLastToCur = TLastToCur.rowRange(0, 3).colRange(0, 3);
    cv::Mat translationLastToCur = TLastToCur.rowRange(0, 3).col(3);

    for (auto &pSPoint: lastNode->mvpMapPoints) {
        if (pSPoint == nullptr)
            continue;
        if (pSPoint->mSatus == STABLE) {
            delete pSPoint;
            pSPoint = nullptr;
            continue;
        }
        //由于在pass information 之前一定会merge point cloud ,理论上merge之后，point只会存在stable，或者observing两种状态。
        if (pSPoint->mSatus == UNDETERMINE) {
            cerr << "Have semantic point undetermined after judge ,which should not happen!" << endl;
            exit(-1);
        }

        //将点的坐标从上个节点的所有观测，传到当前节点,

        for (map<int, int>::iterator mit = pSPoint->mLabelObservation.begin();
             mit != pSPoint->mLabelObservation.end(); mit++) {
            int label = mit->first;
            if(mit->second == 0)
                continue;

            std::vector<cv::Mat> vLocations = pSPoint->mLocationObservation[label];
            for (auto& pt3d: vLocations) {
                cv::Mat pointInCurxyz = rotationLastToCur * pt3d + translationLastToCur;
                //在后面的点,这种点是由于之前观测不足没有被融合，现在又在相机身后了
                if (pointInCurxyz.at<float>(2, 0) < 0) {
                    behindCount++;
                    continue;
                }
                cv::Mat pointInCuruv = K * pointInCurxyz;
                pointInCuruv /= pointInCuruv.at<float>(2, 0);
                int curIndex = std::round(pointInCuruv.at<float>(1, 0)) * mnFrameWidth +
                               std::round(pointInCuruv.at<float>(0, 0));
                if (curIndex > (mnFrameWidth - 1) * (mnFrameHeight - 1) || curIndex < 0) {
                    outRanger++;
                    continue;
                }
                if (mvpMapPoints[curIndex] == nullptr) {
                    SemanticMapPoint *pNewSPoint = new SemanticMapPoint(pointInCurxyz, label);
                    pNewSPoint->mSatus = OBSERVING;
                    mvpMapPoints[curIndex] = pNewSPoint;
                } else {
                    if(mvpMapPoints[curIndex]->mLabel ==label){
                        mvpMapPoints[curIndex]->AssertObservation(pointInCurxyz, label);
                    }
                };

            }
        }

    }
    printf("CloudPointNode PassInformationFromNode, behind point count : %d \n",behindCount);
    //清空
    for (auto &pSPoint: lastNode->mvpMapPoints)
        delete pSPoint;
}

}