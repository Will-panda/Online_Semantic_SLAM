#include "SemanticMapper.h"
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace ORB_SLAM2
{

SemanticMapper::SemanticMapper(System *pSystem, Tracking *pTracking, FrameDrawer *pFrameDrawer,
                               const string &strSettingPath) : mpTracker(pTracking), mpSystem(pSystem),
                                                               mpFrameDrawer(pFrameDrawer),mbNeedNewNode(false),
                                                               mReferenceNode(nullptr), mbPointCloudUdate(false),
                                                               mbPointStatusUpdate(false),mbFinish(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fps = fSettings["Camera.fps"];
    if (fps < 1)
        fps = 30;
    mT = 1e3 / fps;
    mK = mpTracker->mK.clone();
    mbf = mpTracker->mbf;
    fx = mK.at<float>(0, 0);
    fy = mK.at<float>(1, 1);
    cx = mK.at<float>(0,2);
    cy = mK.at<float>(1,2);
    invfx = 1.0f / fx;
    invfy = 1.0f / fy;
    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
    mPointSize = fSettings["Viewer.PointSize"];

}

void SemanticMapper::UpdateFrame(Frame *pFrame)
{
    std::unique_lock<mutex> lock(mMutexFrameDataBuffer);
    FrameDataBuffer *curData = new FrameDataBuffer(pFrame->mnId, pFrame->mTcw, pFrame->mImDisparityLeft,
                                                   pFrame->mImSegment, pFrame->mvbValidByGrandient);
    mlDataBuffer.push_back(curData);
}

void SemanticMapper::ProcessNewData()
{
    TicToc timeCost;
    if (mpCurrentData == nullptr)
    {
        cerr << "CurrentData is empty,something went wrong!" << endl;
        exit(-1);
    }
    cout << "Process Data buffer: " << mpCurrentData->nId << endl;
    if (mbNeedNewNode)
    {
        timeCost.Tic();
        PointCloudNode *curNode = new PointCloudNode(this, mpCurrentData);
        mpCurrentData = nullptr;
        cout << "Making PointCloud cost : " << timeCost.Toc() << " ms" << endl;
        timeCost.Tic();
        if (mReferenceNode != nullptr)
        {
            curNode->PassInformationByNode(mReferenceNode);
        }
        cout << "Passing PointCloud by Node cost : " << timeCost.Toc() << " ms" << endl;
        PointCloudNode *lastNode = mReferenceNode;
        delete lastNode;
        mReferenceNode = curNode;
        mbNeedNewNode = false;
    } else
    {
        //initialize node
        if (mpCurrentData->nId == 0)
        {

            timeCost.Tic();
            PointCloudNode *curNode = new PointCloudNode(this, mpCurrentData);
            mpCurrentData = nullptr;
            cout << "Making PointCloud cost : " << timeCost.Toc() << " ms" << endl;
            mReferenceNode = curNode;
        } else
        {
            TicToc timeCost;
            UpdateReferencePointCloud(mpCurrentData);
            mpCurrentData = nullptr;
            cout << "Passing PointCloud by Frame cost : " << timeCost.Toc() << " ms" << endl;

        }
    }

}

void SemanticMapper::Run()
{
    while (1)
    {
        bool bNewFrameCome = false;
        {
            std::unique_lock<mutex> lock(mMutexFrameDataBuffer);
            if (!mlDataBuffer.empty())
            {
                mpCurrentData = mlDataBuffer.front();
                mlDataBuffer.pop_front();
                bNewFrameCome = true;

            } else
                bNewFrameCome = false;


        }

        if (bNewFrameCome)
        {
            ProcessNewData();
        }
        if(mbFinish)
            break;
        usleep(3000);
    }
}

void SemanticMapper::DrawAllSemanticPoint()
{
    std::unique_lock<mutex> lock(mMutexSemanticMapPoints);
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    for (auto &p: mvSemanticMapPoints)
    {
        int flag = p->label;
        switch (flag)
        {
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
        glVertex3f(p->pt3d.at<float>(0, 0), p->pt3d.at<float>(1, 0), p->pt3d.at<float>(2, 0));
    }
    glEnd();
}

void SemanticMapper::SetPointUpdateStatus(bool status)
{
    std::unique_lock<mutex> lock(mMutexPointUpdate);
    mbPointStatusUpdate = status;
}

cv::Mat SemanticMapper::DrawColorMap(cv::Mat &input)
{
    if (input.empty())
        return cv::Mat(480, 480, CV_8UC3, cv::Scalar::all(0));
    cv::Mat mImDisparity = input.clone();

    cv::Mat disparityVize = cv::Mat(input.size(), CV_8UC1, cv::Scalar(0));
    float maxVal = 0;
    for (int i = 0; i < mImDisparity.rows; ++i)
    {
        for (int j = 0; j < mImDisparity.cols; ++j)
        {
            if (maxVal < mImDisparity.at<float>(i, j))
            {
                maxVal = mImDisparity.at<float>(i, j);
            }
        }
    }
    for (int i = 0; i < mImDisparity.rows; ++i)
    {
        for (int j = 0; j < mImDisparity.cols; ++j)
        {
            float valf = 255.0 * mImDisparity.at<float>(i, j) / maxVal;

//            if (mImDebugMaxGrad.at<float>(i, j) > minUsedGrad) {
//                if (mImDebugSegment.at<uchar>(i, j) == ROAD ||
//                    mImDebugSegment.at<uchar>(i, j) == SIDEWAILK ||
//                    mImDebugSegment.at<uchar>(i, j) == POLE)
            disparityVize.at<uchar>(i, j) = uchar(valf);

        }
    }

    cv::Mat imDisparitycolor;
    cv::applyColorMap(disparityVize, imDisparitycolor, cv::COLORMAP_JET);
    return imDisparitycolor;
}


void SemanticMapper::UpdateReferencePointCloud(FrameDataBuffer *pFrameData)
{
    const int minObserTime = 4;
    int outRanger = 0;
    int totalCur = 0;
    int holeLabel = 0;
    int unmatchLabel = 0;
    int trackedClose = 0;
    int out40 = 0;
    int stableCount = 0;
    std::vector<SemanticMapPoint *> &refPointCloud = mReferenceNode->mvpMapPoints;
    cv::Mat refTcw = mReferenceNode->mTcw.clone();
    cv::Mat refTwc = refTcw.inv();
    cv::Mat rRtoW = refTwc.rowRange(0, 3).colRange(0, 3);
    cv::Mat tRtoW = refTwc.rowRange(0, 3).col(3);

    cv::Mat curTcw = pFrameData->tcw.clone();
    //calculate ralive transpertation
    cv::Mat curToRef = refTcw * curTcw.inv();
    cv::Mat rotationCurToRef = curToRef.rowRange(0, 3).colRange(0, 3);
    cv::Mat translationCurToRef = curToRef.rowRange(0, 3).col(3);

    cv::Mat disparityOrigin = pFrameData->disparity;
    float *pDis = (float *) disparityOrigin.data;
    cv::Mat segmentOrign = pFrameData->segment;
    uchar *pflag = segmentOrign.data;
    std::vector<bool> vVliadByGradient = pFrameData->mvbValid;
    //calculate semidense 3d point
    int total = disparityOrigin.total();

    const int rowStart = disparityOrigin.rows * 0.33;
    const int rowEnd = disparityOrigin.rows - 5;
    const int colStart = 5;
    const int colEnd = disparityOrigin.cols - 5;
    for (int v = rowStart; v < rowEnd; v++)
    {
        int rowShift = v * disparityOrigin.cols;
        for (int u = colStart; u < colEnd; u++)
        {
            //valid by gradient
            int index = rowShift + u;
            if (!vVliadByGradient[index])
                continue;

            //only use some flag
            int curFlag = pflag[index];
            if (curFlag == ROAD || curFlag == SIDEWAILK || curFlag == POLE)
            {
                float disp = pDis[index];
                float zc = mbf / disp;
                if (zc > 0)
                {
                    totalCur++;
                    float xc = (u - cx) * zc * invfx;
                    float yc = (v - cy) * zc * invfy;
                    cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << xc, yc, zc);
                    cv::Mat x3Dr = rotationCurToRef * x3Dc + translationCurToRef;
                    cv::Mat x2dr = mK * x3Dr;
                    x2dr /= x2dr.at<float>(2, 0);
                    if (x3Dr.at<float>(2, 0) > 40)
                    {
                        out40++;
                        continue;
                    }
                    int x = int(x2dr.at<float>(0, 0) + 0.5f);
                    if (x < colStart || x > colEnd)
                    {
                        outRanger++;
                        continue;
                    }

                    int y = int(x2dr.at<float>(1, 0) + 0.5f);
                    if (y < rowStart || y > rowEnd)
                    {
                        outRanger++;
                        continue;
                    }

                    int refIndex = y * disparityOrigin.cols + x;
                    if (x3Dr.at<float>(2, 0) < mpTracker->GetDepthThreshold())
                        trackedClose++;

                    if (refPointCloud[refIndex] == nullptr)
                    {
                        holeLabel++;
                        SemanticMapPoint *pSMapPoint = new SemanticMapPoint(x3Dr, curFlag);
                        refPointCloud[refIndex] == pSMapPoint;
                    } else if (refPointCloud[refIndex]->GetOberveTime() > minObserTime)
                    {

                        bool goodPoint = refPointCloud[refIndex]->MergeObservation();
                        if (goodPoint)
                        {
                            cv::Mat xw = rRtoW * refPointCloud[refIndex]->GetLocation() + tRtoW;
                            int label = refPointCloud[refIndex]->GetLabel();
                            AddMapPoint(xw, label);
                            stableCount++;
                        }
                        delete refPointCloud[refIndex];
                        refPointCloud[refIndex] = nullptr;

                    } else if (refPointCloud[refIndex]->GetLabel() != curFlag)
                    {
                        unmatchLabel++;
                        delete refPointCloud[refIndex];
                        refPointCloud[refIndex] = nullptr;
                        continue;
                    } else
                        refPointCloud[refIndex]->AssertObservation(x3Dr);
                }
            }

        }
    }

    float outLabelRatio = holeLabel / float(totalCur);
    float trackedCloseRatio = trackedClose / float(totalCur);
    float unmatchLabelRatio = unmatchLabel / float(totalCur);
    if (trackedCloseRatio < 0.7)
    {
        mbNeedNewNode = true;
    }
    cout << "Frame " << pFrameData->nId << " total valid: " << totalCur << " out of range point : " << outRanger
         << endl <<
         " \t holeLabelRatio: " << " (" << holeLabel << ")" << outLabelRatio << " unmatchLabelRatio: " << " ("
         << unmatchLabel << ") " << unmatchLabelRatio << endl <<"statble: "<<stableCount<<
         " \t trackedCloseRatio: " << " (" << trackedClose << ")" << trackedCloseRatio <<
         endl;
    printf("over 40m points count: %d ,%f \n", out40,float(out40) / totalCur);
    delete pFrameData;

//    printf("close distance is : %f \n", mpTracker->GetDepthThreshold());

}

void SemanticMapper::AddMapPoint(const cv::Mat &w3d, int label)
{
    LabelPoint *pPoint = new LabelPoint;
    pPoint->pt3d = w3d.clone();
    pPoint->label = label;
    std::unique_lock<mutex> lock(mMutexSemanticMapPoints);
    mvSemanticMapPoints.push_back(pPoint);
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


PointCloudNode::PointCloudNode(SemanticMapper *pMapper, FrameDataBuffer *pFrameData) :
        mTcw(pFrameData->tcw.clone()), mpSemanticMapper(pMapper)
{
    mK = mpSemanticMapper->mK.clone();
    mbf = mpSemanticMapper->mbf;

    cv::Mat disparityOrigin = pFrameData->disparity;
    float *pDis = (float *) disparityOrigin.data;
    mnFrameWidth = disparityOrigin.cols;
    mnFrameHeight = disparityOrigin.rows;
    cv::Mat segmentOrign = pFrameData->segment;
    uchar *pflag = segmentOrign.data;
    std::vector<bool> vVliadByGradient = pFrameData->mvbValid;
    //calculate semidense 3d point
    int total = disparityOrigin.total();
    mvpMapPoints = std::vector<SemanticMapPoint*>(total, nullptr);

    mnRowStart = disparityOrigin.rows * 0.33;
    mnRowEnd = disparityOrigin.rows - 5;
    mnColStart = 5;
    mnColEnd = disparityOrigin.cols - 5;

    for (int v = mnRowStart; v < mnRowEnd; v++)
    {
        int rowShift = v * disparityOrigin.cols;
        for (int u = mnColStart; u < mnColEnd; u++)
        {
            //valid by gradient
            int index = rowShift + u;
            if (!vVliadByGradient[index])
                continue;

            //only use some flag
            int flag = pflag[index];
            if (flag == ROAD || flag == SIDEWAILK ||
                flag == POLE)
            {
                float disp = pDis[index];
                float zc = mbf / disp;
                if (zc > 0)
                {

                    float xc = (u - pMapper->cx) * zc * pMapper->invfx;
                    float yc = (v - pMapper->cy) * zc * pMapper->invfy;
                    cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << xc, yc, zc);
                    SemanticMapPoint *pSMapPoint = new SemanticMapPoint(x3Dc, flag);
                    mvpMapPoints.push_back(pSMapPoint);
                }
            }

        }
    }
    delete pFrameData;
}

void PointCloudNode::PassInformationByNode(PointCloudNode *lastNode)
{
    int outRanger = 0;
    int behindCount = 0;
    cv::Mat TLastToCur = mTcw * lastNode->mTcw.inv();
    cv::Mat rotationLastToCur = TLastToCur.rowRange(0, 3).colRange(0, 3);
    cv::Mat translationLastToCur = TLastToCur.rowRange(0, 3).col(3);

    for (auto &pSPoint: lastNode->mvpMapPoints)
    {
        if (pSPoint == nullptr)
            continue;

        int lastLabel = pSPoint->GetLabel();
        cv::Mat last3d = pSPoint->GetLocation();

        cv::Mat ptCurxyz = rotationLastToCur * last3d + translationLastToCur;
        if (ptCurxyz.at<float>(2, 0) < 0)
        {
            behindCount++;
            delete pSPoint;
            pSPoint = nullptr;
            continue;
        }
        cv::Mat ptCuruv = mK * ptCurxyz;
        ptCuruv /= ptCuruv.at<float>(2, 0);

        int x = int(ptCuruv.at<float>(0, 0) + 0.5f);
        if (x < mnColStart || x > mnColEnd)
        {
            outRanger++;
            continue;
        }
        int y = int(ptCuruv.at<float>(1, 0) + 0.5f);
        if (y < mnRowStart || y > mnRowEnd)
        {
            outRanger++;
            continue;
        }
        int curIndex = y * mnFrameWidth + x;

        if (mvpMapPoints[curIndex] == nullptr)
        {
            for (auto &pt: pSPoint->mvLocationObserve)
            {
                pt = rotationLastToCur * pt + translationLastToCur;
            }
            mvpMapPoints[curIndex] = pSPoint;
        } else if (mvpMapPoints[curIndex]->GetLabel() == lastLabel)
        {
            mvpMapPoints[curIndex]->AssertObservation(ptCurxyz);
        } else
        {
            delete pSPoint;
            pSPoint == nullptr;
            delete mvpMapPoints[curIndex];
            mvpMapPoints[curIndex] = nullptr;
        }
    }
//    printf("CloudPointNode PassInformationFromNode, behind point count : %d \n", behindCount);
}

}