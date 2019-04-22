//
// Created by liwenqiang on 4/22/19.
//
#include "opencv2/opencv.hpp"
#include <string>
#include <vector>
#include <iostream>
#include "TicToc.h"

using namespace cv;
using namespace std;

void ComputeKeyPointsOctTree(const cv::Mat &image, std::vector<cv::KeyPoint> &allKeypoints);

void ComputeKeyPointsWhoutOctTree(const cv::Mat &image, std::vector<cv::KeyPoint> &allKeypoints);

void computeOrientation(const Mat &image, vector<KeyPoint> &keypoints, const vector<int> &umax);

float IC_Angle(const Mat &image, Point2f pt, const vector<int> &u_max);

void PreCompute();

cv::Mat DrawSegment(const cv::Mat& imageSeg1);

vector<cv::KeyPoint> DistributeOctTree(const vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
                                                     const int &maxX, const int &minY, const int &maxY, const int &N);
class ExtractorNode
{
public:
  ExtractorNode() : bNoMore(false)
  {}

  void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);


  std::vector<cv::KeyPoint> vKeys;
  cv::Point2i UL, UR, BL, BR;
  std::list<ExtractorNode>::iterator lit;
  bool bNoMore;
};

vector<int> umax;
int HALF_PATCH_SIZE = 15;

int main(int argc, char **argv)
{

    string pathToSequence = argv[1];
    int imgIndex1 = 355;
    int imgIndex2 = 357;
    char imgName1[12];
    char imgName2[12];
    sprintf(imgName1, "%06d.png", imgIndex1);
    sprintf(imgName2, "%06d.png", imgIndex2);

    cv::Mat image1 = cv::imread(pathToSequence + "/image_2/" + imgName1);
    cv::Mat imageSeg1 = cv::imread(pathToSequence + "/segment_l/" + imgName1);
    cv::Mat image2 = cv::imread(pathToSequence + "/image_2/" + imgName2);
    cv::Mat imageSeg2 = cv::imread(pathToSequence + "/segment_l/" + imgName2);

    PreCompute();

    //debug segment
    cv::Mat segColor1 = DrawSegment(imageSeg1);
    namedWindow("feature1 Segment ", 0);
    cv::imshow("feature1 Segment ",segColor1);
    cv::Mat segColor2 = DrawSegment(imageSeg2);
    namedWindow("feature2 Segment ", 0);
    cv::imshow("feature2 Segment ",segColor2);


    cvtColor(image1, image1, CV_RGB2GRAY);
    vector<cv::KeyPoint> keypoints1;
    cv::Mat imageShow1;
    ComputeKeyPointsWhoutOctTree(image1, keypoints1);
    cv::drawKeypoints(image1, keypoints1, imageShow1, cv::Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    namedWindow("feature1 original", 0);
    cv::imshow("feature1 original", imageShow1);
    cout<<"feature1 original key point: "<<keypoints1.size()<<endl;

    cv::Mat imageShowOct1;
    vector<cv::KeyPoint> keypointsOct1;
    ComputeKeyPointsOctTree(image1, keypointsOct1);
    cv::drawKeypoints(image1, keypointsOct1, imageShowOct1, cv::Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);



    namedWindow("feature1 with OctTree", 0);
    cv::imshow("feature1 with OctTree", imageShowOct1);
    cout<<"feature1 Octree key point: "<<keypointsOct1.size()<<endl;

    cv::Mat imageShowOct2;
    cvtColor(image2, image2, CV_RGB2GRAY);
    vector<cv::KeyPoint> keypointsOct2;
    ComputeKeyPointsOctTree(image2, keypointsOct2);
    cv::drawKeypoints(image2, keypointsOct2, imageShowOct2, cv::Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    namedWindow("feature2 with OctTree", 0);
    cv::imshow("feature2 with OctTree", imageShowOct2);

    cv::waitKey(0);
    return 0;
}

void PreCompute()
{
    //This is for orientation
    // pre-compute the end of a row in a circular patch
    umax.resize(HALF_PATCH_SIZE + 1);

    int v, v0, vmax = cvFloor(
            HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);//if have decimal party ,vmax and vmin will be the same.
    int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
//    printf("vmax %d ,vim %d \n",vmax,vmin);//actually is the same as patch siaze = 15;
    //is there any difference from compute a squre patch directly?
    const double hp2 = HALF_PATCH_SIZE * HALF_PATCH_SIZE;
    for (v = 0; v <= vmax; ++v)
        umax[v] = cvRound(sqrt(hp2 - v * v));
    // Make sure we are symmetric
    for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1])
            ++v0;
        umax[v] = v0;
        ++v0;
    }
}

void ComputeKeyPointsWhoutOctTree(const cv::Mat &image, std::vector<cv::KeyPoint> &allKeypoints)
{
    cv::FAST(image, allKeypoints, 20);
    computeOrientation(image, allKeypoints,umax);
    int nkps = allKeypoints.size();
    for (int i = 0; i < nkps; i++)
    {
        allKeypoints[i].octave = 0;
        allKeypoints[i].size = 31;
    }
}

cv::Mat DrawSegment(const cv::Mat& imageSeg1)
{
    enum eSegmentLable{
    ROAD = 0, //点很多,分割的效果比较好，可用
    SIDEWAILK = 1,//可用
    BUILDING = 2,//远处不是很准，可用于场景重建，可用。
//    WALL = 3,  //有时会把围栏当做墙（05），这个比较不准，
//    FENCE = 4, //垃圾桶会被当做Fence，大部分能割出来，不全
            POLE = 5,  //比较有特点，可以尝试踢掉视差图中的outliner
//    TRAFFIC_LIGHT=6, //太弱，建议去掉
//    TRAFFIC_SIGN =7,//交通牌，不是很稳定，数量比较稀少，暂时不考虑加入。
//    VEGETATION = 8,//主要是树，由于树比较容易变化，不建议用于地图场景
//    TERRAIN = 9,  //这个很少，也不稳定，建议去掉
            SKY = 10,
    PERSION = 11,
    RIDER=12,
    CAR=13,//可用
    TRUCK=14, //分割非常不稳定，建议与car统一，可用
    BUS=15,
    TRAIN=16,
    MOTORCYCLE=17,
    BICYCLE=18
    };
    cv::Mat debugSegmentVize = cv::Mat(imageSeg1.size(), CV_8UC3, cv::Scalar::all(0));
    for (int v = 0; v < imageSeg1.rows; v++)
    {
        for (int u = 0; u < imageSeg1.cols; u++)
        {
            uchar lable = imageSeg1.at<uchar>(v, u);
            if (lable == SKY)
                debugSegmentVize.at<cv::Vec3b>(v, u) = cv::Vec3b(70, 130, 180);
            if (lable == ROAD)
                debugSegmentVize.at<cv::Vec3b>(v, u) = cv::Vec3b(128, 54, 128);
            if (lable == CAR || lable == TRUCK ||
                lable == BUS || lable == TRAIN ||
                lable == MOTORCYCLE || lable == BICYCLE)
                debugSegmentVize.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 0, 142);
            if (lable == PERSION || lable == RIDER)
                debugSegmentVize.at<cv::Vec3b>(v, u) = cv::Vec3b(220, 20, 60);
            if (lable == SIDEWAILK)
                debugSegmentVize.at<cv::Vec3b>(v, u) = cv::Vec3b(244, 35, 232);
            if (lable == POLE)
                debugSegmentVize.at<cv::Vec3b>(v, u) = cv::Vec3b(220, 220, 0);
        }
    }
    return  debugSegmentVize;
}

void ComputeKeyPointsOctTree(const cv::Mat &image, std::vector<cv::KeyPoint> &allKeypoints)
{
    int EDGE_THRESHOLD = 19;
    int nfeatures = 2000;
    const float W = 30;
    int iniThFAST = 20;
    int minThFAST = 10;

    const int minBorderX = EDGE_THRESHOLD - 3;
    const int minBorderY = minBorderX;
    const int maxBorderX = image.cols - EDGE_THRESHOLD + 3;
    const int maxBorderY = image.rows - EDGE_THRESHOLD + 3;

    vector<cv::KeyPoint> vToDistributeKeys;  //
    vToDistributeKeys.reserve(nfeatures * 10);

    const float width = (maxBorderX - minBorderX);
    const float height = (maxBorderY - minBorderY);

    const int nCols = width / W;
    const int nRows = height / W;
    const int wCell = ceil(width / nCols);
    const int hCell = ceil(height / nRows);

    for (int i = 0; i < nRows; i++)
    {
        const float iniY = minBorderY + i * hCell;
        float maxY = iniY + hCell + 6;

        if (iniY >= maxBorderY - 3)
            continue;
        if (maxY > maxBorderY)
            maxY = maxBorderY;

        for (int j = 0; j < nCols; j++)
        {
            const float iniX = minBorderX + j * wCell;
            float maxX = iniX + wCell + 6;
            if (iniX >= maxBorderX - 6)
                continue;
            if (maxX > maxBorderX)
                maxX = maxBorderX;

            vector<cv::KeyPoint> vKeysCell;
            FAST(image.rowRange(iniY, maxY).colRange(iniX, maxX),
                 vKeysCell, iniThFAST, true);

            if (vKeysCell.empty())
            {
                FAST(image.rowRange(iniY, maxY).colRange(iniX, maxX),
                     vKeysCell, minThFAST, true);
            }

            if (!vKeysCell.empty())
            {
                for (vector<cv::KeyPoint>::iterator vit = vKeysCell.begin(); vit != vKeysCell.end(); vit++)
                {
                    (*vit).pt.x += j * wCell;
                    (*vit).pt.y += i * hCell;
                    vToDistributeKeys.push_back(*vit);
                }
            }
        }
    }


    allKeypoints = DistributeOctTree(vToDistributeKeys, minBorderX, maxBorderX,
                                  minBorderY, maxBorderY, 200);
    const int scaledPatchSize = 31 * 1;
//
//    // Add border to coordinates and scale information
    const int nkps = allKeypoints.size();
    for (int i = 0; i < nkps; i++)
    {
        allKeypoints[i].pt.x += minBorderX;
        allKeypoints[i].pt.y += minBorderY;
        allKeypoints[i].octave = 0;
        allKeypoints[i].size = scaledPatchSize;
    }
//
//    // compute orientations
    computeOrientation(image, allKeypoints, umax);
}

float IC_Angle(const Mat &image, Point2f pt, const vector<int> &u_max)
{
    int m_01 = 0, m_10 = 0;
    const uchar *center = &image.at<uchar>(cvRound(pt.y), cvRound(pt.x));

    // Treat the center line differently, v=0
    for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
        m_10 += u * center[u];

    // Go line by line in the circular patch
    int step = (int) image.step1();
    for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
    {
        // Proceed over the two lines
        int v_sum = 0;
        int d = u_max[v];
        for (int u = -d; u <= d; ++u)
        {
            int val_plus = center[u + v * step], val_minus = center[u - v * step];
            v_sum += (val_plus - val_minus);
            m_10 += u * (val_plus + val_minus);
        }
        m_01 += v * v_sum;
    }

    return fastAtan2((float) m_01, (float) m_10);
}

void computeOrientation(const Mat &image, vector<KeyPoint> &keypoints, const vector<int> &umax)
{
    for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                 keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
    {
        keypoint->angle = IC_Angle(image, keypoint->pt, umax);
    }
}

vector<cv::KeyPoint> DistributeOctTree(const vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
                                                     const int &maxX, const int &minY, const int &maxY, const int &N)
{
    // Compute how many initial nodes,nodes rectangle but close to square enough
    // Node height is maxY - minY
    const int nIni = round(static_cast<float>(maxX - minX) / (maxY - minY));

    //width
    const float hX = static_cast<float>(maxX - minX) / nIni;

    list<ExtractorNode> lNodes;

    vector<ExtractorNode *> vpIniNodes;
    vpIniNodes.resize(nIni);

    for (int i = 0; i < nIni; i++)
    {
        ExtractorNode ni;
        ni.UL = cv::Point2i(hX * static_cast<float>(i), 0);
        ni.UR = cv::Point2i(hX * static_cast<float>(i + 1), 0);
        ni.BL = cv::Point2i(ni.UL.x, maxY - minY);
        ni.BR = cv::Point2i(ni.UR.x, maxY - minY);
        ni.vKeys.reserve(vToDistributeKeys.size());

        lNodes.push_back(ni);
        vpIniNodes[i] = &lNodes.back();
    }

    //Associate points to childs
    for (size_t i = 0; i < vToDistributeKeys.size(); i++)
    {
        const cv::KeyPoint &kp = vToDistributeKeys[i];
        vpIniNodes[kp.pt.x / hX]->vKeys.push_back(kp);
    }

    list<ExtractorNode>::iterator lit = lNodes.begin();

    //node
    while (lit != lNodes.end())
    {
        if (lit->vKeys.size() == 1)
        {
            lit->bNoMore = true;
            lit++;
        } else if (lit->vKeys.empty())
            lit = lNodes.erase(lit);
        else
            lit++;
    }

    bool bFinish = false;

    int iteration = 0;

    vector<pair<int, ExtractorNode *> > vSizeAndPointerToNode;
    vSizeAndPointerToNode.reserve(lNodes.size() * 4);

    while (!bFinish)
    {
        iteration++;

        int prevSize = lNodes.size();

        lit = lNodes.begin();

        int nToExpand = 0;

        vSizeAndPointerToNode.clear();

        while (lit != lNodes.end())
        {
            if (lit->bNoMore)
            {
                // If node only contains one point do not subdivide and continue
                lit++;
                continue;
            } else
            {
                // If more than one point, subdivide
                ExtractorNode n1, n2, n3, n4;
                lit->DivideNode(n1, n2, n3, n4);

                // Add childs if they contain points
                if (n1.vKeys.size() > 0)
                {
                    lNodes.push_front(n1);
                    if (n1.vKeys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
                        lNodes.front().lit = lNodes.begin();//save location in list lNode.
                    }
                }
                if (n2.vKeys.size() > 0)
                {
                    lNodes.push_front(n2);
                    if (n2.vKeys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if (n3.vKeys.size() > 0)
                {
                    lNodes.push_front(n3);
                    if (n3.vKeys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if (n4.vKeys.size() > 0)
                {
                    lNodes.push_front(n4);
                    if (n4.vKeys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }

                lit = lNodes.erase(lit);
                continue;
            }
        }

        // Finish if there are more nodes than required features
        // or all nodes contain just one point
        if ((int) lNodes.size() >= N || (int) lNodes.size() == prevSize)
        {
            bFinish = true;
        } else if (((int) lNodes.size() + nToExpand * 3) > N)
        {

            while (!bFinish)
            {

                prevSize = lNodes.size();

                vector<pair<int, ExtractorNode *> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                vSizeAndPointerToNode.clear();

                sort(vPrevSizeAndPointerToNode.begin(), vPrevSizeAndPointerToNode.end());
                for (int j = vPrevSizeAndPointerToNode.size() - 1; j >= 0; j--)
                {
                    ExtractorNode n1, n2, n3, n4;
                    vPrevSizeAndPointerToNode[j].second->DivideNode(n1, n2, n3, n4);

                    // Add childs if they contain points
                    if (n1.vKeys.size() > 0)
                    {
                        lNodes.push_front(n1);
                        if (n1.vKeys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n2.vKeys.size() > 0)
                    {
                        lNodes.push_front(n2);
                        if (n2.vKeys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n3.vKeys.size() > 0)
                    {
                        lNodes.push_front(n3);
                        if (n3.vKeys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n4.vKeys.size() > 0)
                    {
                        lNodes.push_front(n4);
                        if (n4.vKeys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                    if ((int) lNodes.size() >= N)
                        break;
                }

                if ((int) lNodes.size() >= N || (int) lNodes.size() == prevSize)
                    bFinish = true;

            }
        }
    }

    // Retain the best point in each node
    vector<cv::KeyPoint> vResultKeys;
    vResultKeys.reserve(2000);
    for (list<ExtractorNode>::iterator lit = lNodes.begin(); lit != lNodes.end(); lit++)
    {
        vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;
        cv::KeyPoint *pKP = &vNodeKeys[0];
        float maxResponse = pKP->response;

        for (size_t k = 1; k < vNodeKeys.size(); k++)
        {
            if (vNodeKeys[k].response > maxResponse)
            {
                pKP = &vNodeKeys[k];
                maxResponse = vNodeKeys[k].response;
            }
        }

        vResultKeys.push_back(*pKP);
    }

    return vResultKeys;
}

void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4)
{
    const int halfX = ceil(static_cast<float>(UR.x - UL.x) / 2);
    const int halfY = ceil(static_cast<float>(BR.y - UL.y) / 2);

    //Define boundaries of childs
    n1.UL = UL;
    n1.UR = cv::Point2i(UL.x + halfX, UL.y);
    n1.BL = cv::Point2i(UL.x, UL.y + halfY);
    n1.BR = cv::Point2i(UL.x + halfX, UL.y + halfY);
    n1.vKeys.reserve(vKeys.size());

    n2.UL = n1.UR;
    n2.UR = UR;
    n2.BL = n1.BR;
    n2.BR = cv::Point2i(UR.x, UL.y + halfY);
    n2.vKeys.reserve(vKeys.size());

    n3.UL = n1.BL;
    n3.UR = n1.BR;
    n3.BL = BL;
    n3.BR = cv::Point2i(n1.BR.x, BL.y);
    n3.vKeys.reserve(vKeys.size());

    n4.UL = n3.UR;
    n4.UR = n2.BR;
    n4.BL = n3.BR;
    n4.BR = BR;
    n4.vKeys.reserve(vKeys.size());

    //Associate points to childs
    for (size_t i = 0; i < vKeys.size(); i++)
    {
        const cv::KeyPoint &kp = vKeys[i];
        if (kp.pt.x < n1.UR.x)
        {
            if (kp.pt.y < n1.BR.y)
                n1.vKeys.push_back(kp);
            else
                n3.vKeys.push_back(kp);
        } else if (kp.pt.y < n1.BR.y)
            n2.vKeys.push_back(kp);
        else
            n4.vKeys.push_back(kp);
    }

    if (n1.vKeys.size() == 1)
        n1.bNoMore = true;
    if (n2.vKeys.size() == 1)
        n2.bNoMore = true;
    if (n3.vKeys.size() == 1)
        n3.bNoMore = true;
    if (n4.vKeys.size() == 1)
        n4.bNoMore = true;

}