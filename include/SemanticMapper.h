#ifndef SEMANTIC_MAPPER
#define SEMANTIC_MAPPER

#include "System.h"
//#include "SemanticMapPoint.h"

namespace ORB_SLAM2{

class System;
class Tracking;
//class SemanticMapPoint;
//struct LabelPoint;

enum eSPointStatus{
    UNDETERMINE = 0,
    OBSERVING,
    STABLE
};

class SemanticMapPoint{

public:
//    SemanticMapPoint(){};
    SemanticMapPoint(const cv::Mat & pt3d,int label):
    mnobserveTime(1),mLabel(label),mSatus(UNDETERMINE)
    {
        pt3d.copyTo(mLocation);
        mLocationObservation[label].push_back(pt3d);
        mLabelObservation[label]++;
    }
    inline void AssertObservation(const cv::Mat & pt3d,int label)
    {
        mLocationObservation[label].push_back(pt3d);
        mLabelObservation[label]++;
        mnobserveTime++;
    }
    inline void AssertObservation(SemanticMapPoint* pSPoint)
    {
        mLocationObservation[pSPoint->mLabel].push_back(pSPoint->mLocation);
        mLabelObservation[pSPoint->mLabel]++;
        mnobserveTime++;
        mSatus = pSPoint->mSatus;
    }
    inline int GetObervetime() {return mnobserveTime;}
    inline void SetStatus(eSPointStatus status){mSatus = status;}

public:
    cv::Mat mLocation;
    int mLabel;
    eSPointStatus mSatus;
    std::map<int, int > mLabelObservation;
    std::map<int,std::vector<cv::Mat> >mLocationObservation;
    int mnobserveTime;
};

//感觉应该写成KeyFrame。。。
class SemanticMapper;

class CloudPointNode{
public:
    CloudPointNode(){}
    CloudPointNode(SemanticMapper* mapper,const std::vector<LabelPoint>& labelPoints,const cv::Mat& Tcw_,
                   const cv::Mat& k_,const int& h,const int& w);
    void PassInformationFromNode(CloudPointNode* lasNode);
    void UpdatePointCloudStatus();
public:
    cv::Mat Tcw;
    cv::Mat K;
    std::vector<SemanticMapPoint*> mvpMapPoints;
    int mnFrameWidth;
    int mnFrameHeight;
    SemanticMapper* mpSemanticMapper;
};


class SemanticMapper{

public:
    SemanticMapper(System* pSys,Tracking* pTracking,const string &strSettingPath);
    void UpdateFrame(Frame *pFrame);
    cv::Mat DrawColorMap(cv::Mat& input);
//    void SegmentRefine();  //这个暂时没有用处，因为主要是片状噪音，用语义难以消除
    void SavePointCloud();
    void Run();
    float getAverage(const cv::Mat& img,int step,int u,int v,int& count);
    void UpdateReferencePointCloud(Frame* pFrame);
    void DrawCurrentCamera();
    void DrawAllSemanticPoint();
    void SetPointUpdateStatus(bool status);
    void AddMapPoint(LabelPoint& lPoint);
    void DrawCurrentFramePointCloud();
    std::vector<LabelPoint> mvSemanticMapPoints;
    std::mutex mMutexSemanticMapPoints;

    std::mutex mMutexPointUpdate;
    bool mbPointStatusUpdate;

private:
//    vector<LabelPoint> mvSemiDensePoints;
    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
    // 1/fps in ms
    double mT;
//    std::mutex mMutexPoints;
    Tracking* mpTracker;
    System* mpSystem;

    std::mutex mMutexDisparity;
    cv::Mat mImDisparityOrigin;

    std::mutex mMutexSegment;
    cv::Mat mImSegmentOrign;

    cv::Mat mCameraPose;
    std::mutex mMutexCamera;

    int mnFrameWidth;
    int mnFrameHeight;

    Frame* mCurrentFrame;
    std::vector<LabelPoint> mCurrentPointCloud;
    bool mbPointCloudUdate;
    std::mutex mMutexCurrentFrame;
//    std::list<SemanticMapPoint*> mvAllSemanticMappoint;

    CloudPointNode* mCurrentReferenceNode;
    bool mbNeedNewNode;

    int mPointSize;



};

}

#endif