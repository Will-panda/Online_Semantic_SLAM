#ifndef SEMANTIC_MAPPER
#define SEMANTIC_MAPPER

#include "System.h"
#include "SemanticMapPoint.h"

namespace ORB_SLAM2
{

class System;

class Tracking;

class FrameDrawer;
//class SemanticMapPoint;
//struct LabelPoint;



struct FrameDataBuffer
{
int nId;
std::vector<bool> mvbValid;
cv::Mat tcw;
cv::Mat disparity;
cv::Mat segment;

FrameDataBuffer(int id, const cv::Mat &t, const cv::Mat &dis, const cv::Mat &seg, const std::vector<bool> &vl) :
        nId(id), tcw(t.clone()), disparity(dis.clone()), mvbValid(vl), segment(seg.clone())
{
}

};


class SemanticMapper;

class SemanticMapPoint;

class PointCloudNode
{
public:
  PointCloudNode()
  {}

  PointCloudNode(SemanticMapper *mapper, FrameDataBuffer *pFrameData);

  void PassInformationByNode(PointCloudNode *lastNode);
//    void UpdatePointCloudStatus();
public:
  cv::Mat mTcw;
  cv::Mat mK;
  std::vector<SemanticMapPoint *> mvpMapPoints;
  int mnRowStart;
  int mnRowEnd;
  int mnColStart;
  int mnColEnd;
  int mnFrameWidth;
  int mnFrameHeight;
  float mbf;
  SemanticMapper *mpSemanticMapper;
};


class SemanticMapper
{

public:
  SemanticMapper(System *pSys, Tracking *pTracking, FrameDrawer *pFrameDrawer, const string &strSettingPath);

  void UpdateFrame(Frame *pFrame);

  void Run();

  void UpdateReferencePointCloud(FrameDataBuffer *pFrameData);

  void ProcessNewData();

  void AddMapPoint(const cv::Mat &w3d, int label);

  void DrawAllSemanticPoint();

  void SavePointCloud();

  void SetPointUpdateStatus(bool status);

  cv::Mat DrawColorMap(cv::Mat &input);

public:
  std::vector<LabelPoint *> mvSemanticMapPoints;
  std::mutex mMutexSemanticMapPoints;
  std::mutex mMutexPointUpdate;
  bool mbPointStatusUpdate;
  cv::Mat mK;
  float mbf;
  float cx, cy;
  float fx, fy;
  float invfx;
  float invfy;
//    int mnFrameWidth;
//    int mnFrameHeight;
private:
//    vector<LabelPoint> mvSemiDensePoints;
  float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
  // 1/fps in ms
  double mT;
//    std::mutex mMutexPoints;
  Tracking *mpTracker;
  FrameDrawer *mpFrameDrawer;

  System *mpSystem;

  std::list<FrameDataBuffer *> mlDataBuffer;
  std::mutex mMutexFrameDataBuffer;
  FrameDataBuffer *mpCurrentData;

  std::vector<LabelPoint> mvCurrentPointCloud;
  bool mbPointCloudUdate;
  std::mutex mMutexCurrentFrame;
//    std::list<SemanticMapPoint*> mvAllSemanticMappoint;

  PointCloudNode *mReferenceNode;
  bool mbNeedNewNode;
  bool mbFinish;
  int mPointSize;


};

}

#endif