//
// Created by liwenqiang on 4/1/19.
//

#ifndef OS_SLAM_SEMANTICMAPPOINT_H
#define OS_SLAM_SEMANTICMAPPOINT_H

#include "System.h"

namespace ORB_SLAM2
{
enum eSPointStatus
{
UNDETERMINE = 0,
OBSERVING,
STABLE,
};

class SemanticMapPoint
{

public:
//    SemanticMapPoint(){};
  SemanticMapPoint(const cv::Mat &pt3d, int label);


  void AssertObservation(const cv::Mat &pt3d);

  void AssertObservation(SemanticMapPoint *pSPoint);

  bool MergeObservation();

  inline int GetOberveTime(){ return mnObserveTime; }
  inline eSPointStatus GetStatus()  { return mSatus ; }
  inline int GetLabel()  { return mLabel ; }
  inline cv::Mat GetLocation() { return mLocation.clone();}

  inline void SetLabel(int l){mLabel = l;}
  inline void SetLocation(const cv::Mat& pt3d){ mLocation = pt3d.clone();}
  inline void SetStatus(eSPointStatus status)  { mSatus = status; }

public:
  std::vector<cv::Mat> mvLocationObserve;

private:
  cv::Mat mLocation;
  int mnObserveTime;
  eSPointStatus mSatus;
  int mLabel;


};
}
#endif //OS_SLAM_SEMANTICMAPPOINT_H
