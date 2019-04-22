/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM2
{

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

class ORBextractor
{
public:

  enum
  {
  HARRIS_SCORE = 0, FAST_SCORE = 1
  };
  enum eSegmentLable
  {
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
  RIDER = 12,
  CAR = 13,//可用
  TRUCK = 14, //分割非常不稳定，建议与car统一，可用
  BUS = 15,
  TRAIN = 16,
  MOTORCYCLE = 17,
  BICYCLE = 18
  };

  ORBextractor(int nfeatures, float scaleFactor, int nlevels,
               int iniThFAST, int minThFAST);

  ORBextractor(int _nfeatures, float _scaleFactor, int _nlevels,
                             int _iniThFAST, int _minThFAST,int deletDynamic);
  ~ORBextractor()
  {}

  // Compute the ORB features and descriptors on an image.
  // ORB are dispersed on the image using an octree.
  // Mask is ignored in the current implementation.
  void operator()(cv::InputArray image, cv::InputArray mask,
                  std::vector<cv::KeyPoint> &keypoints,
                  cv::OutputArray descriptors);

  int inline GetLevels()
  {
      return nlevels;
  }

  float inline GetScaleFactor()
  {
      return scaleFactor;
  }

  std::vector<float> inline GetScaleFactors()
  {
      return mvScaleFactor;
  }

  std::vector<float> inline GetInverseScaleFactors()
  {
      return mvInvScaleFactor;
  }

  std::vector<float> inline GetScaleSigmaSquares()
  {
      return mvLevelSigma2;
  }

  std::vector<float> inline GetInverseScaleSigmaSquares()
  {
      return mvInvLevelSigma2;
  }

  std::vector<cv::Mat> mvImagePyramid;

protected:

  void ComputePyramid(cv::Mat image);

  void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> > &allKeypoints);

  std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
                                              const int &maxX, const int &minY, const int &maxY, const int &nFeatures,
                                              const int &level);

  void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> > &allKeypoints);

  std::vector<cv::Point> pattern;

  int nfeatures;
  double scaleFactor;
  int nlevels;
  int iniThFAST;
  int minThFAST;

  std::vector<int> mnFeaturesPerLevel;

  std::vector<int> umax;

  std::vector<float> mvScaleFactor;
  std::vector<float> mvInvScaleFactor;
  std::vector<float> mvLevelSigma2;
  std::vector<float> mvInvLevelSigma2;
  int mDeleteDynamic;
};

} //namespace ORB_SLAM

#endif

