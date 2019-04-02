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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap,const string &strSettingPath):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    int width = fSettings["Camera.width"];
    int height = fSettings["Camera.height"];
    mIm = cv::Mat(height,width,CV_8UC3, cv::Scalar(0,0,0));
    mImDisparity = cv::Mat(height,width,CV_8UC3, cv::Scalar(0));
    mImDebugMaxGrad = cv::Mat(height,width,CV_8UC3, cv::Scalar(0));

}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,255,0),-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}
cv::Mat FrameDrawer::DebugDynamic()
{
    unique_lock<mutex> lock(mMutex);
    return mImDynamicProposal;
}

cv::Mat FrameDrawer::DebugOpticalFlow()
{
    unique_lock<mutex> lock(mMutex);
    return mImOpticalFlowDebug;
}

cv::Mat FrameDrawer::DrawDisparity(){
    int state;
    {
        unique_lock<mutex> lock(mMutex);
        state = mState;
    }

    if(state == Tracking::NO_IMAGES_YET || state == Tracking::SYSTEM_NOT_READY)
        return mImDebugMaxGrad;


    cv::Mat disparityVize = cv::Mat(mImDisparity.size(),CV_8UC1,cv::Scalar(0));
    float maxVal = 0;
    for (int i = 0; i < mImDisparity.rows; ++i) {
        for (int j = 0; j < mImDisparity.cols; ++j) {
            if(maxVal < mImDisparity.at<float>(i,j)){
                maxVal =  mImDisparity.at<float>(i,j);
            }
        }
    }
    //
    for (int i = 0; i < mImDisparity.rows; ++i) {
        for (int j = 0; j < mImDisparity.cols; ++j) {
            float valf = 255.0 * mImDisparity.at<float>(i,j) / maxVal;

            //
            if(mImDebugMaxGrad.at<float>(i,j) > minUsedGrad)
            {
                if(mImDebugSegment.at<uchar>(i,j) == ROAD ||
                   mImDebugSegment.at<uchar>(i,j) == SIDEWAILK||
                   mImDebugSegment.at<uchar>(i,j) == POLE)
                    disparityVize.at<uchar>(i,j) = uchar(valf);
            }
        }
    }
    //
    cv::Mat imDisparitycolor;
    cv::applyColorMap(disparityVize, imDisparitycolor, cv::COLORMAP_JET);
    return imDisparitycolor;
}

cv::Mat FrameDrawer::DebugDrawMaxGradPoint() {
    int state;
    {
        unique_lock<mutex> lock(mMutex);
        state = mState;
    }

    if(state ==Tracking::NO_IMAGES_YET)
        return cv::Mat::zeros(480,720,CV_8UC1);


    cv::Mat debugMaxGrad = mIm.clone();
    if(debugMaxGrad.channels() < 3) {
        cv::cvtColor(debugMaxGrad,debugMaxGrad,CV_GRAY2BGR);
    }
    for(int v = 0; v < mImDebugMaxGrad.rows; v++){
        for(int u = 0; u < mImDebugMaxGrad.cols; u++){
            if(mImDebugMaxGrad.at<float>(v,u) > minUsedGrad){
//                cv::circle(debugMaxGrad,cv::Point(u,v),1,cv::Scalar(0,0,200),-1);
                debugMaxGrad.at<cv::Vec3b>(v,u) = cv::Vec3b(0,0,200);
            }
        }
    }
    return debugMaxGrad;
}

cv::Mat FrameDrawer::DebugDrawSegMent() {
    int state;
    {
        unique_lock<mutex> lock(mMutex);
        state = mState;
    }

    if(state ==Tracking::NO_IMAGES_YET)
        return cv::Mat::zeros(480,720,CV_8UC1);


    assert(mImDebugSegment.type() == CV_8UC1);
    cv::Mat debugSegmentVize = cv::Mat(mImDebugSegment.size(),CV_8UC3,cv::Scalar::all(0));
    for(int v = 0; v < mImDebugSegment.rows; v++){
        for(int u = 0; u < mImDebugSegment.cols; u++){
            uchar lable = mImDebugSegment.at<uchar>(v,u);
            if(lable == SKY)
                debugSegmentVize.at<cv::Vec3b>(v,u) = cv::Vec3b(70,130,180);
            if(lable == ROAD)
                debugSegmentVize.at<cv::Vec3b>(v,u) = cv::Vec3b(128,54,128);
//            if(lable == CAR || lable == TRUCK ||
//                lable == BUS ||lable ==TRAIN  ||
//                lable ==MOTORCYCLE ||lable == BICYCLE)
//                debugSegmentVize.at<cv::Vec3b>(v,u) = cv::Vec3b(0,0,142);
//            if(lable == PERSION || lable == RIDER)
//                debugSegmentVize.at<cv::Vec3b>(v,u) = cv::Vec3b(220,20,60);
            if(lable == SIDEWAILK)
                debugSegmentVize.at<cv::Vec3b>(v,u) = cv::Vec3b(244,35,232);
//            if(lable == TRAFFIC_SIGN)
//                debugSegmentVize.at<cv::Vec3b>(v,u) = cv::Vec3b(220,220,0);
        }
    }
    return debugSegmentVize;
}
void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked << ", Mappable ratio: "<<mnMappableRatio;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    pTracker->mImDebugDynamicProposal.copyTo(mImDynamicProposal);
    pTracker->mImDebugOptiFlow.copyTo(mImOpticalFlowDebug);
    pTracker->mCurrentFrame.mImDisparityLeft.copyTo(mImDisparity);
    pTracker->mCurrentFrame.mGrandientMax.copyTo(mImDebugMaxGrad);
    pTracker->mCurrentFrame.mImSegment.copyTo(mImDebugSegment);

    mnMappableRatio = pTracker->mCurrentFrame.GetMappableRatio();
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mvbOutlier = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;

    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
