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

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

Viewer::Viewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking,
               SemanticMapper *pSmapper,
               const string &strSettingPath) : mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer),
                                               mpTracker(pTracking),
                                               mbFinishRequested(false), mbFinished(true), mbStopped(true),
                                               mbStopRequested(false), mpSmapper(pSmapper)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if (fps < 1)
        fps = 30;
    mT = 1e3 / fps;
/*
 * actually these two value are never used
 * */
//    mImageWidth = fSettings["Camera.width"];
//    mImageHeight = fSettings["Camera.height"];
//    if(mImageWidth<1 || mImageHeight<1)
//    {
//        mImageWidth = 640;
//        mImageHeight = 480;
//    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer", 1024, 768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points", false, true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true);
    pangolin::Var<bool> menuReset("menu.Reset", false, false);
    pangolin::Var<bool> menuPause("menu.Pause", false, true);
    pangolin::Var<bool> menuSemantic("menu.Semantic", false, true);
    pangolin::Var<bool> menuShowFlowOutlier("menu.Show Flow Outlier", false, true);
    pangolin::Var<bool> menuShowFlow("menu.Show Flow ", false, true);
    pangolin::Var<bool> menuShowOrb("menu.Show ORB ", false, true);
    pangolin::Var<bool> menuShowDisparity("menu.Show Disparity ", false, true);
    pangolin::Var<bool> menuShowSemanticColor("menu.Show SemanticColor ", false, true);
    pangolin::Var<bool> menuShowOriginal("menu.Show Color ", false, true);


    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

//    cv::namedWindow("ORB-SLAM2: Current Frame");
//    cv::namedWindow("ORB-SLAM2: Distparity");

//    cv::namedWindow("ORB-SLAM2: SegmentVize");
//    cv::namedWindow("ORB-SLAM2: debugMaxGrad");
    bool bFollow = true;
    bool bLocalizationMode = false;
    bool bPause = false;
    while (1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        if (menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        } else if (menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(
                    pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        } else if (!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if (menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        } else if (!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        if (menuPause && !bPause)
        {
            bPause = true;
//            mpMapDrawer->DrawSemiDenseMapPoints();
            mpSystem->SetPause(true);
        } else if (!menuPause && bPause)
        {
            bPause = false;
            mpSystem->SetPause(false);
        }


        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        mpMapDrawer->DrawCurrentCamera(Twc);

        //Draw keyfame;
        if (menuShowKeyFrames || menuShowGraph)
        {
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);
        }
        //Draw map point
        if (menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        //Draw optical flow outlier
        if (menuShowFlowOutlier)
        {
            cv::namedWindow("OS-SLAM:ShowFlowOutlier", 0);
            cv::Mat imgDynamicDebug = mpFrameDrawer->DebugDynamic();
            cv::imshow("OS-SLAM:ShowFlowOutlier", imgDynamicDebug);
        }

        //Draw optical flow result
        if (menuShowFlow)
        {
            cv::namedWindow("OS-SLAM:Show Flow", 0);
            cv::Mat imgOpticalDebug = mpFrameDrawer->DebugOpticalFlow();
            cv::imshow("OS-SLAM:Show Flow", imgOpticalDebug);
        }

        if (menuShowOrb)
        {
            cv::namedWindow("OS-SLAM:Show ORB", 0);
            cv::Mat imgORB = mpFrameDrawer->DrawFrame();
            cv::imshow("OS-SLAM:Show ORB", imgORB);
        }


        if (menuShowDisparity)
        {
            cv::namedWindow("OS-SLAM: Disparity", 0);
            cv::Mat imgDis = mpFrameDrawer->DrawDisparity();
            cv::imshow("OS-SLAM: Disparity", imgDis);
        }

        if (menuShowSemanticColor)
        {
            cv::namedWindow("OS-SLAM: Semantic Color", 0);
            cv::Mat imgColor = mpFrameDrawer->DebugDrawSegMent();
            cv::imshow("OS-SLAM: Semantic Color", imgColor);
        }
        if(menuShowOriginal)
        {
            cv::namedWindow("OS-SLAM: Original", 0);
            cv::Mat imgColor = mpFrameDrawer->DrawOriginal();
            cv::imshow("OS-SLAM: Original", imgColor);
        }
        //Draw semantic point
        if (menuSemantic)
            mpSmapper->DrawAllSemanticPoint();

        //Draw diaptity Map;

        pangolin::FinishFrame();

//        if ()
//        cv::Mat im = mpFrameDrawer->DrawFrame();
//        cv::Mat distparity  = mpFrameDrawer->DrawDisparity();
//        cv::Mat SegmentVize = mpFrameDrawer->DebugDrawSegMent();
//        cv::Mat debugMaxGrad = mpFrameDrawer->DebugDrawMaxGradPoint();


//        cv::imshow("ORB-SLAM2: Current Frame",im);
//        cv::imshow("ORB-SLAM2: Distparity",distparity);
//        string dispatity_result = "/data/dataset/kitti/kitti_color/result/" + to_string(mpTracker->mCurrentFrame.mnId);
//        cv::imwrite(dispatity_result + ".png",distparity);
        cv::waitKey(mT);

//        cv::imshow("ORB-SLAM2: SegmentVize",SegmentVize);
//        cv::imshow("ORB-SLAM2: debugMaxGrad",debugMaxGrad);

        if (menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if (bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
        }

        if (Stop())
        {
            while (isStopped())
            {
                usleep(3000);
            }
        }

        if (CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if (!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if (mbFinishRequested)
        return false;
    else if (mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
