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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer, const string &strGTFile) : mSensor(sensor), mpViewer(static_cast<Viewer *>(NULL)),
                                                                 mbReset(false), mbActivateLocalizationMode(false),
                                                                 mbDeactivateLocalizationMode(false), mbPause(false)
{
    // Output welcome message
    cout << endl <<
         "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
         "This program comes with ABSOLUTELY NO WARRANTY;" << endl <<
         "This is free software, and you are welcome to redistribute it" << endl <<
         "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if (mSensor == MONOCULAR)
        cout << "Monocular" << endl;
    else if (mSensor == STEREO)
        cout << "Stereo" << endl;
    else if (mSensor == RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
    }


    std::chrono::steady_clock::time_point timeOfVocStart = std::chrono::steady_clock::now();
    //Load ORB Vocabulary
    bool bVocLoad = false;
    mpVocabulary = new ORBVocabulary();
    if (strVocFile.find(".bin") != std::string::npos)
    {
        cout<<"Loading ORB Vocabulary From bin file. This is fast"<<endl;
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
    }
    if(strVocFile.find(".txt") != std::string::npos)
    {
        cout<<"Loading ORB Vocabulary From txt file. This could take a while..."<<endl;
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    }
    if (!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    std::chrono::steady_clock::time_point timeOfVocEnd = std::chrono::steady_clock::now();
    double tiemCostOfVoc = std::chrono::duration_cast<std::chrono::duration<double> >(timeOfVocEnd - timeOfVocStart).count();
    cout << "Vocabulary loaded: " << tiemCostOfVoc<<" s"<<endl << endl;

    mbUseGT = false;
    if (strGTFile == " ")
    {
        cout << "Not use grount truth!" << endl << endl;
    } else
    {
        mbUseGT = true;
        //Load ground truth trajectory
        LoadTrajectory(strGTFile);
        cout << "Groud Truth Trajectory Loaded!" << endl << endl;
    }

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap, strSettingsFile);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor, mbUseGT);
    if (!mbUseGT)
    {

        //Initialize the Local Mapping thread and launch
        mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR);
        mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

        //Initialize the Loop Closing thread and launch
        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR);
//        mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);



        //Set pointers between threads
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);

        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);

        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);
    }
    mpSMapper = new SemanticMapper(this, mpTracker, mpFrameDrawer,strSettingsFile);
    mptSMapper = new thread(&SemanticMapper::Run, mpSMapper);
    //Initialize the Viewer thread and launch
    if (bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker,mpSMapper, strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

}

cv::Mat System::TrackStereoSemantic(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat &imSeg,
                                   const double &timestamp)
{

    bool bPause;
    {
        unique_lock<mutex> lock(mMutexPause);
        bPause = mbPause;
    }

    if (bPause)
    {
        while (1)
        {
            {
                unique_lock<mutex> lock(mMutexPause);
                if (!mbPause)
                    break;
            }
            usleep(8000);
        }
    }

    if (mSensor != STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    if (!mbUseGT)
    {    // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode)
            {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped())
                {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode)
            {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
            }
        }

    }

    cv::Mat Tcw = mpTracker->GrabImageStereoSemantic(imLeft, imRight, imSeg,timestamp);

    mpFrameDrawer->Update(mpTracker);

    TicToc timeCheck;
    mpSMapper->UpdateFrame(&mpTracker->mCurrentFrame);
    cout<<"Semantic updateFrame cost: "<< timeCheck.Toc()<<" ms"<<endl;

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n = 0;
    int curn = mpMap->GetLastBigChangeIdx();
    if (n < curn)
    {
        n = curn;
        return true;
    } else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    if (!mbUseGT)
    {
        mpLocalMapper->RequestFinish();
        mpLoopCloser->RequestFinish();
        if (mpViewer)
        {
            mpViewer->RequestFinish();
            while (!mpViewer->isFinished())
                usleep(5000);
        }

        // Wait until all thread have effectively stopped
        while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
        {
            usleep(5000);
        }

        if (mpViewer)
            pangolin::BindToContext("ORB-SLAM2: Map Viewer");
    } else
    {
        return;
    }

}

void System::SetPause(bool flag)
{
    {
        unique_lock<mutex> lock(mMutexPause);
        mbPause = flag;
    }
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if (mSensor == MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                 lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++, lbL++)
    {
        if (*lbL)
            continue;

        KeyFrame *pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while (pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Two;

        cv::Mat Tcw = (*lit) * Trw;
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " "
          << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame *pKF = vpKFs[i];

        // pKF->SetPose(pKF->GetPose()*Two);

        if (pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1)
          << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    if (mbUseGT)
        return;
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if (mSensor == MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(), lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend; lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame *pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

        while (pKF->isBad())
        {
            //  cout << "bad parent" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Two;

        cv::Mat Tcw = (*lit) * Trw;
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

        f << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2) << " "
          << twc.at<float>(0) << " " <<
          Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " " << twc.at<float>(1)
          << " " <<
          Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " " << twc.at<float>(2)
          << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint *> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

void System::LoadTrajectory(const string &strGT)
{

    ifstream fTrajectoryGT;
    fTrajectoryGT.open(strGT.c_str());

    while (!fTrajectoryGT.eof())
    {
        string s;
        getline(fTrajectoryGT, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            cv::Mat Twc = cv::Mat::eye(4, 4, CV_32F);

            ss >> Twc.at<float>(0, 0) >> Twc.at<float>(0, 1) >> Twc.at<float>(0, 2) >> Twc.at<float>(0, 3) >>
               Twc.at<float>(1, 0) >> Twc.at<float>(1, 1) >> Twc.at<float>(1, 2) >> Twc.at<float>(1, 3) >>
               Twc.at<float>(2, 0) >> Twc.at<float>(2, 1) >> Twc.at<float>(2, 2) >> Twc.at<float>(2, 3);
            mvTrajectorGT.push_back(Twc);
        }
    }
    return;
}

void System::DrawTrajectory(const vector<cv::Mat> &poses)
{
    if (poses.empty())
    {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    cout << "poses size : " << poses.size() << endl;

    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++)
        {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.at<float>(0, 3), p1.at<float>(1, 3), p1.at<float>(2, 3));
            glVertex3d(p2.at<float>(0, 3), p2.at<float>(1, 3), p2.at<float>(2, 3));
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

} //namespace ORB_SLAM
