/**
* This file is a example of stereo semantic SLAM based on ORB-SLAM.
* Created by : Li Wenqiang , 03-05-2019
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<string> &vstrImageSeg,
                vector<string>& vstrImageDisparity,vector<double> &vTimestamps);

void DrawDisparity(const cv::Mat& imDis);
int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<string> vstrImageSeg;
    vector<string> vstrImageDisparity;
    vector<double> vTimestamps;

    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight,vstrImageSeg,vstrImageDisparity,vTimestamps);

    // Retrieve trajectory of ground Truth
//    string strPathGTFile = string(argv[3]) + "/pose.txt";
    string strPathGTFile = " ";


    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    bool useViewer = true;
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,useViewer,strPathGTFile);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imLeft, imRight,imSeg,imDisparity;
    int start=0;
//    int end = 1150;
    int end = nImages;
    for(int ni=start; ni<end; ni++)
    {
//        if(ni > 100) break;
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        imSeg = cv::imread(vstrImageSeg[ni],CV_LOAD_IMAGE_UNCHANGED);
//        imDisparity = cv::imread(vstrImageDisparity[ni],CV_LOAD_IMAGE_UNCHANGED);
//        if(imDisparity.empty()){
//            cerr<<"Havn't found disparity image at: "
//            <<vstrImageDisparity[ni]<<endl;
//            return 1;
//        }
//        cout<<"imDisparity type: "<<imDisparity.type()<<endl;
//        DrawDisparity(imDisparity);

//        assert(imDisparity.type() == CV_32F);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereoSemantic(imLeft,imRight,imSeg,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    string resultPath = string(argv[3])+"/CameraTrajectory.txt";
    SLAM.SaveTrajectoryKITTI(resultPath);

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string>& vstrImageLeft,
                vector<string>& vstrImageRight, vector<string>& vstrImageSeg,
                vector<string>& vstrImageDisparity,vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_2/";
    string strPrefixRight = strPathToSequence + "/image_3/";
    string strPrefixseg = strPathToSequence + "/segment_l/";
//    string strDisparity = strPathToSequence + "/disparity_l/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);
    vstrImageSeg.resize(nTimes);
    vstrImageDisparity.resize(nTimes);
    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
        vstrImageSeg[i] = strPrefixseg + ss.str() + ".png";
//        vstrImageDisparity[i] = strDisparity + ss.str() + ".png";
    }
}

void DrawDisparity(const cv::Mat& imDis)
{

    cv::Mat disparityVize = cv::Mat(imDis.size(), CV_8UC1, cv::Scalar(0));
    float maxVal = 0;
    for (int i = 0; i < imDis.rows; ++i) {
        for (int j = 0; j < imDis.cols; ++j) {
            if (maxVal < imDis.at<uchar>(i, j)) {
                maxVal = imDis.at<uchar>(i, j);
            }
        }
    }
    cout<<"max dis: "<<maxVal<<endl;
    for (int i = 0; i < imDis.rows; ++i) {
        for (int j = 0; j < imDis.cols; ++j) {
            float valf = 255.0 * imDis.at<uchar>(i, j) / (float)maxVal;

            disparityVize.at<uchar>(i, j) = uchar(valf);
        }
    }
    cv::Mat debugDis;
    cv::applyColorMap(disparityVize, debugDis, cv::COLORMAP_JET);
    cv::imshow("DebugDis",debugDis);
    cv::waitKey(0);
}





