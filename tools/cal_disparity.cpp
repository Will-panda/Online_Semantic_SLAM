//
// Created by liwenqiang on 3/31/19.
//
#include "Thirdparty/libelas/src/elas.h"
#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<string> &vstrImageSeg, vector<double> &vTimestamps);

void DrawDisparity(const cv::Mat &imDis);

int main(int argc, char **argv)
{

//compute disparity

    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<string> vstrImageSeg;
    vector<double> vTimestamps;
    LoadImages(string(argv[1]), vstrImageLeft, vstrImageRight, vstrImageSeg, vTimestamps);

    cout << "Total: " << vstrImageLeft.size();
    for (int i = 0; i < vstrImageLeft.size(); i++)
    {
        cout << "process img : " << i << " .." << endl;
        cv::Mat imLeft = cv::imread(vstrImageLeft[i], CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat imRight = cv::imread(vstrImageRight[i], CV_LOAD_IMAGE_UNCHANGED);

        cv::cvtColor(imLeft, imLeft, CV_RGB2GRAY);
        cv::cvtColor(imRight, imRight, CV_RGB2GRAY);
        Elas::parameters param;
        param.postprocess_only_left = false;
        Elas elas(param);

        int totalPixel = imLeft.cols * imRight.rows;
        float pLeftDisparity[totalPixel];
        float pRightDisparity[totalPixel];

        const int32_t dims[3] = {imLeft.cols, imLeft.rows, imLeft.cols};
        assert(imLeft.type() == CV_8UC1);
        assert(imRight.type() == CV_8UC1);
        elas.process(imLeft.data, imRight.data, pLeftDisparity, pRightDisparity, dims);

        cv::Mat imgDisLeft = cv::Mat::zeros(imLeft.size(), CV_8U);
        for (int i = 0; i < imLeft.rows; ++i)
        {
            for (int j = 0; j < imLeft.cols; ++j)
            {
                float valf = pLeftDisparity[i * imLeft.cols + j];
                if(valf < 0.1) continue;
                imgDisLeft.at<uchar>(i, j) = uchar(valf);
            }
        }

//        cv::Mat disparityLeft = cv::Mat(imLeft.rows,imLeft.cols,CV_32F,pLeftDisparity);
        char resultName[15];
        std::sprintf(resultName, "%06d", i);

        string disparityPath = string(argv[1]) + "/disparity_l/" + resultName + ".png";
//        DrawDisparity(imgDisLeft); //for debug
        cv::imwrite(disparityPath,imgDisLeft);
    }
    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<string> &vstrImageSeg, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if (!s.empty())
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

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);
    vstrImageSeg.resize(nTimes);
    for (int i = 0; i < nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
        vstrImageSeg[i] = strPrefixseg + ss.str() + ".png";
    }
}

void DrawDisparity(const cv::Mat &imDis)
{

    cv::Mat disparityVize = cv::Mat(imDis.size(), CV_8UC1, cv::Scalar(0));
    float maxVal = 0;
    float minVal = 300;
    for (int i = 0; i < imDis.rows; ++i)
    {
        for (int j = 0; j < imDis.cols; ++j)
        {
            if (maxVal < imDis.at<uchar >(i, j))
                maxVal = imDis.at<uchar>(i, j);
            if(minVal > imDis.at<uchar>(i, j))
                minVal = imDis.at<uchar>(i,j);
        }
    }
    cout << "max dis: " << maxVal << endl;
    cout << "min dis: " << minVal << endl;
    for (int i = 0; i < imDis.rows; ++i)
    {
        for (int j = 0; j < imDis.cols; ++j)
        {
            float valf = 255.0 * imDis.at<uchar>(i, j) / maxVal;

            disparityVize.at<uchar>(i, j) = uchar(valf);
        }
    }
    cv::Mat debugDis;
    cv::applyColorMap(disparityVize, debugDis, cv::COLORMAP_JET);
    cv::imshow("DebugDis", debugDis);
    cv::waitKey(0);
}
