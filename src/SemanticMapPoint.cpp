//
// Created by liwenqiang on 4/1/19.
//
#include "SemanticMapPoint.h"

namespace ORB_SLAM2
{

SemanticMapPoint::SemanticMapPoint(const cv::Mat &pt3d, int label) :
        mnObserveTime(1), mLabel(label), mSatus(UNDETERMINE)
{
    pt3d.copyTo(mLocation);
    mvLocationObserve.push_back(pt3d);
}

void SemanticMapPoint::AssertObservation(const cv::Mat &pt3d)
{
    mvLocationObserve.push_back(pt3d);
    mnObserveTime++;
}

void SemanticMapPoint::AssertObservation(SemanticMapPoint *pSPoint)
{
    mnObserveTime += pSPoint->GetOberveTime();
    mvLocationObserve.insert(mvLocationObserve.end(),pSPoint->mvLocationObserve.begin(),pSPoint->mvLocationObserve.end());
    mSatus = pSPoint->GetStatus();
}

bool SemanticMapPoint::MergeObservation()
{
    const float distanceThreshold = 5;
    for (auto &location: mvLocationObserve)
    {
        if (location.at<float>(2, 0) < 0)
        {
            return false;
        }
    }
    //calculate the diatance between point and camera center,if too large, discard
    vector<float> vdistance;
//        cout << "disttance: ";
    for (auto &pt :mvLocationObserve)
    {
//        float dist = sqrt(pt.dot(pt));
        float dist = pt.at<float>(0,2);
        vdistance.push_back(dist);
//            cout << dist << ", ";
    }
//        cout << endl;

    vector<float> orderedDistance = vdistance;
    std::sort(orderedDistance.begin(), orderedDistance.end());
    float difference = *(orderedDistance.rbegin()) - *(orderedDistance.begin());
        cout << "  difference: " << difference << endl;

    if (difference > distanceThreshold)
    {
        return false;
    }
//        cout<<"final distance: "<<finalDis<<endl;
    mLocation = *mvLocationObserve.rbegin();
    mSatus = STABLE;
    return true;
}


}

