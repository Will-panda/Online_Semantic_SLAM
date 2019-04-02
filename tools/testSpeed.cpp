//
// Created by liwenqiang on 4/2/19.
//
#include "System.h"
#include "Eigen/Eigen"

int main(int argc, char **argv)
{
    cv::Mat m1 = (cv::Mat_<float>(3, 3) << 1., 0., 3., 0., 5., 6., 7., 8., 0.);
    cv::Mat m2 = (cv::Mat_<float>(3, 3) << 0., 2., 1., 4., 5., 6., 7., 1., 0.);
    cv::Mat m3 = cv::Mat::zeros(3,3,CV_32F);
    const int times = 10000;
    TicToc timeChecker;

    for (int i = 0; i < times; i++)
    {
        m3 += m1 * m2;
    }
    printf("Opencv Multiple %d cost %lf ms.\n", times, timeChecker.Toc());
    cout<<"Result: "<<endl<<m3<<endl;
    m3 = cv::Mat::zeros(3,3,CV_32F);
    timeChecker.Tic();
    for (int i = 0; i < times; i++)
    {
        m3 += m1 + m2;
    }
    printf("Opencv Add %d cost %lf ms.\n", times, timeChecker.Toc());
    cout<<"Result: "<<endl<<m3<<endl;

    Eigen::Matrix3f eM1, eM2, eM3;
    eM1 << 1., 0., 3., 0., 5., 6., 7., 8., 0.;
    eM2 << 0., 2., 1., 4., 5., 6., 7., 1., 0.;
    eM3 = Eigen::Matrix3f::Zero();

    timeChecker.Tic();
    for (int i = 0; i < times; i++)
    {
        eM3 += eM1 * eM2;
    }
    printf("Eigen Multiple %d cost %lf ms.\n", times, timeChecker.Toc());
    cout<<"Result: "<<endl<<eM3<<endl;

    eM3 = Eigen::Matrix3f::Zero();
    timeChecker.Tic();
    for (int i = 0; i < times; i++)
    {
        eM3 += eM1 + eM2;
    }
    printf("Eigen Add %d cost %lf ms.\n", times, timeChecker.Toc());
    cout<<"Result: "<<endl<<eM3<<endl;

}