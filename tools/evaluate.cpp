//
// Created by liwenqiang on 4/4/19.
//

#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "TicToc.h"
#include "pangolin/pangolin.h"
#include <unistd.h>
#include "sophus/se3.h"

void DrawTrajectory(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &GTs,
                    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &Test);

void ComputeRMSE(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &GTs,
                 std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &Test);

void LoadTrajectory(const std::string &file,
                    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &trajectory);

int main(int argc, char **argv)
{
    std::string gtTrajectoryFile = argv[1];
    std::string testTrajectoryFile = argv[2];
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > GTs;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > Tests;

    LoadTrajectory(gtTrajectoryFile, GTs);
    LoadTrajectory(testTrajectoryFile, Tests);

    ComputeRMSE(GTs, Tests);

    DrawTrajectory(GTs, Tests);

    return 0;
}


void LoadTrajectory(const std::string &file,
                    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &trajectory)
{
    std::ifstream fTrajectory;

    fTrajectory.open(file.c_str());
    Eigen::Matrix4d Twc = Eigen::Matrix4d::Ones();
    while (!fTrajectory.eof())
    {
        std::string s;
        getline(fTrajectory, s);
        if (!s.empty())
        {
            std::stringstream ss;
            ss << s;
            Eigen::Matrix4d Twc = Eigen::Matrix4d::Ones();

            ss >> Twc(0, 0) >> Twc(0, 1) >> Twc(0, 2) >> Twc(0, 3) >>
               Twc(1, 0) >> Twc(1, 1) >> Twc(1, 2) >> Twc(1, 3) >>
               Twc(2, 0) >> Twc(2, 1) >> Twc(2, 2) >> Twc(2, 3);
            trajectory.push_back(Twc);
        }
    }
    return;
}

void DrawTrajectory(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &GTs,
                    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &Test)
{
    using namespace std;
    if (GTs.empty() || Test.empty())
    {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuGroundTruth("menu.GroundTruth", true, true);
    pangolin::Var<bool> menuTest("menu.Test", true, true);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    cout << "poses1 size : " << GTs.size() << endl;
    cout << "poses2 size : " << Test.size() << endl;


    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        if (menuGroundTruth)
        {
            glLineWidth(2);
            for (size_t i = 0; i < GTs.size() - 1; i++)
            {
                glColor3f(1, 0.0f, 1);
                glBegin(GL_LINES);
                auto p1 = GTs[i], p2 = GTs[i + 1];
                glVertex3d(p1(0, 3), p1(1, 3), p1(2, 3));
                glVertex3d(p2(0, 3), p2(1, 3), p2(2, 3));
                glEnd();
            }
        }
        if (menuTest)
        {
            glLineWidth(2);
            for (size_t i = 0; i < Test.size() - 1; i++)
            {
                glColor3f(0.f, 0.0f, 0.f);
                glBegin(GL_LINES);
                auto p1 = Test[i], p2 = Test[i + 1];
                glVertex3d(p1(0, 3), p1(1, 3), p1(2, 3));
                glVertex3d(p2(0, 3), p2(1, 3), p2(2, 3));
                glEnd();
            }
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

void ComputeRMSE(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &GTs,
                 std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &Test)
{
    using namespace std;
    if (GTs.empty() || Test.empty())
    {
        cerr << "Trajectory is empty!" << endl;
        return;
    }
    assert(GTs.size() == Test.size());

    double sum_err = 0;
    for (int i = 0; i < GTs.size(); i++)
    {
        auto p1 = GTs[i];
        auto p2 = Test[i];
        Sophus::SE3 soP1(p1.block<3, 3>(0, 0), p1.col(3).head(3));
        Sophus::SE3 soP2(p2.block<3, 3>(0, 0), p2.col(3).head(3));
        auto kesi = (soP1.inverse() * soP2).log();
        sum_err += kesi.transpose() * kesi;
    }
    double RMSE = sqrt(sum_err / GTs.size());
    cout<<"RMSE: "<<RMSE<<endl;
    return;
}