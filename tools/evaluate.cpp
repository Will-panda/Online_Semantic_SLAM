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

void ComputeError(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &GTs,
                  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &Test);

void LoadTrajectory(const std::string &file,
                    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &trajectory);

void ComputeLength(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &trajectory,
                   std::vector<double> &trajectoryLength);

struct errors {
int32_t first_frame;
float   r_err;
float   t_err;
float   len;
float   speed;
errors (int32_t first_frame,float r_err,float t_err,float len,float speed) :
        first_frame(first_frame),r_err(r_err),t_err(t_err),len(len),speed(speed) {}
};

std::vector<errors> calcSequenceErrors (std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &posesGt,
                                        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &poseResult,std::vector<double>& distGT);


int32_t lastFrameFromSegmentLength(std::vector<double> &dist,int32_t first_frame,float len);

void SaveErrors(const std::vector<errors>& totalError,const std::string& outputPath);

int main(int argc, char **argv)
{
    std::string gtTrajectoryFile = argv[1];
    std::string testTrajectoryFile = argv[2];
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > poseGT;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > poseTest;

    LoadTrajectory(gtTrajectoryFile, poseGT);

    LoadTrajectory(testTrajectoryFile, poseTest);

    if (poseGT.empty() || poseTest.empty())
    {
        std::cerr << "Trajectory is empty!" << std::endl;
        return -1;
    }
    assert(poseGT.size() == poseTest.size());

//    ComputeError(poseGT,poseTest);

    ComputeRMSE(poseGT, poseTest);

    std::vector<double> trajectoryLenthGT;
    std::vector<double> trajectoryLenthTest;
    ComputeLength(poseGT, trajectoryLenthGT);
    ComputeLength(poseTest, trajectoryLenthTest);
    printf("length of GT: %lf m\n",trajectoryLenthGT.back());

    std::vector<errors> totalError =  calcSequenceErrors(poseGT,poseTest,trajectoryLenthGT);

    std::string outputPath = argv[3];
    SaveErrors(totalError,outputPath);


    DrawTrajectory(poseGT, poseTest);

    return 0;
}

void SaveErrors(const std::vector<errors>& totalError,const std::string& outputPath)
{
    float lengths[] = {100,200,300,400,500,600,700,800};
    int32_t num_lengths = 8;
    using namespace std;
    // file names
    char file_name_tl[1024]; sprintf(file_name_tl,"%s/tl.txt",outputPath.c_str() );
    char file_name_rl[1024]; sprintf(file_name_rl,"%s/rl.txt",outputPath.c_str() );
    char file_name_ts[1024]; sprintf(file_name_ts,"%s/s.txt",outputPath.c_str() );
    char file_name_rs[1024]; sprintf(file_name_rs,"%s/rs.txt",outputPath.c_str() );

    // open files
    FILE *fp_tl = fopen(file_name_tl,"w");
    FILE *fp_rl = fopen(file_name_rl,"w");
    FILE *fp_ts = fopen(file_name_ts,"w");
    FILE *fp_rs = fopen(file_name_rs,"w");

    // for each segment length do
    for (int32_t i=0; i<num_lengths; i++) {

        float t_err = 0;
        float r_err = 0;
        float num   = 0;

        // for all errors do
        for (auto it=totalError.begin(); it!=totalError.end(); it++) {
            if (fabs(it->len-lengths[i])<1.0) {
                t_err += it->t_err;
                r_err += it->r_err;
                num++;
            }
        }

        // we require at least 3 values
        if (num>2.5) {
            fprintf(fp_tl,"%f %f\n",lengths[i],t_err/num);
            fprintf(fp_rl,"%f %f\n",lengths[i],r_err/num);
        }
    }

    // for each driving speed do (in m/s)
    for (float speed=2; speed<25; speed+=2) {

        float t_err = 0;
        float r_err = 0;
        float num   = 0;

        // for all errors do
        for (auto it=totalError.begin(); it!=totalError.end(); it++) {
            if (fabs(it->speed-speed)<2.0) {
                t_err += it->t_err;
                r_err += it->r_err;
                num++;
            }
        }

        // we require at least 3 values
        if (num>2.5) {
            fprintf(fp_ts,"%f %f\n",speed,t_err/num);
            fprintf(fp_rs,"%f %f\n",speed,r_err/num);
        }
    }

    // close files
    fclose(fp_tl);
    fclose(fp_rl);
    fclose(fp_ts);
    fclose(fp_rs);
}

void ComputeLength(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &trajectory,
                   std::vector<double> &trajectoryLength)
{
    trajectoryLength.push_back(0.);
    for (int i = 1; i < trajectory.size(); i++)
    {
         Eigen::Matrix4d p1 = trajectory[i-1];
         Eigen::Matrix4d p2 = trajectory[i];
         Eigen::Vector3d dp = p2.col(3).head(3) - p1.col(3).head(3);
         double dist = dp.norm();
        trajectoryLength.push_back(dist + trajectoryLength[i-1]);
    }
}

int32_t lastFrameFromSegmentLength(std::vector<double> &dist,int32_t first_frame,float len) {
    for (int32_t i=first_frame; i<dist.size(); i++)
        if (dist[i]>dist[first_frame]+len)
            return i;
    return -1;
}

std::vector<errors> calcSequenceErrors (std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &posesGt,
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &poseResult,std::vector<double>& distGT) {
    using namespace std;
    // error vector
    vector<errors> err;

    float lengths[] = {100,200,300,400,500,600,700,800};
    int32_t num_lengths = 8;

    // parameters
    int32_t step_size = 10; // every second

    // for all start positions do
    for (int32_t first_frame=0; first_frame<posesGt.size(); first_frame+=step_size) {

        // for all segment lengths do
        for (int32_t i=0; i<num_lengths; i++) {

            // current length
            float len = lengths[i];

            // compute last frame
            int32_t last_frame = lastFrameFromSegmentLength(distGT,first_frame,len);

            // continue, if sequence not long enough
            if (last_frame==-1)
                continue;

            // compute rotational and translational errors
            Eigen::Matrix4d pose_delta_gt     = posesGt[first_frame].inverse()*posesGt[last_frame];
            Eigen::Matrix4d pose_delta_result = poseResult[first_frame].inverse()*poseResult[last_frame];
            Eigen::Matrix4d pose_error        = pose_delta_result.inverse()*pose_delta_gt;
            double r_err = acos(max( min(((pose_error.trace()-2)*0.5f),1.0), -1.0));
            double t_err = pose_error.col(3).head(3).norm();

            // compute speed
            float num_frames = static_cast<float>(last_frame-first_frame+1);
            float speed = len/(0.1f*num_frames);

            // write to file
            err.push_back(errors(first_frame,r_err/len,t_err/len,len,speed));
        }
    }

    // return error vector
    return err;
}

void LoadTrajectory(const std::string &file,
                    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &trajectory)
{
    std::ifstream fTrajectory;

    fTrajectory.open(file.c_str());
    while (!fTrajectory.eof())
    {
        std::string s;
        getline(fTrajectory, s);
        if (!s.empty())
        {
            std::stringstream ss;
            ss << s;
            Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();

            ss >> Twc(0, 0) >> Twc(0, 1) >> Twc(0, 2) >> Twc(0, 3) >>
               Twc(1, 0) >> Twc(1, 1) >> Twc(1, 2) >> Twc(1, 3) >>
               Twc(2, 0) >> Twc(2, 1) >> Twc(2, 2) >> Twc(2, 3);
            trajectory.push_back(Twc);
        }
    }

    if (!trajectory[0].isIdentity())
    {
        Eigen::Matrix4d origin = trajectory[0];
        for (int i = 1; i < trajectory.size(); i++)
        {
            trajectory[i] = origin.inverse() * trajectory[i];
        }
        trajectory[0] = Eigen::Matrix4d::Identity();
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
    cout << "RMSE: " << RMSE << endl;
    return;
}