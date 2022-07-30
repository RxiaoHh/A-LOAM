// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>
#include <pcl/common/transforms.h>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "lidarFactor.hpp"

#include "../include/aloam_velodyne/scanRegistration.hpp"

#define DISTORTION 0

int corner_correspondence = 0, plane_correspondence = 0;

constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

int skipFrameNum = 5;
bool systemInited = false;

double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());

pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());

int laserCloudCornerLastNum = 0;
int laserCloudSurfLastNum = 0;

// Transformation from current frame to world frame
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);

// q_curr_last(x, y, z, w), t_curr_last
double para_q[4] = {0, 0, 0, 1};
double para_t[3] = {0, 0, 0};

Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
std::mutex mBuf;

// undistort lidar point
void TransformToStart(PointType const *const pi, PointType *const po)
{
    //interpolation ratio
    double s;
    if (DISTORTION)
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    else
        s = 1.0;
    //s = 1;
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    Eigen::Vector3d t_point_last = s * t_last_curr;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

void TransformToEnd(PointType const *const pi, PointType *const po)
{
    // undistort point first
    pcl::PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp);

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    //Remove distortion time info
    po->intensity = int(pi->intensity);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserOdometry");
    ros::NodeHandle nh;

    nh.param<int>("mapping_skip_frame", skipFrameNum, 2);
    printf("Mapping %d Hz \n", 10 / skipFrameNum);

    //读取点云
    std::string file_name_1 = "/home/renxiao/catkin_ws/src/A-LOAM/data/1.pcd";
    // std::string file_name_2 = "/home/renxiao/catkin_ws/src/A-LOAM/data/2.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_name_1,*cloud_input) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd");
        return -1;
    }
    // if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_name_2,*cloud_target) == -1)
    // {
    //     PCL_ERROR("Couldn't read file test_pcd.pcd");
    //     return -1;
    // }

    // 定义旋转矩阵，绕z轴
    float theta = M_PI / 9;	
    Eigen::Matrix4f trans_matrix = Eigen::Matrix4f::Identity();
    trans_matrix(0, 0) = cos(theta);
    trans_matrix(0, 1) = -sin(theta);
    trans_matrix(1, 0) = sin(theta);
    trans_matrix(1, 1) = cos(theta);

    trans_matrix(0,3) = 0.0;
    trans_matrix(1,3) = 0.0;
    trans_matrix(2,3) = 0.0;    
    // std::cout << "trans_matrix = " << trans_matrix << std::endl;
	pcl::transformPointCloud(*cloud_input, *cloud_target, trans_matrix);

    std::vector<pcl::PointCloud<PointType>> testResult;//存放最小平面点和较小平面点
    testResult = laserCloudHandler(*cloud_input);      
    *laserCloudSurfLast = testResult[1];               //上一帧较小平面点
    laserCloudSurfLastNum = laserCloudSurfLast->points.size();
    kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

    testResult = laserCloudHandler(*cloud_target);
    *surfPointsFlat = testResult[0];                   //当前帧最小平面点

    //计算较小平面点入射角
    std::vector<double> laserCloudSurfLastNormalsAngle;
    calculateNormalAngle(laserCloudSurfLast,laserCloudSurfLastNormalsAngle);
    //计算当前帧最小平面点入射角
    std::vector<double> surfPointsFlatNormalAngle;
    calculateNormalAngle(surfPointsFlat,surfPointsFlatNormalAngle);

    std::cout << "surfPointsFlat.szie() = " << surfPointsFlat->size() << "  laserCloudSurfLast.size() = " << laserCloudSurfLast->size() << std::endl;
    std::cout << "laserCloudSurfLastNormalsAngle.size() = " << laserCloudSurfLastNormalsAngle.size() << std::endl;
    std::cout << "surfPointsFlatNormalAngle.size() = " << surfPointsFlatNormalAngle.size() << std::endl;
                 
    int surfPointsFlatNum = surfPointsFlat->points.size();

    for (size_t opti_counter = 0; opti_counter < 10; ++opti_counter)
    {
        corner_correspondence = 0;
        plane_correspondence = 0;

        //ceres::LossFunction *loss_function = NULL;
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization *q_parameterization =
            new ceres::EigenQuaternionParameterization();
        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(para_q, 4, q_parameterization);
        problem.AddParameterBlock(para_t, 3);

        pcl::PointXYZI pointSel;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        std::vector<int> pointAngleSearchInd;
        std::vector<float> pointAngleSearchSqDis;
        // std::vector<double> partNormalsAngle;

        // 寻找平面点对应的平面特征
        for (int i = 0; i < surfPointsFlatNum; ++i)
        {
            TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
            kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);//寻找到离当前点最近的点

/**************************************************************************************************************/
           //查找离当前点最近点，最近的10个点
            pcl::PointXYZI pointAngleSel = laserCloudSurfLast->points[pointSearchInd[0]];
            kdtreeSurfLast->nearestKSearch(pointAngleSel, 10, pointAngleSearchInd, pointAngleSearchSqDis);

            //计算当前点和法向量夹角
            double currPointAngle = surfPointsFlatNormalAngle[i];
            double angleDis = 1000000;
            int partAngleIdx = 0;

            //跟当前点入射角最接近的点
            for(size_t k = 0;k < pointAngleSearchInd.size();k++) 
            {
                double tempAngleDis = std::fabs(laserCloudSurfLastNormalsAngle[pointAngleSearchInd[k]] - currPointAngle);
                if(tempAngleDis < angleDis)
                {
                    angleDis = tempAngleDis;
                    partAngleIdx = pointAngleSearchInd[k]; //寻找到跟当前点入射角最接近点的索引
                }
            }

/**************************************************************************************************************/

            int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
            {
                // closestPointInd = pointSearchInd[0];//最近点索引赋值,用于后续查找平面
                closestPointInd = partAngleIdx;     //加入入射角后最近点索引

                int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
                {
                    if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                        break;
                    double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                        (laserCloudSurfLast->points[j].y - pointSel.y) *
                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                        (laserCloudSurfLast->points[j].z - pointSel.z) *
                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                    if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                    {
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                    else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                    {
                        minPointSqDis3 = pointSqDis;
                        minPointInd3 = j;
                    }
                }

                for (int j = closestPointInd - 1; j >= 0; --j)
                {
                    if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                        break;

                    double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                        (laserCloudSurfLast->points[j].y - pointSel.y) *
                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                        (laserCloudSurfLast->points[j].z - pointSel.z) *
                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                    if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                    {
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                    else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                    {
                        minPointSqDis3 = pointSqDis;
                        minPointInd3 = j;
                    }
                }

                if (minPointInd2 >= 0 && minPointInd3 >= 0)
                {

                    Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                               surfPointsFlat->points[i].y,
                                               surfPointsFlat->points[i].z);
                    Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                 laserCloudSurfLast->points[closestPointInd].y,
                                                 laserCloudSurfLast->points[closestPointInd].z);
                    Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                 laserCloudSurfLast->points[minPointInd2].y,
                                                 laserCloudSurfLast->points[minPointInd2].z);
                    Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                 laserCloudSurfLast->points[minPointInd3].y,
                                                 laserCloudSurfLast->points[minPointInd3].z);

                    double s;
                    if (DISTORTION)
                        s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                    else
                         s = 1.0;
                    ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                    problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                    plane_correspondence++;
                }
            }
        }

        //printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
        // printf("data association time %f ms \n", t_data.toc());

        if ((corner_correspondence + plane_correspondence) < 10)
        {
            printf("less correspondence! *************************************************\n");
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        //输出优化信息
        std::cout << summary.BriefReport() << std::endl;
    }

    t_w_curr = t_w_curr + q_w_curr * t_last_curr;
    q_w_curr = q_w_curr * q_last_curr;

    Eigen::Matrix4d test_trans_matrix = Eigen::Matrix4d::Identity();
    test_trans_matrix.block<3,3>(0,0) = q_w_curr.toRotationMatrix();
    test_trans_matrix.block<3,1>(0,3) = t_w_curr;
    // std::cout << "test_trans_matrix = " << test_trans_matrix << std::endl;

    pcl::transformPointCloud(*cloud_target, *cloud_target, test_trans_matrix);
    displayCloud(cloud_input,cloud_target); //显示

    return 0;
}