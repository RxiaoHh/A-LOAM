#ifndef SCAN_REGISTRATION_HPP
#define SCAN_REGISTRATION_HPP

#include <cmath>
#include <vector>
#include <string>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/common/common.h>
#include<pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>//使用OMP需要添加的头文件

#define TYPENAME    pcl::PointXYZ

bool displayCloud(const pcl::PointCloud<TYPENAME>::Ptr cloud_input,
                        const pcl::PointCloud<TYPENAME>::Ptr cloud_target);

// void laserCloudHandler(const pcl::PointCloud<pcl::PointXYZ>& laserCloudMsg);
std::vector<pcl::PointCloud<PointType>> laserCloudHandler(const pcl::PointCloud<pcl::PointXYZ>& laserCloudMsg);

void calculateNormalAngle(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,std::vector<double>& normalBuf);


#endif

