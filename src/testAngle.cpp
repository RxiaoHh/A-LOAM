#include <iostream>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>//使用OMP需要添加的头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/registration/icp.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
using namespace std;


int main()
{
	//------------------加载点云数据-------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>());
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/renxiao/catkin_ws/src/A-LOAM/data/1.pcd", *cloud) == -1)
	{
		PCL_ERROR("Could not read file\n");
	}

    // 定义旋转矩阵，绕z轴
    float theta = M_PI / 7;	
    Eigen::Matrix4f trans_matrix = Eigen::Matrix4f::Identity();
    trans_matrix(0,3) = 0.5;
    trans_matrix(1,3) = 0.6;
    trans_matrix(2,3) = 1.0;

    trans_matrix(0, 0) = cos(theta);
    trans_matrix(0, 1) = -sin(theta);
    trans_matrix(1, 0) = sin(theta);
    trans_matrix(1, 1) = cos(theta);

    std::cout << "trans_matrix = " << trans_matrix << std::endl;

	pcl::transformPointCloud(*cloud, *cloud_target, trans_matrix);    

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>());
    //创建ICP的实例类
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud);
    icp.setInputTarget(cloud_target);
    icp.setMaxCorrespondenceDistance(50);//设置对应点对之间的最大距离（此值对配准结果影响较大）。
    icp.setTransformationEpsilon(0.001);// 设置两次变化矩阵之间的差值（一般设置为1e-10即可）；步长
    icp.setEuclideanFitnessEpsilon(0.001);// 设置收敛条件是均方误差和小于阈值， 停止迭代；
    icp.setMaximumIterations(30); //最大迭代次数，icp是一个迭代的方法，最多迭代这些次（若结合可视化并逐次显示，可将次数设置为1）；
    icp.align(*cloud_final);//icp执行计算，并将变换后的点云保存在cloud_final里
    cout << "matrix: " << icp.getFinalTransformation() << endl;    


	//------------------计算法线----------------------
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;//OMP加速
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_2(new pcl::PointCloud<pcl::Normal>);
    
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	n.setNumberOfThreads(10);//设置openMP的线程数
	//n.setViewPoint(0,0,0);//设置视点，默认为（0，0，0）
	n.setInputCloud(cloud_target);
	n.setSearchMethod(tree);
	n.setKSearch(10);//点云法向计算时，需要所搜的近邻点大小
	//n.setRadiusSearch(0.03);//半径搜素
	n.compute(*normals);//开始进行法向计

    n.setInputCloud(cloud_final);
    n.compute(*normals_2);//开始进行法向计
    
    std::cout << "cloud->size() = " << cloud_target->size() << std::endl;
    std::cout << "normals->size() = " << normals->size() << std::endl;
    
    std::cout << "cloud_final->size() = " << cloud_final->size() << std::endl;
    std::cout << "normals_2->size() = " << normals_2->size() << std::endl;

    for(size_t i = 0;i < cloud_target->size();i++)
    {
        Eigen::Vector3f v1(cloud_target->points[i].x,cloud_target->points[i].y,cloud_target->points[i].z);
        Eigen::Vector3f v2(normals->points[i].normal_x,normals->points[i].normal_y,normals->points[i].normal_z);

        Eigen::Vector3f v3(cloud_final->points[i].x,cloud_final->points[i].y,cloud_final->points[i].z);
        Eigen::Vector3f v4(normals_2->points[i].normal_x,normals_2->points[i].normal_y,normals_2->points[i].normal_z);

        std::cout << i << "  source = " << pcl::getAngle3D(v1,v2,false) << "  translation = " << pcl::getAngle3D(v3,v4,false) << std::endl;
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("icp test"));  //定义窗口共享指针
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(cloud_target,250,0,0); //设置源点云的颜色为红色
    view->addPointCloud(cloud_target,sources_cloud_color,"sources_cloud_v1");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color (cloud_final,250,250,250);  //目标点云为白色
    view->addPointCloud(cloud_final,target_cloud_color,"target_cloud_v1"); //将点云添加到v1窗口
 
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"sources_cloud_v1");  //设置显示点的大小
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target_cloud_v1");
 
    while(!view->wasStopped())
    {
        view->spinOnce();  //运行视图 
    }

	return 0;
}
