#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include <pcl/filters/conditional_removal.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
// #include "rclcpp/rclcpp.hpp"
// #include "pcl/common/time.h"
#include <chrono>


Eigen::Matrix4d PoseToMat(const geometry_msgs::msg::Pose &pose) {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat(0, 3) = pose.position.x;
    mat(1, 3) = pose.position.y;
    mat(2, 3) = pose.position.z;

    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 m(q);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mat(i, j) = m[i][j];
        }
    }
    return mat;
}

double Deg2Rad(double deg) {
    return deg * M_PI / 180.0;
}

Eigen::Matrix4d CreateTfMatrix(double x, double y, double z, double yaw, double pitch, double roll)
{
    yaw = Deg2Rad(yaw);
    pitch = Deg2Rad(pitch);
    roll = Deg2Rad(roll);
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
    return T;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CropMapInFov(const pcl::PointCloud<pcl::PointXYZ>::Ptr &global_map,const Eigen::Matrix4d &pose_estimation, const nav_msgs::msg::Odometry &cur_odom, double FOV, double FOV_FAR)
{
    // pose_estimation是map to odom
    Eigen::Matrix4d T_odom_to_base_link = PoseToMat(cur_odom.pose.pose);
    Eigen::Matrix4d T_map_to_base_link = pose_estimation * T_odom_to_base_link;

    Eigen::Matrix4d T_base_link_to_map = T_map_to_base_link.inverse();

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_in_base_link(new pcl::PointCloud<pcl::PointXYZ>());
    // global_map_in_base_link = T_base_link_to_map * global_map;
    // 获取全局地图中在fov角中的部分,耗时
    // 1

    pcl::transformPointCloud(*global_map, *global_map_in_base_link, T_base_link_to_map);


    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
    // float min_radius = 5.0f;
    float max_radius = 10.0f;

    // 添加条件：点到原点的距离在min_radius和max_radius之间
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -max_radius)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, max_radius)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -max_radius)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, max_radius)));

    // 创建条件滤波器
    pcl::ConditionalRemoval<pcl::PointXYZ> cond_removal;
    cond_removal.setCondition(range_cond);
    cond_removal.setInputCloud(global_map_in_base_link);
    cond_removal.setKeepOrganized(false); // 如果设置为true，保留点云结构

    // 应用滤波器
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cond_removal.filter(*filtered_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_map(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::transformPointCloud(*filtered_cloud, *filtered_cloud_map, T_map_to_base_link);

    // 
    
    return filtered_cloud_map;

}


int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    // 读取PCD文件
    auto start = std::chrono::high_resolution_clock::now();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/fyk/Projects/ros2_ws/src/pcl_study/PCD/voxel2025RMUC.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file scans.pcd \n");
        return -1;
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by function: " << duration.count() << " 微秒" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = nullptr; 
    Eigen::Matrix4d pose_estimation = CreateTfMatrix(0,0,0,0,0,0);
    nav_msgs::msg::Odometry cur_odom;
    cur_odom.pose.pose.position.x = 0.0;
    cur_odom.pose.pose.position.y = 0.0;
    cur_odom.pose.pose.position.z = 0.0;

    start = std::chrono::high_resolution_clock::now();

    cloud_filtered = CropMapInFov(cloud, pose_estimation, cur_odom,6.28,3);

    end = std::chrono::high_resolution_clock::now();

    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << "剪切程序耗时: " << duration.count() << " 微秒" << std::endl;
    pcl::visualization::CloudViewer viewer("Conditional Filtered Cloud");
    viewer.showCloud(cloud_filtered);

    while (!viewer.wasStopped())
    {
        // 保持窗口打开
    }
    
    

}