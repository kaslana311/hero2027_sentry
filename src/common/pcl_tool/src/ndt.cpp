#include "pcl/registration/ndt.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include <pcl/filters/approximate_voxel_grid.h>
#include <iostream>

double Deg2Rad(double deg) {
    return deg * M_PI / 180.0;
}
// 欧拉角转旋转矩阵
Eigen::Matrix4f CreateTfMatrix(float x, float y, float z, float yaw, float pitch, float roll)
{
    yaw = Deg2Rad(yaw);
    pitch = Deg2Rad(pitch);
    roll = Deg2Rad(roll);
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = Eigen::Vector3f(x, y, z);
    return T;
}



int main(int argc, char **argv)
{

    std::cout << "1" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);

    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/fyk/Projects/ros2_ws/src/pcl_study/PCD/pcd.pcd", *cloud_source) == -1)
    {
        PCL_ERROR("Couldn't read file scans.pcd \n");
        return -1;
    }
    std::cout << "2" << std::endl;

    cloud_target->width = cloud_source->width;
    cloud_target->height = 1;
    cloud_target->is_dense = true;
    cloud_target->points.resize(cloud_source->width * cloud_source->height);
    for (size_t i = 0; i < cloud_source->points.size(); i++)
    {
        cloud_target->points[i].x = cloud_source->points[i].x + 10.0f;
        cloud_target->points[i].y = cloud_source->points[i].y + 19.0f;
        cloud_target->points[i].z = cloud_source->points[i].z + 1.0f;
    }
    std::cout << "3" << std::endl;

    // 对源点云进行下采样
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    // voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
    // voxel_grid.setInputCloud(cloud_source);
    // voxel_grid.filter(*cloud_source_filtered);
    // pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel_grid;
    // voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
    // voxel_grid.setInputCloud(cloud_source);
    // voxel_grid.filter(*cloud_source_filtered);
    std::cout << "4" << std::endl;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    // 设置变换收敛阈值
    ndt.setTransformationEpsilon(0.01);
    // 设置步长
    ndt.setStepSize(0.1);
    // 设置NDT网格结构分辨率
    ndt.setResolution(0.1);
    // 设置最大迭代次数
    ndt.setMaximumIterations(35);
    ndt.setInputSource(cloud_source);
    ndt.setInputTarget(cloud_target);
    std::cout << "5" << std::endl;

    Eigen::Matrix4f guess =CreateTfMatrix(10.0, 19.0, 1.0, 0, 0, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*output_cloud,guess);
    std::cout << "6" << std::endl;

    if (ndt.hasConverged())
    {
        std::cout << "NDT has converged:" << ndt.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << ndt.getFinalTransformation() << std::endl;
    }
    else
    {
        std::cout << "NDT has not converged." << std::endl;
        return (-1);

    }
    // PCL_ERROR("end \n");
      
    return 0;
}
