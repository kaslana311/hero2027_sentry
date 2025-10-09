#include <iostream>
#include "pcl/common/io.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"


int main(int argc, char const *argv[])
{

    // 拼接点
    // pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b;
    // cloud_a.width = 5;
    // cloud_a.height = 1;
    // cloud_a.points.resize(cloud_a.width * cloud_a.height);

    // cloud_b.width = 3;
    // cloud_b.height = 1;
    // cloud_b.points.resize(cloud_b.width * cloud_b.height);

    // for(auto& point : cloud_a.points)
    // {
    //     point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    //     point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    //     point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    // }
    // for(auto& point : cloud_b.points)
    // {
    //     point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    //     point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    //     point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    // }

    // pcl::PointCloud<pcl::PointXYZ> cloud_combined;
    // cloud_combined = cloud_a + cloud_b;

    // std::cout << cloud_combined.size()<< std::endl;
    // pcl::io::savePCDFileASCII("/home/fyk/Projects/ros2_ws/src/pcl_study/PCD/test_pcd.pcd", cloud_combined);
    // 拼接字段

    pcl::PointCloud<pcl::PointNormal> cloud_a;
    pcl::PointCloud<pcl::PointXYZ> cloud_b;

    cloud_a.width = 5;
    cloud_a.height = 1;
    cloud_a.is_dense = true;
    cloud_a.points.resize(cloud_a.width * cloud_a.height);

    for(auto& point : cloud_a.points)
    {
        point.normal_x= 1024 * rand() / (RAND_MAX + 1.0f);
        point.normal_y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.normal_z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    cloud_b.width = 5;
    cloud_b.height = 1;
    cloud_b.is_dense = true;
    cloud_b.points.resize(cloud_b.width * cloud_b.height);

    for (auto &point : cloud_b.points)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    pcl::PointCloud<pcl::PointNormal> cloud_combined;
    pcl::concatenateFields(cloud_a, cloud_b, cloud_combined);
    std::cout << cloud_combined.size() << std::endl;
    return 0;
}
