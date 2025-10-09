#include <iostream>

#include "pcl/filters/voxel_grid.h"
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

int main(int argc, char const *argv[]) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(
            "src/common/point_cloud_files/source_pcd/fast_lio_latest_map.pcd",
            *cloud) == -1) {
        PCL_ERROR("Couldn't read file scans.pcd \n");
        return -1;
    }

    // 将点云空间划分为0.01*0.01*0.01的网格，计算其中所有点的重心，并用重心点替代体素内的所有点
    float size = 0.06;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(size, size, size);
    voxel_filter.filter(*cloud_filtered);
    // pcl::io::savePCDFile("/home/fyk/Projects/ros2_ws/src/pcl_study/PCD/scans_filtered.pcd",cloud_filtered);
    pcl::io::savePCDFileASCII(
        "src/common/point_cloud_files/filtered_pcd/fast_lio_latest_map.pcd",
        *cloud_filtered);
    return 0;
}
