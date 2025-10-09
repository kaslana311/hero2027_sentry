#include "iostream"
#include "pcl/filters/passthrough.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

int main(int argc, char const *argv[]) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(
            "/home/fyk/Projects/ros2_ws/src/pcl_study/PCD/2025RMUC.pcd",
            *cloud) == -1) {
        PCL_ERROR("Couldn't read file scans.pcd \n");
        return -1;
    }

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);

    pass.filter(*cloud_filtered);

    pcl::io::savePCDFileASCII(
        "/home/fyk/Projects/ros2_ws/src/pcl_study/PCD/2025RMUC_filtered.pcd",
        *cloud_filtered);

    return 0;
}
