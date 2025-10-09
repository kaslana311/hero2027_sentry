#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/radius_outlier_removal.h"

#include <iostream>

int main(int argc, char const *argv[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("/home/fyk/Projects/ros2_ws/src/pcl_study/PCD/pcd.pcd",*cloud) == -1)
    {
        return -1;
    }
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;

    sor.setInputCloud(cloud); // 输入点云
    sor.setRadiusSearch(0.1);
    sor.setMinNeighborsInRadius(1);

    // sor.setMeanK(50);
    // sor.setStddevMulThresh(1.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*filter_cloud);
    
    pcl::io::savePCDFile("/home/fyk/Projects/ros2_ws/src/pcl_study/PCD/remove2_pcd.pcd",*filter_cloud);
    return 0;
}
