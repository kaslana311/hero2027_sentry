#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

int main(int argc, char const *argv[])
{
    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    // 读取PCD文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/fyk/Projects/ros2_ws/src/pcl_study/PCD/scans.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file scans.pcd \n");
        return -1;
    }
    // 打印点云中的点数，推荐使用 const auto&，尤其是在处理点云等复杂数据结构时，可以提高性能并减少内存开销。
    std::cout << "Loaded " << cloud->width * cloud->height << " data points from scans.pcd with the following fields: " << std::endl;
    std::cout << "width " << cloud->width << std::endl;
    std::cout << "length " << cloud->height << std::endl;

    
    // for (const auto& point : *cloud)
    // {
    //     std::cout << "    " << point.x << " " << point.y << " " << point.z << std::endl;
    // }
    return 0;
    
}
