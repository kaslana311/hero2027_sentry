#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

int main(int argc, char const* argv[]) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // 无序点云，长度即点运算的个数
    cloud.width = 255;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.resize(cloud.width * cloud.height);
    // 自动推导类型加引用
    for (auto& point : cloud) {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    pcl::io::savePCDFileASCII(
        "/home/fyk/Projects/ros2_ws/src/pcl_study/PCD/test_pcd.pcd", cloud);
    return 0;
}
