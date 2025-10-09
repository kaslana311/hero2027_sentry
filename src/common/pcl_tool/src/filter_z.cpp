#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>

int main(int argc, char** argv) {
    // 定义输入输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);

    // 1. 读取点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(
            "/home/dji/projects/hero2025_sentinel_rmuc_ws/src/planning/"
            "trg_planner_ros/prebuilt_maps/fast_lio_latest_map.pcd",
            *cloud) == -1) {
        std::cerr << "Error loading point cloud file." << std::endl;
        return -1;
    }

    std::cout << "原始点云点数: " << cloud->size() << std::endl;

    // 2. 创建滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass;

    // 3. 设置滤波器参数
    pass.setInputCloud(cloud);     // 输入点云
    pass.setFilterFieldName("z");  // 设置过滤维度（Z轴）
    pass.setFilterLimits(-std::numeric_limits<float>::max(),
                         1.0);  // 保留Z值 < 1.0米的点
    // pass.setFilterLimitsNegative(true); // 如果要保留范围外的点（反向过滤）

    // 4. 执行滤波
    pass.filter(*cloud_filtered);

    std::cout << "过滤后点云点数: " << cloud_filtered->size() << std::endl;

    // 5. 保存结果
    pcl::io::savePCDFileASCII(
        "/home/dji/projects/hero2025_sentinel_rmuc_ws/src/planning/"
        "trg_planner_ros/prebuilt_maps/fast_lio_latest_map1.pcd",
        *cloud_filtered);

    return 0;
}