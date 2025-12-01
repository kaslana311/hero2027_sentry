#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>  //PCL中支持的点类型的头文件

#include <filesystem>
// #include <pcl/visualization/pcl_visualizer.h>
using namespace std;
using namespace pcl;
#define VOXEL 0
#define PLY 0
#define M2MM 0
#define TF 0
#define MOVE 0
#define ANGLE 0
#define WRITE 1
#define PATHT 0
#define SHOW(x, y, z, m, n, p)                                               \
  std::cout << x << ' ' << y << ' ' << z << ' ' << m << ' ' << n << ' ' << p \
            << ' ' << std::endl
//读取CAD模型z
int main() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::string path =
      "/home/z/Downloads/fastlio2/ply_transform_pcd_get/build/scans.pcd";
  std::string name = std::filesystem::path(path).stem().string();
  if (PLY && pcl::io::loadPLYFile<pcl::PointXYZ>(path, *cloud) == -1) {
    PCL_ERROR("无法读取PLY文件！\n");
    return -1;
  }
  if (!PLY && pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1) {
    PCL_ERROR("无法读取Pcd文件！\n");
    return -1;
  }
  // Mm 2 M
  float min_z = 1000000, min_x = 1000000, min_y = 1000000, max_z = -1000000,
        max_y = -1000000, max_x = -1000000;
  if (TF) {
    Eigen::Matrix4f transformation_matrix;
    transformation_matrix << 1.000, 0.000, -0.002, 0.011, 0.000, 1.000, -0.001,
        -0.006, 0.002, 0.001, 1.000, 0.017, 0.000, 0.000, 0.000, 1.000;

    pcl::transformPointCloud(*cloud, *cloud, transformation_matrix);
  }
  for (auto it = cloud->begin(); it != cloud->end();) {
    auto& p = *it;
    if (M2MM) {
      p.x /= 1000;
      p.y /= 1000;
      p.z /= 1000;
    }

    if (p.x < min_x) {
      min_x = p.x;
    }
    if (p.x > max_x) {
      max_x = p.x;
    }
    if (p.y < min_y) {
      min_y = p.y;
    }
    if (p.y > max_y) {
      max_y = p.y;
    }
    if (p.z < min_z) {
      min_z = p.z;
    }
    if (p.z > max_z) {
      max_z = p.z;
    }
    ++it;
  }
  SHOW(min_x, min_y, min_z, max_x, max_y, max_z);
  if (MOVE) {
    for (auto& p : *cloud) {
      p.x -= min_x;
      p.y -= min_y;
      p.z -= min_z;
    }
  }
  for (int i = 0; i < 200; i++) {
    pcl::PointXYZ p;
    p.x = 10;
    p.y = 0;
    p.z = i / 100.0 * 3;
    cloud->push_back(p);
  }

  if (VOXEL) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    int origin = cloud->size();
    voxel.setInputCloud(cloud);
    float size = 0.02;
    voxel.setLeafSize(size, size, size);
    voxel.filter(*cloud);
    std::cout << "origin size " << origin << " vox_filter " << cloud->size()
              << std::endl;
  }

  if (PATHT) {
    pcl::PassThrough<pcl::PointXYZ> pass_z, pass_x, pass_y;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.2, 1.9 + 1.65);

    // pass_x.setInputCloud(cloud);
    // pass_x.setFilterFieldName("x");
    // pass_x.setFilterLimits(-15.0, 0.0);

    // pass_y.setInputCloud(cloud);
    // pass_y.setFilterFieldName("y");
    // pass_y.setFilterLimits(-6.0, 0.0);

    pass_z.filter(*cloud);
    // pass_x.filter(*cloud);
    // pass_y.filter(*cloud);
  }
  if (ANGLE) {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // 第一步：绕Z轴旋转90度
    float angle_z = -M_PI / 2;  // 90度，弧度制
    transform.rotate(Eigen::AngleAxisf(angle_z, Eigen::Vector3f::UnitZ()));

    // // 第二步：绕X轴旋转30度
    float angle_y = -M_PI / 6;  // 30度，弧度制
    transform.rotate(Eigen::AngleAxisf(angle_y, Eigen::Vector3f::UnitY()));

    // float angle_x = 3 * M_PI / 2;
    // transform.rotate(Eigen::AngleAxisf(angle_x, Eigen::Vector3f::UnitX()));
    // 应用变换
    // transform = transform.inverse();
    pcl::transformPointCloud(*cloud, *cloud, transform);
  }
  std::cout << "ok" << std::endl;
  if (WRITE)
    pcl::io::savePCDFileASCII(
        "/home/z/Downloads/fastlio2/ply_transform_pcd_get/build/scans.pcd",
        *cloud);

  return 0;
}
