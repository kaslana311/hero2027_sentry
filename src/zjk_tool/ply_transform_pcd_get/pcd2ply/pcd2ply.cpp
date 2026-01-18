#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>  //PCL中支持的点类型的头文件
// #include <pcl/visualization/pcl_visualizer.h>
using namespace std;
using namespace pcl;

//读取CAD模型z
int main() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile("/home/dji/project/HERO2025_Sentry/src/common/point_cloud_files/filtered_pcd/fast_lio_latest_map.pcd",
                           *cloud) == -1) {
    return -1;
  }
  std::cout << "read pcd success " << std::endl;
  pcl::io::savePLYFileASCII("/home/dji/zjk/temp/hero2026_-sentry/src/zjk_tool/ply_transform_pcd_get/pcd2ply/ply_files/s.ply", *cloud);
  return 0;
}
