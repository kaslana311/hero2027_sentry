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
  if (pcl::io::loadPCDFile("/home/z/ros_ws/src/pb2025_sentry_nav/"
                           "pb2025_nav_bringup/pcd/simulation/rmul_2026.pcd",
                           *cloud) == -1) {
    return -1;
  }
  std::cout << "read pcd success " << std::endl;
  pcl::io::savePLYFileASCII("/home/z/Downloads/rm_map/align/ps.ply", *cloud);
  return 0;
}
