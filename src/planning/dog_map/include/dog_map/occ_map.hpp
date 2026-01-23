#pragma once

#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_struct_traits.h>
#include <pcl/point_types.h>
#include <tbb/parallel_for.h>
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <opencv2/core/mat.hpp>
#include <pcl/impl/point_types.hpp>
#include <vector>

#include "dog_map/eigen_alias.hpp"
using namespace analy_utils;
template <typename T>
T yaml_get_value(const YAML::Node & node, const std::string & key)
{
  try {
    return node[key].as<T>();
  } catch (YAML::Exception & e) {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
    throw YAML::Exception(e.mark, ss.str());
  }
}
const int8_t LOG_OCC_HIT = 30;   // 占据增量
const int8_t LOG_OCC_FREE = -5;  // 空闲减量
const uint8_t THR_OCC = 35;      // 判定为障碍物的阈值
const uint8_t MAX_LOG = 250;
const uint8_t MIN_LOG = 5;

class OccMap
{
public:
  OccMap(const std::string & pgm_yaml_path = "");
  ~OccMap();
  void InitMap(cv::Mat & grid_map);
  void Update(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Pose pose);
  bool IsOccupied(const Vec2i & idx);
  bool IsOccupied(const Vec3i & idx);
  bool IsOccupied(int index);
  void Clear();
  void racyHandle(const Vec3i & start_idx, const std::vector<Vec3i> & obs_list);
  void racyHandle(const Vec3i & start_idx, const Vec3i & end_idx);
  bool IsStaticMapPoint(const Vec2i & idx);
  bool IsStaticMapPoint(int index);
  void TimeGoON();
  void PubRes(pcl::PointCloud<pcl::PointXYZI> & pub_pc);
  void PassFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, Pose pose);

private:
  cv::Mat grid_map_;
  uint8_t * occ_map;
  double resolution;
  double local_map_size;
  int local_map_size_d;
  double resolution_z;
  int layer_z;
  long long update_frame = 0;
  std::vector<double> cloud_at_map_origin;
  std::string pgm_yaml_path;
  std::map<int, long long> time_list_;  //索引+未观测到的累计帧数
};
