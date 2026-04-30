#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <small_gicp/benchmark/read_points.hpp>
#include <small_gicp/factors/general_factor.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <sstream>
#include <string>
#include <vector>
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

struct Config
{
  std::string base_cloud;
  std::string target_cloud;
  Eigen::Matrix4d init = Eigen::Matrix4d::Identity();

  double smgicp_global_leaf_size = 0.5;
  double smgicp_local_leaf_size = 0.5;
  int smgicp_max_iterations = 50;
  int smgicp_num_threads = 4;
  double smgicp_max_dist_sq = 1.0;
  int neighbor_num = 20;
};
// ...existing code...

static bool loadConfig(const std::string & path, Config & cfg)
{
  YAML::Node node = YAML::LoadFile(path);

  cfg.base_cloud = yaml_get_value<std::string>(node, "base_cloud");
  cfg.target_cloud = yaml_get_value<std::string>(node, "target_cloud");

  if (node["init"]) {
    auto deg2rad = [](double deg) { return deg * M_PI / 180.0; };
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    // 2. 按照你描述的顺序逐步施加变换 (使用 .rotate 和 .translate)
    // 注意：Isometry 的链式操作默认是相对于当前局部坐标系的

    // Step 1: 绕 Y 轴转 -35 度
    T.rotate(Eigen::AngleAxisd(deg2rad(-35.0), Eigen::Vector3d::UnitY()));

    // Step 2: 绕 Z 轴转 180 度
    T.rotate(Eigen::AngleAxisd(deg2rad(180.0), Eigen::Vector3d::UnitZ()));

    // Step 3: 沿 X 方向平移 0.4 米
    // (此时 X 方向已经因为前两步旋转指向了初始坐标系的斜后方)
    T.translate(Eigen::Vector3d(0.4, 0, 0));

    // Step 4: 绕 Y 轴转 35 度
    T.rotate(Eigen::AngleAxisd(deg2rad(35.0), Eigen::Vector3d::UnitY()));

    // 将计算结果赋值给你的 config_.init
    cfg.init = T.matrix();
  }

  if (node["smgicp"]) {
    auto sm = node["smgicp"];
    if (sm["global_leaf_size"]) cfg.smgicp_global_leaf_size = sm["global_leaf_size"].as<double>();
    if (sm["local_leaf_size"]) cfg.smgicp_local_leaf_size = sm["local_leaf_size"].as<double>();
    if (sm["max_iterations"]) cfg.smgicp_max_iterations = sm["max_iterations"].as<int>();
    if (sm["num_threads"]) cfg.smgicp_num_threads = sm["num_threads"].as<int>();
    if (sm["max_dist_sq"]) cfg.smgicp_max_dist_sq = sm["max_dist_sq"].as<double>();
    if (sm["neighbor_num"]) cfg.neighbor_num = sm["neighbor_num"].as<int>();
  }

  return !(cfg.base_cloud.empty() || cfg.target_cloud.empty());
}

int main(int argc, char * argv[])
{
  if (argc < 2) {
    std::cerr << "usage: " << argv[0] << " config.yaml\n";
    return -1;
  }

  Config cfg;
  if (!loadConfig(argv[1], cfg)) {
    std::cerr << "failed to load config\n";
    return -1;
  }
  std::cout << "=== Config ===\n";
  std::cout << "base_cloud: " << cfg.base_cloud << "\n";
  std::cout << "target_cloud: " << cfg.target_cloud << "\n";
  std::cout << "init:\n" << cfg.init << "\n";
  std::cout << "smgicp_global_leaf_size: " << cfg.smgicp_global_leaf_size << "\n";
  std::cout << "smgicp_local_leaf_size: " << cfg.smgicp_local_leaf_size << "\n";
  std::cout << "smgicp_max_iterations: " << cfg.smgicp_max_iterations << "\n";
  std::cout << "smgicp_num_threads: " << cfg.smgicp_num_threads << "\n";
  std::cout << "smgicp_max_dist_sq: " << cfg.smgicp_max_dist_sq << "\n";
  std::cout << "neighbor_num: " << cfg.neighbor_num << "\n";
  pcl::PointCloud<pcl::PointXYZ>::Ptr base(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile(cfg.base_cloud, *base) < 0) {
    std::cerr << "failed to load base cloud: " << cfg.base_cloud << "\n";
    return -1;
  }
  if (pcl::io::loadPCDFile(cfg.target_cloud, *target) < 0) {
    std::cerr << "failed to load target cloud: " << cfg.target_cloud << "\n";
    return -1;
  }
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> base_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr base_;  //先验地图
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

  std::shared_ptr<small_gicp::Registration<
    small_gicp::GICPFactor, small_gicp::ParallelReductionOMP, small_gicp::RestrictDoFFactor>>
    register_;
  base_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *base, cfg.smgicp_global_leaf_size);
  // Estimate covariances of points

  small_gicp::estimate_covariances_omp(*base_, cfg.neighbor_num, cfg.smgicp_num_threads);
  // Create KdTree for base
  base_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    base_, small_gicp::KdTreeBuilderOMP(cfg.smgicp_num_threads));

  //匹配器初始化
  register_ = std::make_shared<small_gicp::Registration<
    small_gicp::GICPFactor, small_gicp::ParallelReductionOMP, small_gicp::RestrictDoFFactor>>();

  register_->reduction.num_threads = cfg.smgicp_num_threads;
  register_->rejector.max_dist_sq = cfg.smgicp_max_dist_sq;
  register_->optimizer.max_iterations = cfg.smgicp_max_iterations;

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *target, cfg.smgicp_local_leaf_size);
  small_gicp::estimate_covariances_omp(*source_, cfg.neighbor_num, cfg.smgicp_num_threads);
  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(cfg.smgicp_num_threads));
  //初始位姿估计

  //匹配
  auto init_T_target_source = Eigen::Isometry3d(cfg.init);
  auto result = register_->align(*base_, *source_, *base_tree_, init_T_target_source);
  std::cout << "===== Final Transform =====\n";
  if (result.converged) {
    std::cout << "Gicp registration converged\n";
    std::cout << "average distance: " << result.error / result.num_inliers << "\n";
  }
  std::cout << result.T_target_source.matrix() << std::endl;
  // 输出 xyz + rpy
  Eigen::Matrix3d R = result.T_target_source.matrix().block<3, 3>(0, 0);
  Eigen::Vector3d t = result.T_target_source.matrix().block<3, 1>(0, 3);

  double roll = std::atan2(R(2, 1), R(2, 2));
  double pitch = std::asin(-R(2, 0));
  double yaw = std::atan2(R(1, 0), R(0, 0));

  std::cout << "xyz: " << t.x() << " " << t.y() << " " << t.z() << "\n";
  std::cout << "rpy(rad): " << roll << " " << pitch << " " << yaw << "\n";
  // === 可视化 ===
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_aligned(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*target, *source_aligned, result.T_target_source.matrix());

  pcl::visualization::PCLVisualizer::Ptr viewer(
    new pcl::visualization::PCLVisualizer("GICP Align"));
  viewer->setBackgroundColor(0, 0, 0);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> base_color(base, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_color(target, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_color(
    source_aligned, 0, 0, 255);

  viewer->addPointCloud(base, base_color, "base");
  viewer->addPointCloud(target, src_color, "source_raw");
  viewer->addPointCloud(source_aligned, aligned_color, "source_aligned");

  viewer->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "base");
  viewer->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source_raw");
  viewer->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source_aligned");

  std::cout << "[Viz] green=base, red=source, blue=aligned\n";
  viewer->spin();

  return 0;
}