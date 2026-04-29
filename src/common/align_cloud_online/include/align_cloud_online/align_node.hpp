#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <memory>
#include <mutex>
#include <vector>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#ifdef USE_NDT
#include <pcl/registration/ndt.h>
#else
#include <small_gicp/benchmark/read_points.hpp>
#include <small_gicp/factors/general_factor.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#endif

namespace align_cloud_online
{
struct Config
{
  Eigen::Matrix4d init = Eigen::Matrix4d::Identity();
#ifndef USE_NDT
  double smgicp_global_leaf_size = 0.5;
  double smgicp_local_leaf_size = 0.5;
  int smgicp_max_iterations = 50;
  int smgicp_num_threads = 4;
  double smgicp_max_dist_sq = 1.0;
  int neighbor_num = 20;
#else
  double voxel_size_;
  double ndt_epsilon_;
  double ndt_step_size_;
  double ndt_resolution_;
  int ndt_iterations_;
#endif
};
class AlignNode : public rclcpp::Node
{
public:
  AlignNode();

  void sync_cloud_callback(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & cloud_msg1,
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & cloud_msg2);
  void init_registration();

  void PublishAlignedCloud();

private:
  message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> lidar1_sub_;
  message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> lidar2_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    livox_ros_driver2::msg::CustomMsg, livox_ros_driver2::msg::CustomMsg>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, cloud2;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud1_pub_;
  Config config_;
  Eigen::Matrix4d current_transform_{Eigen::Matrix4d::Identity()};
  std::string target_frame_id_;
  rclcpp::Time target_stamp_{0, 0, RCL_ROS_TIME};
#ifndef USE_NDT
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> cloud_msg1_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> cloud_msg2_tree_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr cloud_msg1_;  //先验地图
  pcl::PointCloud<pcl::PointCovariance>::Ptr cloud_msg2_;  //源地图

  std::shared_ptr<small_gicp::Registration<
    small_gicp::GICPFactor, small_gicp::ParallelReductionOMP, small_gicp::RestrictDoFFactor>>
    register_;
#else
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
#endif
};

}  // namespace align_cloud_online
