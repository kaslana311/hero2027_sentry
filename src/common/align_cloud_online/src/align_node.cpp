#include "align_cloud_online/align_node.hpp"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

using namespace align_cloud_online;

AlignNode::AlignNode() : rclcpp::Node("align_cloud_online")
{
   this->declare_parameter("init_pose", std::vector<double>(6, 0.0));
#ifndef USE_NDT
  this->declare_parameter("smgicp_global_leaf_size", 0.05);
  this->declare_parameter("smgicp_local_leaf_size", 0.05);
  this->declare_parameter("smgicp_max_iterations", 500);
  this->declare_parameter("smgicp_num_threads", 4);
  this->declare_parameter("smgicp_max_dist_sq", 4.0);
  this->declare_parameter("neighbor_num", 20);
#else
  this->declare_parameter("ndt_epsilon_", 0.01);
  this->declare_parameter("ndt_step_size_", 0.1);
  this->declare_parameter("ndt_resolution_", 1.0);
  this->declare_parameter("ndt_iterations_", 400);
#endif
  this->declare_parameter("lidar1_topic", "/livox/lidar_192_168_1_193");
  this->declare_parameter("lidar2_topic", "/livox/lidar_192_168_1_126");
  this->declare_parameter("output_topic", "/livox/aligned/pointcloud");
  this->config_.init = Eigen::Matrix4d::Identity();
  std::vector<double> init_vec;
  this->get_parameter("init_pose", init_vec);
  if (init_vec.size() == 6) {
    const double x = init_vec[0];
    const double y = init_vec[1];
    const double z = init_vec[2];
    const double roll  = init_vec[3];
    const double pitch = init_vec[4];
    const double yaw   = init_vec[5];

    Eigen::AngleAxisd rx(roll,  Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rz(yaw,   Eigen::Vector3d::UnitZ());

   
    Eigen::Matrix3d R = (rz * ry * rx).toRotationMatrix();  // ZYX顺序
    this->config_.init.setIdentity();
    this->config_.init.block<3,3>(0,0) = R;
    this->config_.init(0,3) = x;
    this->config_.init(1,3) = y;
    this->config_.init(2,3) = z;
  } else {
    RCLCPP_WARN(this->get_logger(), "init_pose must have 6 values (xyzrpy), using identity");
  }
  std::cout << "=== Config ===\n";
  std::cout << "init:\n" << this->config_.init << "\n ";

  cloud1 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud2 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
#ifndef USE_NDT
  this->get_parameter("smgicp_global_leaf_size", this->config_.smgicp_global_leaf_size);
  this->get_parameter("smgicp_local_leaf_size", this->config_.smgicp_local_leaf_size);
  this->get_parameter("smgicp_max_iterations", this->config_.smgicp_max_iterations);
  this->get_parameter("smgicp_num_threads", this->config_.smgicp_num_threads);
  this->get_parameter("smgicp_max_dist_sq", this->config_.smgicp_max_dist_sq);
  this->get_parameter("neighbor_num", this->config_.neighbor_num);
#else
  this->get_parameter("ndt_epsilon_", this->config_.ndt_epsilon_);
  this->get_parameter("ndt_step_size_", this->config_.ndt_step_size_);
  this->get_parameter("ndt_resolution_", this->config_.ndt_resolution_);
  this->get_parameter("ndt_iterations_", this->config_.ndt_iterations_);
#endif
#ifndef USE_NDT
  std::cout << "smgicp_global_leaf_size: " << this->config_.smgicp_global_leaf_size << "\n";
  std::cout << "smgicp_local_leaf_size: " << this->config_.smgicp_local_leaf_size << "\n";
  std::cout << "smgicp_max_iterations: " << this->config_.smgicp_max_iterations << "\n";
  std::cout << "smgicp_num_threads: " << this->config_.smgicp_num_threads << "\n";
  std::cout << "smgicp_max_dist_sq: " << this->config_.smgicp_max_dist_sq << "\n";
  std::cout << "neighbor_num: " << this->config_.neighbor_num << "\n";
#else
  std::cout << "ndt_epsilon_: " << this->config_.ndt_epsilon_ << "\n";
  std::cout << "ndt_step_size_: " << this->config_.ndt_step_size_ << "\n";
  std::cout << "ndt_resolution_: " << this->config_.ndt_resolution_ << "\n";
  std::cout << "ndt_iterations_: " << this->config_.ndt_iterations_ << "\n";
#endif
  std::string lidar1_topic, lidar2_topic, output_topic;
  this->get_parameter("lidar1_topic", lidar1_topic);
  this->get_parameter("lidar2_topic", lidar2_topic);
  this->get_parameter("output_topic", output_topic);
  this->lidar1_sub_.subscribe(this, lidar1_topic);
  this->lidar2_sub_.subscribe(this, lidar2_topic);
  init_registration();
  this->sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), lidar1_sub_, lidar2_sub_);
  this->sync_->registerCallback(
    std::bind(&AlignNode::sync_cloud_callback, this, std::placeholders::_1, std::placeholders::_2));
  this->aligned_cloud2_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic + "2", 10);
  this->aligned_cloud1_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic + "1", 10);
}
void AlignNode::init_registration()
{
#ifndef USE_NDT
  this->register_ = std::make_shared<small_gicp::Registration<
    small_gicp::GICPFactor, small_gicp::ParallelReductionOMP, small_gicp::RestrictDoFFactor>>();
  this->register_->reduction.num_threads = this->config_.smgicp_num_threads;
  this->register_->rejector.max_dist_sq = this->config_.smgicp_max_dist_sq;
  this->register_->optimizer.max_iterations = this->config_.smgicp_max_iterations;
#else
  this->ndt.setTransformationEpsilon(this->config_.ndt_epsilon_);
  this->ndt.setStepSize(this->config_.ndt_step_size_);
  this->ndt.setResolution(this->config_.ndt_resolution_);
  this->ndt.setMaximumIterations(this->config_.ndt_iterations_);
#endif
}

void AlignNode::sync_cloud_callback(
  const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & cloud_msg1,
  const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & cloud_msg2)
{
  // Convert ROS messages to PCL point clouds
  std::cout<<"align start "<<std::endl;
  // Lambda 函数：将 Livox CustomMsg 填充到 pcl::PointCloud
  auto convert_livox_to_pcl = [](
                                const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr & out_cloud) {
    out_cloud->reserve(msg->point_num);
    for (uint32_t i = 0; i < msg->point_num; ++i) {
      pcl::PointXYZ pt;
      pt.x = msg->points[i].x;
      pt.y = msg->points[i].y;
      pt.z = msg->points[i].z;
      out_cloud->push_back(pt);
    }
    out_cloud->header.frame_id = msg->header.frame_id;
    // 注意：Livox 的时间戳通常是纳秒或微秒，PCL 头使用的是微秒
  };

  // 执行转换
  convert_livox_to_pcl(cloud_msg1, cloud1);
  convert_livox_to_pcl(cloud_msg2, cloud2);
#ifndef USE_NDT
  // 进行下采样
  this->cloud_msg1_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *cloud1, this->config_.smgicp_global_leaf_size);
  this->cloud_msg2_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *cloud2, this->config_.smgicp_local_leaf_size);
  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(
    *this->cloud_msg1_, this->config_.neighbor_num, this->config_.smgicp_num_threads);
  small_gicp::estimate_covariances_omp(
    *this->cloud_msg2_, this->config_.neighbor_num, this->config_.smgicp_num_threads);

  this->cloud_msg1_tree_ =
    std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
      this->cloud_msg1_, small_gicp::KdTreeBuilderOMP(this->config_.smgicp_num_threads));
  this->cloud_msg2_tree_ =
    std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
      this->cloud_msg2_, small_gicp::KdTreeBuilderOMP(this->config_.smgicp_num_threads));

  //初始位姿估计
  auto result = this->register_->align(
    *this->cloud_msg1_, *this->cloud_msg2_, *this->cloud_msg2_tree_,
    Eigen::Isometry3d(this->config_.init));
  // 处理结果
  config_.init = result.T_target_source.matrix();
  RCLCPP_INFO(this->get_logger(), "Alignment done. Fitness: %f", result.error);
#else
  this->ndt.setInputSource(cloud2);
  this->ndt.setInputTarget(cloud1);
  pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
  this->ndt.align(aligned_cloud, Eigen::Matrix4f(this->config_.init.cast<float>()).matrix());
  config_.init = this->ndt.getFinalTransformation().cast<double>().matrix();
  RCLCPP_INFO(this->get_logger(), "Alignment done. Fitness: %f", this->ndt.getFitnessScore());
#endif

  pcl::transformPointCloud(*cloud2, *cloud2, config_.init);
  // 发布对齐后的点云
  PublishAlignedCloud();
}
void AlignNode::PublishAlignedCloud()
{
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*cloud2, output_msg);
  output_msg.header.frame_id = cloud2->header.frame_id;
  sensor_msgs::msg::PointCloud2 output_msg1;
  pcl::toROSMsg(*cloud1, output_msg1);
  output_msg1.header.frame_id = cloud1->header.frame_id;
  aligned_cloud2_pub_->publish(output_msg);

  aligned_cloud1_pub_->publish(output_msg1);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AlignNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
