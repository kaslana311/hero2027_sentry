#include "lidar_merge/merge.hpp"

#include <cstdint>
#include <string>

namespace lidar_merge
{
LidarMergeNode::LidarMergeNode(const std::string & name) : Node(name)
{
  this->declare_parameter("lidar1_topic", "/livox/lidar1");
  this->declare_parameter("lidar2_topic", "/livox/lidar2");
  this->declare_parameter("lidar1_to_lidar2_x", 0.0);
  this->declare_parameter("lidar1_to_lidar2_y", 0.0);
  this->declare_parameter("lidar1_to_lidar2_z", 0.0);
  this->declare_parameter("lidar1_to_lidar2_roll", 0.0);
  this->declare_parameter("lidar1_to_lidar2_pitch", 0.0);
  this->declare_parameter("lidar1_to_lidar2_yaw", 0.0);
  auto lidar1_topic = this->get_parameter("lidar1_topic").as_string();
  auto lidar2_topic = this->get_parameter("lidar2_topic").as_string();
  double l1_to_l2_x = this->get_parameter("lidar1_to_lidar2_x").as_double();
  double l1_to_l2_y = this->get_parameter("lidar1_to_lidar2_y").as_double();
  double l1_to_l2_z = this->get_parameter("lidar1_to_lidar2_z").as_double();
  double l1_to_l2_roll = this->get_parameter("lidar1_to_lidar2_roll").as_double();
  double l1_to_l2_pitch = this->get_parameter("lidar1_to_lidar2_pitch").as_double();
  double l1_to_l2_yaw = this->get_parameter("lidar1_to_lidar2_yaw").as_double();
  Eigen::AngleAxisd rollAngle(l1_to_l2_roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(l1_to_l2_pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(l1_to_l2_yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  T_lidar1_to_lidar2_ = Eigen::Matrix4d::Identity();
  T_lidar1_to_lidar2_.block<3, 3>(0, 0) = q.toRotationMatrix();
  T_lidar1_to_lidar2_(0, 3) = l1_to_l2_x;
  T_lidar1_to_lidar2_(1, 3) = l1_to_l2_y;
  T_lidar1_to_lidar2_(2, 3) = l1_to_l2_z;

  cloud_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/livox/fused_cloud", 10);
  lidar1_sub_.subscribe(this, lidar1_topic);
  lidar2_sub_.subscribe(this, lidar2_topic);
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), lidar1_sub_, lidar2_sub_);
  sync_->registerCallback(std::bind(
    &LidarMergeNode::Sync_lidarCallback, this, std::placeholders::_1, std::placeholders::_2));
}
void LidarMergeNode::Sync_lidarCallback(
  const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg1,
  const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg2)
{
  // 以msg2为基础构建融合点云
  auto fused_msg = std::make_unique<livox_ros_driver2::msg::CustomMsg>(*msg2);
  fused_msg->points.reserve(msg1->points.size() + msg2->points.size());

  // 遍历 msg1 中的点，进行坐标变换后追加到 fused_msg
  uint32_t count_filter = 0;
  for (const auto & p1 : msg1->points) {
    livox_ros_driver2::msg::CustomPoint pt = p1;

    int64_t time =
      static_cast<int64_t>(msg1->timebase + p1.offset_time) - static_cast<int64_t>(msg2->timebase);
    if (time < 0) continue;
    count_filter++;
    // 过滤掉时间早于 msg2 的点
    Eigen::Vector4d p_vec(p1.x, p1.y, p1.z, 1.0);
    Eigen::Vector4d p_trans = T_lidar1_to_lidar2_ * p_vec;

    pt.x = p_trans.x();
    pt.y = p_trans.y();
    pt.z = p_trans.z();
    pt.offset_time = static_cast<uint32_t>(time);
    fused_msg->points.push_back(pt);
  }
  fused_msg->point_num += count_filter;
  // 更新总点数
  fused_msg->point_num = fused_msg->points.size();

  // 发布融合后的点云
  PublishPointCloud(std::move(fused_msg));
}
void LidarMergeNode::PublishPointCloud(livox_ros_driver2::msg::CustomMsg::UniquePtr msg1)
{
  // 发布融合后的点云
  cloud_pub_->publish(std::move(msg1));
}
}  // namespace lidar_merge
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_merge::LidarMergeNode>("lidar_merge_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
