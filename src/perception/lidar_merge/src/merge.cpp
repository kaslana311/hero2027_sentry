#include "lidar_merge/merge.hpp"

#include <chrono>
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
    SyncPolicy(5), lidar1_sub_, lidar2_sub_);
  sync_->registerCallback(std::bind(
    &LidarMergeNode::Sync_lidarCallback, this, std::placeholders::_1, std::placeholders::_2));
}
void LidarMergeNode::Sync_lidarCallback(
  const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg1,
  const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg2)
{
  auto start_time = std::chrono::steady_clock::now();

  // 1. 只拷贝头部和基础信息，不再直接拷贝数组后再排序
  auto fused_msg = std::make_unique<livox_ros_driver2::msg::CustomMsg>();
  fused_msg->header = msg2->header;
  fused_msg->timebase = msg2->timebase;
  fused_msg->lidar_id = msg2->lidar_id;

  fused_msg->points.reserve(msg1->points.size() + msg2->points.size());

  // 2. 提前计算时间差和外参矩阵（手动展开矩阵乘法，避开内部循环的Eigen构造开销）
  const int64_t time_diff =
    static_cast<int64_t>(msg1->timebase) - static_cast<int64_t>(msg2->timebase);

  const double r00 = T_lidar1_to_lidar2_(0, 0), r01 = T_lidar1_to_lidar2_(0, 1),
               r02 = T_lidar1_to_lidar2_(0, 2), tx = T_lidar1_to_lidar2_(0, 3);
  const double r10 = T_lidar1_to_lidar2_(1, 0), r11 = T_lidar1_to_lidar2_(1, 1),
               r12 = T_lidar1_to_lidar2_(1, 2), ty = T_lidar1_to_lidar2_(1, 3);
  const double r20 = T_lidar1_to_lidar2_(2, 0), r21 = T_lidar1_to_lidar2_(2, 1),
               r22 = T_lidar1_to_lidar2_(2, 2), tz = T_lidar1_to_lidar2_(2, 3);

  // 3. 高效双指针归并排序 (Two-pointer merge)
  size_t i = 0;  // msg1 index
  size_t j = 0;  // msg2 index
  uint32_t count_filter = 0;

  // 快速跳过 msg1 中时间非法的小于 0 的点（因为本身是递增的）
  while (i < msg1->points.size() && (time_diff + msg1->points[i].offset_time) < 0) {
    i++;
  }

  // 核心归并循环：因为 msg1 和 msg2 原本就是按时间递增的，所以谁时间小就先压入谁
  while (i < msg1->points.size() && j < msg2->points.size()) {
    uint32_t t1 = static_cast<uint32_t>(time_diff + msg1->points[i].offset_time);
    uint32_t t2 = msg2->points[j].offset_time;

    if (t1 <= t2) {
      livox_ros_driver2::msg::CustomPoint pt = msg1->points[i];
      const double px = pt.x, py = pt.y, pz = pt.z;
      pt.x = static_cast<float>(r00 * px + r01 * py + r02 * pz + tx);
      pt.y = static_cast<float>(r10 * px + r11 * py + r12 * pz + ty);
      pt.z = static_cast<float>(r20 * px + r21 * py + r22 * pz + tz);
      pt.offset_time = t1;

      fused_msg->points.push_back(pt);
      i++;
      count_filter++;
    } else {
      fused_msg->points.push_back(msg2->points[j]);
      j++;
    }
  }

  // 将剩余的点全部压入
  while (i < msg1->points.size()) {
    livox_ros_driver2::msg::CustomPoint pt = msg1->points[i];
    const double px = pt.x, py = pt.y, pz = pt.z;
    pt.x = static_cast<float>(r00 * px + r01 * py + r02 * pz + tx);
    pt.y = static_cast<float>(r10 * px + r11 * py + r12 * pz + ty);
    pt.z = static_cast<float>(r20 * px + r21 * py + r22 * pz + tz);
    pt.offset_time = static_cast<uint32_t>(time_diff + pt.offset_time);

    fused_msg->points.push_back(pt);
    i++;
    count_filter++;
  }

  while (j < msg2->points.size()) {
    fused_msg->points.push_back(msg2->points[j]);
    j++;
  }

  std::cout << "data " << count_filter << std::endl;
  fused_msg->point_num = fused_msg->points.size();

  // 发布融合后的点云
  PublishPointCloud(std::move(fused_msg));

  auto end_time = std::chrono::steady_clock::now();
  double time_consuming =
    std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
  std::cout << " -- [ INFO] Map updated in " << time_consuming << " ms." << std::endl;
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
