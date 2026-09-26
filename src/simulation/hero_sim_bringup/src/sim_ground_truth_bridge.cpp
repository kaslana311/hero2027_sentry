#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace hero_sim_bringup
{

class SimGroundTruthBridge : public rclcpp::Node
{
public:
  SimGroundTruthBridge()
  : Node("sim_ground_truth_bridge")
  {
    const auto source_odom_topic =
      declare_parameter<std::string>("source_odom_topic", "chassis_odometry_gt");
    const auto output_odom_topic =
      declare_parameter<std::string>("output_odom_topic", "odometry/filtered");
    const auto source_cloud_topic =
      declare_parameter<std::string>("source_cloud_topic", "livox/lidar");
    const auto source_cloud_topic_left =
      declare_parameter<std::string>("source_cloud_topic_left", "livox/lidar_left");
    const auto source_cloud_topic_right =
      declare_parameter<std::string>("source_cloud_topic_right", "livox/lidar_right");
    const auto output_cloud_topic =
      declare_parameter<std::string>("output_cloud_topic", "cloud_registered");
    const auto input_cmd_vel_topic =
      declare_parameter<std::string>("input_cmd_vel_topic", "cmd_vel_nav2_result");
    const auto output_cmd_vel_topic =
      declare_parameter<std::string>("output_cmd_vel_topic", "cmd_vel");

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    simulation_base_frame_ =
      declare_parameter<std::string>("simulation_base_frame", "base_footprint");
    algorithm_base_frame_ =
      declare_parameter<std::string>("algorithm_base_frame", "base_link");
    cloud_target_frame_ = declare_parameter<std::string>("cloud_target_frame", "chassis");
    zero_at_first_odometry_ = declare_parameter<bool>("zero_at_first_odometry", true);
    relay_cmd_vel_ = declare_parameter<bool>("relay_cmd_vel", true);
    dual_lidar_ = declare_parameter<bool>("dual_lidar", false);
    lidar_sync_tolerance_sec_ = declare_parameter<double>("lidar_sync_tolerance_sec", 0.075);

    const auto left_lidar_pose = declare_parameter<std::vector<double>>(
      "left_lidar_pose",
      {-0.031029689, 0.207956386, 0.103822859, 0.004491681, 0.593489259, 1.574538158});
    const auto right_lidar_pose = declare_parameter<std::vector<double>>(
      "right_lidar_pose", {0.0, -0.217, 0.121, 0.0, 0.61086, -1.57079});
    left_lidar_to_target_ = poseToTransform(left_lidar_pose, "left_lidar_pose");
    right_lidar_to_target_ = poseToTransform(right_lidar_pose, "right_lidar_pose");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic, 10);
    // Gazebo sensor topics are best-effort, while the team dog_map consumes
    // Point-LIO's reliable cloud_registered output. Bridge between the two QoS
    // contracts instead of forwarding the simulator QoS unchanged.
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_cloud_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      source_odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&SimGroundTruthBridge::odometryCallback, this, std::placeholders::_1));
    if (dual_lidar_) {
      left_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        source_cloud_topic_left, rclcpp::SensorDataQoS(),
        std::bind(&SimGroundTruthBridge::leftCloudCallback, this, std::placeholders::_1));
      right_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        source_cloud_topic_right, rclcpp::SensorDataQoS(),
        std::bind(&SimGroundTruthBridge::rightCloudCallback, this, std::placeholders::_1));
    } else {
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        source_cloud_topic, rclcpp::SensorDataQoS(),
        std::bind(&SimGroundTruthBridge::cloudCallback, this, std::placeholders::_1));
    }
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_cmd_vel_topic, 10,
      std::bind(&SimGroundTruthBridge::cmdVelCallback, this, std::placeholders::_1));

    publishStaticTransforms();

    if (dual_lidar_) {
      RCLCPP_INFO(
        get_logger(),
        "Simulation bridge ready: odom '%s' -> '%s', dual clouds ['%s', '%s'] -> '%s' "
        "in frame '%s', cmd_vel '%s' -> '%s'",
        source_odom_topic.c_str(), output_odom_topic.c_str(), source_cloud_topic_left.c_str(),
        source_cloud_topic_right.c_str(), output_cloud_topic.c_str(), cloud_target_frame_.c_str(),
        input_cmd_vel_topic.c_str(), output_cmd_vel_topic.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Simulation bridge ready: odom '%s' -> '%s', cloud '%s' -> '%s', cmd_vel '%s' -> '%s'",
        source_odom_topic.c_str(), output_odom_topic.c_str(), source_cloud_topic.c_str(),
        output_cloud_topic.c_str(), input_cmd_vel_topic.c_str(), output_cmd_vel_topic.c_str());
    }
  }

private:
  tf2::Transform poseToTransform(const std::vector<double> & pose, const std::string & name)
  {
    if (pose.size() != 6) {
      throw std::runtime_error(name + " must contain [x, y, z, roll, pitch, yaw]");
    }

    tf2::Quaternion rotation;
    rotation.setRPY(pose[3], pose[4], pose[5]);
    return tf2::Transform(rotation, tf2::Vector3(pose[0], pose[1], pose[2]));
  }

  void publishStaticTransforms()
  {
    const auto stamp = now();

    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = stamp;
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.child_frame_id = odom_frame_;
    map_to_odom.transform.rotation.w = 1.0;

    geometry_msgs::msg::TransformStamped simulation_to_algorithm_base;
    simulation_to_algorithm_base.header.stamp = stamp;
    simulation_to_algorithm_base.header.frame_id = simulation_base_frame_;
    simulation_to_algorithm_base.child_frame_id = algorithm_base_frame_;
    simulation_to_algorithm_base.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(
      std::vector<geometry_msgs::msg::TransformStamped>{
        map_to_odom, simulation_to_algorithm_base});
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    tf2::Transform world_to_base;
    tf2::fromMsg(msg->pose.pose, world_to_base);

    if (!have_initial_pose_) {
      initial_world_to_base_inverse_ = world_to_base.inverse();
      have_initial_pose_ = true;
      RCLCPP_INFO(
        get_logger(), "Captured the first Gazebo pose as the map/odom origin: [%.3f, %.3f, %.3f]",
        msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    }

    const tf2::Transform odom_to_base =
      zero_at_first_odometry_ ? initial_world_to_base_inverse_ * world_to_base : world_to_base;

    nav_msgs::msg::Odometry output = *msg;
    output.header.frame_id = odom_frame_;
    output.child_frame_id = algorithm_base_frame_;
    tf2::toMsg(odom_to_base, output.pose.pose);
    odom_pub_->publish(output);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = msg->header.stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = simulation_base_frame_;
    transform.transform = tf2::toMsg(odom_to_base);
    tf_broadcaster_->sendTransform(transform);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // The cloud stays in the simulated lidar frame. dog_map performs the frame transform using TF.
    cloud_pub_->publish(*msg);
  }

  void leftCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    left_cloud_ = msg;
    left_cloud_is_new_ = true;
    tryPublishMergedCloud();
  }

  void rightCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    right_cloud_ = msg;
    right_cloud_is_new_ = true;
    tryPublishMergedCloud();
  }

  sensor_msgs::msg::PointCloud2 transformCloud(
    const sensor_msgs::msg::PointCloud2 & input, const tf2::Transform & transform) const
  {
    sensor_msgs::msg::PointCloud2 output = input;
    output.header.frame_id = cloud_target_frame_;

    sensor_msgs::PointCloud2Iterator<float> x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> z(output, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
        continue;
      }
      const tf2::Vector3 transformed = transform * tf2::Vector3(*x, *y, *z);
      *x = static_cast<float>(transformed.x());
      *y = static_cast<float>(transformed.y());
      *z = static_cast<float>(transformed.z());
    }
    return output;
  }

  bool schemasMatch(
    const sensor_msgs::msg::PointCloud2 & first,
    const sensor_msgs::msg::PointCloud2 & second) const
  {
    if (first.point_step != second.point_step || first.is_bigendian != second.is_bigendian ||
      first.fields.size() != second.fields.size())
    {
      return false;
    }
    for (std::size_t i = 0; i < first.fields.size(); ++i) {
      const auto & a = first.fields[i];
      const auto & b = second.fields[i];
      if (a.name != b.name || a.offset != b.offset || a.datatype != b.datatype || a.count != b.count) {
        return false;
      }
    }
    return true;
  }

  void appendCloudRows(
    const sensor_msgs::msg::PointCloud2 & input, std::vector<uint8_t> & output) const
  {
    const std::size_t row_data_size =
      static_cast<std::size_t>(input.width) * static_cast<std::size_t>(input.point_step);
    for (uint32_t row = 0; row < input.height; ++row) {
      const auto begin = input.data.begin() + static_cast<std::ptrdiff_t>(row * input.row_step);
      output.insert(output.end(), begin, begin + static_cast<std::ptrdiff_t>(row_data_size));
    }
  }

  sensor_msgs::msg::PointCloud2 mergeClouds(
    const sensor_msgs::msg::PointCloud2 & left,
    const sensor_msgs::msg::PointCloud2 & right) const
  {
    sensor_msgs::msg::PointCloud2 output = left;
    output.height = 1;
    output.width = left.width * left.height + right.width * right.height;
    output.row_step = output.width * output.point_step;
    output.is_dense = left.is_dense && right.is_dense;
    output.data.clear();
    output.data.reserve(output.row_step);
    appendCloudRows(left, output.data);
    appendCloudRows(right, output.data);
    return output;
  }

  void tryPublishMergedCloud()
  {
    if (!left_cloud_ || !right_cloud_ || !left_cloud_is_new_ || !right_cloud_is_new_) {
      return;
    }

    const auto left_stamp = rclcpp::Time(left_cloud_->header.stamp).nanoseconds();
    const auto right_stamp = rclcpp::Time(right_cloud_->header.stamp).nanoseconds();
    const auto tolerance_ns = static_cast<int64_t>(lidar_sync_tolerance_sec_ * 1e9);
    if (left_stamp + tolerance_ns < right_stamp) {
      left_cloud_is_new_ = false;
      return;
    }
    if (right_stamp + tolerance_ns < left_stamp) {
      right_cloud_is_new_ = false;
      return;
    }

    if (!schemasMatch(*left_cloud_, *right_cloud_)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot merge simulated lidar clouds because their PointCloud2 schemas differ");
      left_cloud_is_new_ = false;
      right_cloud_is_new_ = false;
      return;
    }

    try {
      auto left = transformCloud(*left_cloud_, left_lidar_to_target_);
      auto right = transformCloud(*right_cloud_, right_lidar_to_target_);
      auto merged = mergeClouds(left, right);
      merged.header.stamp = left_stamp >= right_stamp ? left_cloud_->header.stamp : right_cloud_->header.stamp;
      cloud_pub_->publish(merged);
    } catch (const std::runtime_error & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Failed to merge simulated lidar clouds: %s", error.what());
    }

    left_cloud_is_new_ = false;
    right_cloud_is_new_ = false;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (relay_cmd_vel_) {
      cmd_vel_pub_->publish(*msg);
    }
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string simulation_base_frame_;
  std::string algorithm_base_frame_;
  std::string cloud_target_frame_;
  bool zero_at_first_odometry_{true};
  bool relay_cmd_vel_{true};
  bool dual_lidar_{false};
  bool left_cloud_is_new_{false};
  bool right_cloud_is_new_{false};
  double lidar_sync_tolerance_sec_{0.075};
  bool have_initial_pose_{false};
  tf2::Transform initial_world_to_base_inverse_;
  tf2::Transform left_lidar_to_target_;
  tf2::Transform right_lidar_to_target_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr left_cloud_;
  sensor_msgs::msg::PointCloud2::SharedPtr right_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

}  // namespace hero_sim_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hero_sim_bringup::SimGroundTruthBridge>());
  rclcpp::shutdown();
  return 0;
}
