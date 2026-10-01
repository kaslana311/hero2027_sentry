#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
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
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

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
    const auto source_cloud_odom_topic =
      declare_parameter<std::string>("source_cloud_odom_topic", "chassis_odometry_3d_gt");
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
    cloud_tf_wait_sec_ = declare_parameter<double>("cloud_tf_wait_sec", 0.5);
    const auto cloud_queue_size = declare_parameter<int>("cloud_queue_size", 20);
    if (!std::isfinite(cloud_tf_wait_sec_) || cloud_tf_wait_sec_ <= 0.0 ||
      cloud_queue_size <= 0 || !std::isfinite(lidar_sync_tolerance_sec_) ||
      lidar_sync_tolerance_sec_ < 0.0)
    {
      throw std::runtime_error("Cloud queue size/wait must be positive; sync tolerance nonnegative");
    }
    cloud_queue_size_ = static_cast<std::size_t>(cloud_queue_size);

    const auto left_lidar_pose = declare_parameter<std::vector<double>>(
      "left_lidar_pose",
      {-0.031029689, 0.207956386, 0.103822859, 0.004491681, 0.593489259, 1.574538158});
    const auto right_lidar_pose = declare_parameter<std::vector<double>>(
      "right_lidar_pose", {0.0, -0.217, 0.121, 0.0, 0.61086, -1.57079});
    left_lidar_to_target_ = poseToTransform(left_lidar_pose, "left_lidar_pose");
    right_lidar_to_target_ = poseToTransform(right_lidar_pose, "right_lidar_pose");
    // Gazebo's ray origin is 3 cm above the MID360 link origin. Its frame_id
    // names the link; robot_state_publisher does not include this sensor pose.
    sensor_to_lidar_ = poseToTransform(
      declare_parameter<std::vector<double>>("lidar_sensor_pose", {0., 0., 0.03, 0., 0., 0.}),
      "lidar_sensor_pose");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    cloud_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&SimGroundTruthBridge::publishPendingClouds, this));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic, 10);
    // Additive diagnostic output: the exact map origin used by cloud registration.
    // A comparison node started later must not infer a new origin from its first sample.
    truth_3d_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry/truth_3d", 10);
    // Gazebo sensor topics are best-effort, while the team dog_map consumes
    // Point-LIO's reliable cloud_registered output. Bridge between the two QoS
    // contracts instead of forwarding the simulator QoS unchanged.
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_cloud_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      source_odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&SimGroundTruthBridge::odometryCallback, this, std::placeholders::_1));
    cloud_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      source_cloud_odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&SimGroundTruthBridge::cloudOdometryCallback, this, std::placeholders::_1));
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
        source_cloud_topic_right.c_str(), output_cloud_topic.c_str(), map_frame_.c_str(),
        input_cmd_vel_topic.c_str(), output_cmd_vel_topic.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Simulation bridge ready: odom '%s' -> '%s', cloud '%s' -> '%s', cmd_vel '%s' -> '%s'",
        source_odom_topic.c_str(), output_odom_topic.c_str(), source_cloud_topic.c_str(),
        output_cloud_topic.c_str(), input_cmd_vel_topic.c_str(), output_cmd_vel_topic.c_str());
    }
    RCLCPP_INFO(
      get_logger(), "Clouds use 6-D '%s' at acquisition stamps, output frame '%s'; "
      "missing transforms wait at most %.2f s (no latest-pose fallback)",
      source_cloud_odom_topic.c_str(), map_frame_.c_str(), cloud_tf_wait_sec_);
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
    enqueueCloud(msg, nullptr);
  }

  void cloudOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Legacy navigation odometry is planar. Do not use it to infer the 3-D
    // sensor attitude on slopes, and do not publish a competing base_link TF.
    if (!have_initial_pose_ || rclcpp::Time(msg->header.stamp).nanoseconds() == 0) {
      return;
    }
    tf2::Transform world_to_chassis;
    tf2::fromMsg(msg->pose.pose, world_to_chassis);
    const auto map_to_chassis = zero_at_first_odometry_ ?
      initial_world_to_base_inverse_ * world_to_chassis : world_to_chassis;
    geometry_msgs::msg::TransformStamped pose;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = map_frame_;
    pose.child_frame_id = "sim_cloud_chassis_truth";
    pose.transform = tf2::toMsg(map_to_chassis);
    nav_msgs::msg::Odometry truth = *msg;
    truth.header.frame_id = map_frame_;
    truth.child_frame_id = pose.child_frame_id;
    tf2::toMsg(map_to_chassis, truth.pose.pose);
    truth_3d_pub_->publish(truth);
    tf_buffer_->setTransform(pose, "sim_ground_truth_bridge", false);
    publishPendingClouds();
  }

  struct PendingCloud
  {
    sensor_msgs::msg::PointCloud2::SharedPtr left;
    sensor_msgs::msg::PointCloud2::SharedPtr right;
    std::chrono::steady_clock::time_point received;
  };

  void enqueueCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & left,
    const sensor_msgs::msg::PointCloud2::SharedPtr & right)
  {
    // A zero ROS stamp means "latest" in TF, so reject it explicitly.
    if (rclcpp::Time(left->header.stamp).nanoseconds() == 0 ||
      (right && rclcpp::Time(right->header.stamp).nanoseconds() == 0))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping cloud with zero acquisition stamp");
      return;
    }
    if (pending_clouds_.size() >= cloud_queue_size_) {
      pending_clouds_.pop_front();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cloud TF queue full; dropping oldest scan");
    }
    pending_clouds_.push_back({left, right, std::chrono::steady_clock::now()});
    publishPendingClouds();
  }

  tf2::Transform cloudPoseAtStamp(const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    const auto pose = tf_buffer_->lookupTransform(
      map_frame_, "sim_cloud_chassis_truth", rclcpp::Time(cloud.header.stamp));
    tf2::Transform transform;
    tf2::fromMsg(pose.transform, transform);
    return transform;
  }

  void publishPendingClouds()
  {
    while (!pending_clouds_.empty()) {
      const auto & pending = pending_clouds_.front();
      // Steady time makes the wait bounded even when Gazebo is paused.
      if (std::chrono::duration<double>(
          std::chrono::steady_clock::now() - pending.received).count() > cloud_tf_wait_sec_)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Dropping cloud: acquisition-time 3-D pose/TF unavailable after %.2f s. "
          "Check chassis_odometry_3d_gt and rebuild the simulator image if missing.",
          cloud_tf_wait_sec_);
        pending_clouds_.pop_front();
        continue;
      }
      try {
        if (pending.right) {
          // Each lidar is registered at its OWN stamp before concatenation.
          auto left = transformCloud(
            *pending.left, cloudPoseAtStamp(*pending.left) * left_lidar_to_target_ * sensor_to_lidar_);
          auto right = transformCloud(
            *pending.right, cloudPoseAtStamp(*pending.right) * right_lidar_to_target_ * sensor_to_lidar_);
          auto merged = mergeClouds(left, right);
          merged.header.stamp = rclcpp::Time(left.header.stamp) >= rclcpp::Time(right.header.stamp) ?
            left.header.stamp : right.header.stamp;
          cloud_pub_->publish(merged);
        } else {
          const auto mount = tf_buffer_->lookupTransform(
            cloud_target_frame_, pending.left->header.frame_id,
            rclcpp::Time(pending.left->header.stamp));
          tf2::Transform chassis_to_sensor;
          tf2::fromMsg(mount.transform, chassis_to_sensor);
          cloud_pub_->publish(transformCloud(
            *pending.left, cloudPoseAtStamp(*pending.left) * chassis_to_sensor * sensor_to_lidar_));
        }
      } catch (const tf2::TransformException &) {
        // Never block this executor: odometry and /tf callbacks must still run.
        return;
      } catch (const std::runtime_error & error) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "Invalid simulated cloud: %s", error.what());
      }
      pending_clouds_.pop_front();
    }
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
    output.header.frame_id = map_frame_;

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

    enqueueCloud(left_cloud_, right_cloud_);

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
  double cloud_tf_wait_sec_{0.5};
  std::size_t cloud_queue_size_{20};
  std::deque<PendingCloud> pending_clouds_;
  bool have_initial_pose_{false};
  tf2::Transform initial_world_to_base_inverse_;
  tf2::Transform left_lidar_to_target_;
  tf2::Transform right_lidar_to_target_;
  tf2::Transform sensor_to_lidar_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr cloud_timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cloud_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr left_cloud_;
  sensor_msgs::msg::PointCloud2::SharedPtr right_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_3d_pub_;
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
