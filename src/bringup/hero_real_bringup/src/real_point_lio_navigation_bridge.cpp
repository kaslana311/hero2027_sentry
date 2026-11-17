#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Real adapter: no truth subscription, no estimator/controller changes.
// Point-LIO remains in odom. Only map->odom carries global alignment/correction;
// body odometry/velocity must never contain that correction a second time.
class RealPointLioNavigationBridge : public rclcpp::Node
{
  using Odom = nav_msgs::msg::Odometry;
  using Twist = geometry_msgs::msg::Twist;
  using Transform = geometry_msgs::msg::TransformStamped;
  using Steady = std::chrono::steady_clock;
public:
  RealPointLioNavigationBridge() : Node("real_point_lio_navigation_bridge")
  {
    relay_ = declare_parameter<bool>("relay_cmd_vel", false);
    publish_map_to_odom_ = declare_parameter<bool>("publish_map_to_odom", true);
    for (const auto * name : {"map_odom_offset_x", "map_odom_offset_y", "map_odom_offset_yaw"}) {
      if (!std::isfinite(declare_parameter<double>(name, 0.))) {
        throw std::runtime_error("Map/odom offsets must be finite");
      }
    }
    // Calibration parameters are immutable only AFTER initial declaration.
    // ROS invokes set callbacks while declaring a parameter as well.
    auto validate_update =
      [](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result; result.successful = true;
        for (const auto & p : parameters) {
          if (p.get_name() == "publish_map_to_odom" || p.get_name() == "base_to_imu_xyz" ||
            p.get_name() == "base_to_imu_rpy" || p.get_name() == "initial_base_pose") {
            result.successful = false;
            result.reason = "TF ownership is startup-only: restart with publish_map_to_odom:=false";
          } else if (p.get_name() == "map_odom_offset_x" || p.get_name() == "map_odom_offset_y" ||
            p.get_name() == "map_odom_offset_yaw")
          {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
              !std::isfinite(p.as_double()))
            {result.successful = false; result.reason = "Offsets require finite double values";}
          }
        }
        return result;
      };
    timeout_ = declare_parameter<double>("data_timeout_sec", 1.0);
    cmd_timeout_ = declare_parameter<double>("command_timeout_sec", 0.5);
    max_velocity_dt_ = declare_parameter<double>("velocity_max_dt_sec", 0.5);
    if (!std::isfinite(timeout_) || timeout_ <= 0. || !std::isfinite(cmd_timeout_) ||
      cmd_timeout_ <= 0. || !std::isfinite(max_velocity_dt_) || max_velocity_dt_ <= 0.)
    {throw std::runtime_error("Timeouts must be finite and positive");}
    const auto offset = declare_parameter<std::vector<double>>(
      "base_to_imu_xyz", {0.062208, -0.207112, 0.0});
    if (offset.size() != 3 || !std::all_of(offset.begin(), offset.end(),
        [](double v) {return std::isfinite(v);}))
    {throw std::runtime_error("base_to_imu_xyz requires three finite coordinates");}
    const auto rpy = declare_parameter<std::vector<double>>(
      "base_to_imu_rpy", {0., 0.614120, -1.5707963267948966});
    const auto start = declare_parameter<std::vector<double>>(
      "initial_base_pose", {0., 0., 0., 0.});
    if (rpy.size() != 3 || start.size() != 4 ||
      !std::all_of(rpy.begin(), rpy.end(), [](double v) {return std::isfinite(v);}) ||
      !std::all_of(start.begin(), start.end(), [](double v) {return std::isfinite(v);}))
    {throw std::runtime_error("base_to_imu_rpy is RPY radians; initial_base_pose is XYZ/yaw");}
    tf2::Quaternion rotation; rotation.setRPY(rpy[0], rpy[1], rpy[2]);
    chassis_to_imu_ = tf2::Transform(rotation,
      tf2::Vector3(offset[0], offset[1], offset[2]));
    tf2::Quaternion initial_yaw; initial_yaw.setRPY(0., 0., start[3]);
    initial_base_ = tf2::Transform(initial_yaw, tf2::Vector3(start[0], start[1], start[2]));
    parameter_callback_ = add_on_set_parameters_callback(validate_update);
    odom_pub_ = create_publisher<Odom>("odometry/filtered", 10);
    map_odom_pub_ = create_publisher<Odom>("base_link_in_map_odom", 10);
    cmd_pub_ = create_publisher<Twist>("cmd_vel", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "pointlio/navigation/status", rclcpp::QoS(1).transient_local());
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    static_tf_pub_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    Transform sensor;
    sensor.header.frame_id = "base_link"; sensor.child_frame_id = "pointlio_imu";
    sensor.transform = tf2::toMsg(chassis_to_imu_);
    static_tf_pub_->sendTransform(sensor);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, true);
    odom_sub_ = create_subscription<Odom>("pointlio/odometry", rclcpp::SensorDataQoS(),
      [this](Odom::ConstSharedPtr m) {odometry(*m);});
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      // The deployment bridge offers RELIABLE; this freshness interlock must
      // request retransmission too. Sensor odometry remains best effort.
      "pointlio/cloud_registered_map", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
        checkClock();
        const auto stamp = stampNs(m->header.stamp);
        if (!failed_ && aligned_ && m->header.frame_id == "map" &&
          stamp >= alignment_ns_ && stamp > cloud_ns_ && freshStamp(stamp) &&
          m->width > 0 && m->height > 0 && !m->data.empty())
        {cloud_ns_ = stamp; cloud_received_ = Steady::now(); have_cloud_ = true;}
      });
    cmd_sub_ = create_subscription<Twist>("cmd_vel_nav2_result", 10,
      [this](Twist::ConstSharedPtr m) {command(*m);});
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
        checkClock();
        if (aligned_ && !failed_ && publish_map_to_odom_ && now().nanoseconds() > 0) {
          publishMapTransform(now());
        }
        const auto reason = health();
        if (reason != "ready" || (active_command_ &&
            elapsed(command_received_) > cmd_timeout_)) {stop();}
        std_msgs::msg::String status;
        std::ostringstream out;
        out << "{\"state\":\"" << reason << "\",\"relay_enabled\":"
            << (relay_ ? "true" : "false") << ",\"command_active\":"
            << (active_command_ ? "true" : "false") << ",\"pose_stamp_ns\":" << pose_ns_
            << ",\"cloud_stamp_ns\":" << cloud_ns_ << "}";
        status.data = out.str(); status_pub_->publish(status);
      });
    RCLCPP_WARN(get_logger(), "REAL INPUT: calibrated Point-LIO IMU pose + cloud; NO truth input. "
      "Unified map->odom->base_footprint->base_link->pointlio_imu TF; "
      "configured initial chassis XYZ/yaw, NOT the simulator spawn. "
      "Backend remains shadow-only. Commands are already in chassis axes, no second yaw rotation.");
  }
private:
  static int64_t stampNs(const builtin_interfaces::msg::Time & stamp)
  {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) {return -1;}
    return static_cast<int64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
  }
  static bool decode(double x, double y, double z,
    const geometry_msgs::msg::Quaternion & q, tf2::Transform & result)
  {
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      !std::isfinite(norm) || norm < 1e-12) {return false;}
    tf2::Quaternion rotation(q.x, q.y, q.z, q.w); rotation.normalize();
    result = tf2::Transform(rotation, tf2::Vector3(x, y, z)); return true;
  }
  static double elapsed(Steady::time_point t)
  {return std::chrono::duration<double>(Steady::now() - t).count();}
  bool freshStamp(int64_t stamp)
  {
    const auto now_ns = get_clock()->now().nanoseconds();
    return stamp > 0 && now_ns > 0 && stamp <= now_ns + 100000000 &&
      static_cast<double>(now_ns - stamp) * 1e-9 <= timeout_;
  }
  void stop()
  {
    if (relay_ && active_command_) {cmd_pub_->publish(Twist());}
    active_command_ = false;
    // Never replay a command after data recovers; require a new Nav2 command.
  }
  void fail()
  {
    if (!failed_) {RCLCPP_ERROR(get_logger(), "Sensor/ROS time reset: restart the real launch");}
    failed_ = true; stop();
  }
  void checkClock()
  {
    const auto now_ns = get_clock()->now().nanoseconds();
    if (last_clock_ns_ > 0 && now_ns < last_clock_ns_ - 500000000) {fail();}
    last_clock_ns_ = now_ns;
  }
  std::string health()
  {
    if (failed_) {return "restart_required";}
    if (!aligned_) {return "waiting_for_alignment";}
    if (!have_pose_ || !velocity_ready_) {return "waiting_for_pose_velocity";}
    if (!freshStamp(pose_ns_) || elapsed(pose_received_) > timeout_) {return "stale_pose";}
    if (!have_cloud_ || !freshStamp(cloud_ns_) || elapsed(cloud_received_) > timeout_) {
      return "stale_cloud";
    }
    return "ready";
  }
  tf2::Transform mapToOdom()
  {
    tf2::Quaternion yaw;
    yaw.setRPY(0., 0., get_parameter("map_odom_offset_yaw").as_double());
    const tf2::Transform offset(yaw, tf2::Vector3(
      get_parameter("map_odom_offset_x").as_double(),
      get_parameter("map_odom_offset_y").as_double(), 0.));
    return offset * initial_map_to_odom_;
  }
  void publishMapTransform(const rclcpp::Time & stamp)
  {
    Transform transform;
    transform.header.stamp = stamp; transform.header.frame_id = "map";
    transform.child_frame_id = "odom";
    transform.transform = tf2::toMsg(mapToOdom());
    tf_pub_->sendTransform(transform);
  }
  void odometry(const Odom & m)
  {
    checkClock();
    if (failed_ || m.header.frame_id != "odom" ||
      m.child_frame_id != "pointlio_imu") {return;}
    const auto stamp = stampNs(m.header.stamp);
    tf2::Transform imu;
    const auto & p = m.pose.pose.position;
    if (!decode(p.x, p.y, p.z, m.pose.pose.orientation, imu)) {return;}
    if (have_pose_ && stamp < pose_ns_ - 500000000) {fail(); return;}
    if (stamp <= pose_ns_ || !freshStamp(stamp)) {return;}
    const auto chassis = imu * chassis_to_imu_.inverse();
    if (!aligned_) {
      // This is a configured physical placement, not an inferred true pose.
      // Both the cloud and navigation subsequently use exactly this main TF.
      initial_map_to_odom_ = initial_base_ * chassis.inverse();
      alignment_ns_ = stamp; aligned_ = true;
      if (publish_map_to_odom_) {publishMapTransform(rclcpp::Time(m.header.stamp));}
    }
    Odom output;
    output.header.stamp = m.header.stamp; output.header.frame_id = "odom";
    output.child_frame_id = "base_link";
    tf2::toMsg(chassis, output.pose.pose);
    // Point-LIO snapshot publishes pose but leaves twist zero. Derive chassis
    // velocity from acquisition-time poses, NOT truth or commanded velocity.
    // Odometry twist is in child_frame_id (base_link), not map coordinates.
    const double dt = static_cast<double>(stamp - pose_ns_) * 1e-9;
    velocity_ready_ = have_pose_ && dt > 1e-6 && dt <= max_velocity_dt_;
    if (velocity_ready_) {
      const auto v = chassis.getBasis().transpose() *
        ((chassis.getOrigin() - previous_pose_.getOrigin()) / dt);
      auto delta = previous_pose_.getRotation().inverse() * chassis.getRotation();
      delta.normalize();
      if (delta.w() < 0.) {delta = tf2::Quaternion(-delta.x(), -delta.y(), -delta.z(), -delta.w());}
      const double sine = tf2::Vector3(delta.x(), delta.y(), delta.z()).length();
      tf2::Vector3 omega(0., 0., 0.);
      if (sine > 1e-12) {
        omega = tf2::Vector3(delta.x(), delta.y(), delta.z()) *
          (2. * std::atan2(sine, delta.w()) / (sine * dt));
      }
      output.twist.twist.linear.x = v.x(); output.twist.twist.linear.y = v.y();
      output.twist.twist.linear.z = v.z(); output.twist.twist.angular.x = omega.x();
      output.twist.twist.angular.y = omega.y(); output.twist.twist.angular.z = omega.z();
    } else {stop();}
    // No covariance is invented: upstream snapshot does not publish one.
    pose_ns_ = stamp; previous_pose_ = chassis;
    pose_received_ = Steady::now(); have_pose_ = true;
    odom_pub_->publish(output);
    Odom global = output;
    try {
      tf2::Transform alignment;
      if (publish_map_to_odom_) {
        alignment = mapToOdom();
      } else {
        const auto tf = tf_buffer_->lookupTransform("map", "odom", m.header.stamp);
        tf2::fromMsg(tf.transform, alignment);
      }
      global.header.frame_id = "map";
      tf2::toMsg(alignment * chassis, global.pose.pose);
      map_odom_pub_->publish(global);
    } catch (const tf2::TransformException &) {
      // No latest-pose or simulator fallback for a missing acquisition-time TF.
    }
    // Preserve the existing URDF ground anchor when odom's native origin is at
    // the IMU, rather than silently moving the model vertically. These two
    // factors still multiply to the RAW odom->body pose: the initialization
    // ground-plane basis cancels, and no global correction enters odometry.
    const auto ground_body = initial_map_to_odom_ * chassis;
    tf2::Quaternion yaw; yaw.setRPY(0., 0., tf2::getYaw(ground_body.getRotation()));
    const tf2::Transform ground_footprint(yaw,
      tf2::Vector3(ground_body.getOrigin().x(), ground_body.getOrigin().y(), 0.));
    const auto footprint = initial_map_to_odom_.inverse() * ground_footprint;
    Transform planar, body;
    planar.header = output.header; planar.child_frame_id = "base_footprint";
    planar.transform = tf2::toMsg(footprint);
    body.header.stamp = m.header.stamp; body.header.frame_id = "base_footprint";
    body.child_frame_id = "base_link"; body.transform = tf2::toMsg(footprint.inverse() * chassis);
    tf_pub_->sendTransform(std::vector<Transform>{planar, body});
  }
  void command(const Twist & m)
  {
    if (!relay_) {return;}
    checkClock();
    const double values[] = {m.linear.x, m.linear.y, m.linear.z,
      m.angular.x, m.angular.y, m.angular.z};
    const bool valid = std::all_of(std::begin(values), std::end(values),
      [](double v) {return std::isfinite(v);});
    if (!valid || health() != "ready") {cmd_pub_->publish(Twist()); active_command_ = false; return;}
    cmd_pub_->publish(m); command_received_ = Steady::now(); active_command_ = true;
  }
  bool publish_map_to_odom_{true};
  bool relay_{false}, aligned_{false}, failed_{false}, have_pose_{false}, have_cloud_{false};
  bool velocity_ready_{false}, active_command_{false};
  double timeout_, cmd_timeout_, max_velocity_dt_;
  int64_t alignment_ns_{0}, pose_ns_{0}, cloud_ns_{0}, last_clock_ns_{0};
  tf2::Transform chassis_to_imu_, initial_map_to_odom_{tf2::Transform::getIdentity()}, previous_pose_, initial_base_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  Steady::time_point pose_received_, cloud_received_, command_received_;
  rclcpp::Publisher<Odom>::SharedPtr odom_pub_, map_odom_pub_;
  rclcpp::Publisher<Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<Twist>::SharedPtr cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealPointLioNavigationBridge>());
  rclcpp::shutdown(); return 0;
}
