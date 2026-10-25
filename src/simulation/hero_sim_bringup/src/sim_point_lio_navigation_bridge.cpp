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
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

// C2 adapter only: no truth subscription, no estimator/controller changes.
// Pose and cloud share exactly the same frozen, explicitly truth-seeded alignment.
class SimPointLioNavigationBridge : public rclcpp::Node
{
  using Odom = nav_msgs::msg::Odometry;
  using Twist = geometry_msgs::msg::Twist;
  using Transform = geometry_msgs::msg::TransformStamped;
  using Steady = std::chrono::steady_clock;
public:
  SimPointLioNavigationBridge() : Node("sim_point_lio_navigation_bridge")
  {
    relay_ = declare_parameter<bool>("relay_cmd_vel", true);
    timeout_ = declare_parameter<double>("data_timeout_sec", 1.0);
    cmd_timeout_ = declare_parameter<double>("command_timeout_sec", 0.5);
    max_velocity_dt_ = declare_parameter<double>("velocity_max_dt_sec", 0.5);
    if (!std::isfinite(timeout_) || timeout_ <= 0. || !std::isfinite(cmd_timeout_) ||
      cmd_timeout_ <= 0. || !std::isfinite(max_velocity_dt_) || max_velocity_dt_ <= 0.)
    {throw std::runtime_error("Timeouts must be finite and positive");}
    const auto offset = declare_parameter<std::vector<double>>(
      "chassis_to_virtual_imu", {0., -0.167, 0.121});
    if (offset.size() != 3 || !std::all_of(offset.begin(), offset.end(),
        [](double v) {return std::isfinite(v);}))
    {throw std::runtime_error("chassis_to_virtual_imu requires three finite coordinates");}
    chassis_to_imu_ = tf2::Transform(tf2::Quaternion(0., 0., 0., 1.),
      tf2::Vector3(offset[0], offset[1], offset[2]));
    odom_pub_ = create_publisher<Odom>("odometry/filtered", 10);
    cmd_pub_ = create_publisher<Twist>("cmd_vel", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "pointlio/navigation/status", rclcpp::QoS(1).transient_local());
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    static_tf_pub_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    Transform identity;
    identity.header.frame_id = "map";
    identity.child_frame_id = "odom";
    identity.transform.rotation.w = 1.;
    static_tf_pub_->sendTransform(identity);
    alignment_sub_ = create_subscription<Transform>("pointlio/evaluation/cloud_alignment",
      rclcpp::QoS(1).transient_local(), [this](Transform::ConstSharedPtr m) {align(*m);});
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
    RCLCPP_WARN(get_logger(), "C2: navigation uses Point-LIO pose + cloud; NO truth fallback. "
      "One-time initial alignment, body-frame finite-difference velocity. "
      "Reset/alignment change requires restarting the whole algorithm launch.");
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
    if (!failed_) {RCLCPP_ERROR(get_logger(), "Time reset or changed alignment: restart C2 launch");}
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
  void align(const Transform & m)
  {
    if (failed_ || m.header.frame_id != "map" || m.child_frame_id != "pointlio_odom") {return;}
    const auto stamp = stampNs(m.header.stamp);
    tf2::Transform transform;
    const auto & p = m.transform.translation;
    if (stamp <= 0 || !decode(p.x, p.y, p.z, m.transform.rotation, transform)) {return;}
    if (aligned_) {
      const double distance = (transform.getOrigin() - map_to_lio_.getOrigin()).length();
      const double angle = transform.getRotation().angleShortestPath(map_to_lio_.getRotation());
      if (stamp != alignment_ns_ || distance > 1e-8 || angle > 1e-8) {fail();}
      return;
    }
    map_to_lio_ = transform; alignment_ns_ = stamp; aligned_ = true;
  }
  void odometry(const Odom & m)
  {
    checkClock();
    if (failed_ || !aligned_ || m.header.frame_id != "pointlio_odom" ||
      m.child_frame_id != "pointlio_imu") {return;}
    const auto stamp = stampNs(m.header.stamp);
    tf2::Transform imu;
    const auto & p = m.pose.pose.position;
    if (!decode(p.x, p.y, p.z, m.pose.pose.orientation, imu)) {return;}
    if (have_pose_ && stamp < pose_ns_ - 500000000) {fail(); return;}
    if (stamp < alignment_ns_ || stamp <= pose_ns_ || !freshStamp(stamp)) {return;}
    const auto chassis = map_to_lio_ * imu * chassis_to_imu_.inverse();
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
    tf2::Quaternion yaw; yaw.setRPY(0., 0., tf2::getYaw(chassis.getRotation()));
    const tf2::Transform footprint(yaw,
      tf2::Vector3(chassis.getOrigin().x(), chassis.getOrigin().y(), 0.));
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
  bool relay_{true}, aligned_{false}, failed_{false}, have_pose_{false}, have_cloud_{false};
  bool velocity_ready_{false}, active_command_{false};
  double timeout_, cmd_timeout_, max_velocity_dt_;
  int64_t alignment_ns_{0}, pose_ns_{0}, cloud_ns_{0}, last_clock_ns_{0};
  tf2::Transform chassis_to_imu_, map_to_lio_, previous_pose_;
  Steady::time_point pose_received_, cloud_received_, command_received_;
  rclcpp::Publisher<Odom>::SharedPtr odom_pub_;
  rclcpp::Publisher<Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<Transform>::SharedPtr alignment_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<Twist>::SharedPtr cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimPointLioNavigationBridge>());
  rclcpp::shutdown(); return 0;
}
