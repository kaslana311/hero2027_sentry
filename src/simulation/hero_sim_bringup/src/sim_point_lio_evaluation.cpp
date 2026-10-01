#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <iterator>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

// Observer only. Truth never goes back to Point-LIO, DOG map, navigation or motors.
class SimPointLioEvaluation : public rclcpp::Node
{
  using Odom = nav_msgs::msg::Odometry;
  using Marker = visualization_msgs::msg::Marker;
  using Markers = visualization_msgs::msg::MarkerArray;
  using Clock = std::chrono::steady_clock;
  struct Pending {Odom::ConstSharedPtr message; Clock::time_point received;};

public:
  SimPointLioEvaluation() : Node("sim_point_lio_evaluation")
  {
    const auto offset = declare_parameter<std::vector<double>>(
      "chassis_to_virtual_imu", {0., -0.217, 0.121});
    if (offset.size() != 3 || !std::all_of(offset.begin(), offset.end(),
        [](double v) {return std::isfinite(v);}))
    {throw std::runtime_error("chassis_to_virtual_imu needs three finite coordinates");}
    chassis_to_imu_ = tf2::Transform(tf2::Quaternion(0., 0., 0., 1.),
      tf2::Vector3(offset[0], offset[1], offset[2]));
    max_gap_ = declare_parameter<double>("truth_max_gap_sec", 0.15);
    if (!std::isfinite(max_gap_) || max_gap_ <= 0.) {
      throw std::runtime_error("truth_max_gap_sec must be positive");
    }
    markers_pub_ = create_publisher<Markers>(
      "evaluation/markers", rclcpp::QoS(1).transient_local());
    truth_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("evaluation/truth_pose", 10);
    estimate_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "evaluation/pointlio_pose", 10);
    truth_path_pub_ = create_publisher<nav_msgs::msg::Path>("evaluation/truth_path", 1);
    estimate_path_pub_ = create_publisher<nav_msgs::msg::Path>("evaluation/pointlio_path", 1);
    error_pub_ = create_publisher<std_msgs::msg::String>("evaluation/error", 10);
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    truth_sub_ = create_subscription<Odom>(
      declare_parameter<std::string>("truth_topic", "truth_3d"), rclcpp::SensorDataQoS(),
      [this](Odom::ConstSharedPtr msg) {truth(msg);});
    estimate_sub_ = create_subscription<Odom>("odometry", rclcpp::SensorDataQoS(),
      [this](Odom::ConstSharedPtr msg) {estimate(msg);});
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
        process();
        if (!aligned_ || failed_) {publishWaiting();}
      });
    RCLCPP_INFO(get_logger(), "Display observer: fixed map/odom origin, one-time alignment; "
      "yellow=origin, green=truth chassis, blue=Point-LIO chassis. No feedback.");
  }

private:
  static bool decode(const Odom & msg, int64_t & stamp, tf2::Transform & pose)
  {
    if (msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000u) {return false;}
    stamp = static_cast<int64_t>(msg.header.stamp.sec) * 1000000000 + msg.header.stamp.nanosec;
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (stamp <= 0 || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(norm) || norm < 1e-12)
    {return false;}
    tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
    rotation.normalize();
    pose = tf2::Transform(rotation, tf2::Vector3(p.x, p.y, p.z));
    return true;
  }

  void truth(Odom::ConstSharedPtr msg)
  {
    int64_t stamp;
    tf2::Transform pose;
    if (msg->header.frame_id != "map" || !decode(*msg, stamp, pose) || failed_) {return;}
    // A 0.5 s backwards jump is a simulator reset, not a network reordering.
    if (!truth_.empty() && stamp < truth_.rbegin()->first - 500000000) {timeReset(); return;}
    truth_[stamp] = pose;
    while (truth_.size() > 400) {truth_.erase(truth_.begin());}
    process();
  }

  void estimate(Odom::ConstSharedPtr msg)
  {
    int64_t stamp;
    tf2::Transform pose;
    if (msg->header.frame_id != "pointlio_odom" || msg->child_frame_id != "pointlio_imu" ||
      !decode(*msg, stamp, pose) || failed_) {return;}
    if (last_estimate_ && stamp < last_estimate_ - 500000000) {timeReset(); return;}
    if (stamp <= last_estimate_) {return;}
    last_estimate_ = stamp;
    pending_.push_back({msg, Clock::now()});
    if (pending_.size() > 50) {pending_.pop_front();}
    process();
  }

  void timeReset()
  {
    failed_ = true;
    pending_.clear();
    RCLCPP_ERROR(get_logger(), "Simulation time reset: restart the whole shadow launch; "
      "the observer will not silently re-align and hide drift");
  }

  bool sample(int64_t stamp, tf2::Transform & pose) const
  {
    auto upper = truth_.lower_bound(stamp);
    if (upper == truth_.end()) {return false;}
    if (upper->first == stamp) {pose = upper->second; return true;}
    if (upper == truth_.begin()) {return false;}
    const auto lower = std::prev(upper);
    if ((upper->first - lower->first) * 1e-9 > max_gap_) {return false;}
    const double ratio = static_cast<double>(stamp - lower->first) / (upper->first - lower->first);
    pose.setOrigin(lower->second.getOrigin().lerp(upper->second.getOrigin(), ratio));
    pose.setRotation(lower->second.getRotation().slerp(upper->second.getRotation(), ratio));
    return true;
  }

  void process()
  {
    if (failed_) {return;}
    while (!pending_.empty()) {
      const auto msg = pending_.front().message;
      int64_t stamp;
      tf2::Transform local_imu, map_chassis;
      if (!decode(*msg, stamp, local_imu)) {pending_.pop_front(); continue;}
      if (!sample(stamp, map_chassis)) {
        if ((!truth_.empty() && stamp < truth_.begin()->first) ||
          Clock::now() - pending_.front().received > std::chrono::seconds(1))
        {
          pending_.pop_front();
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "No acquisition-time truth for comparison; dropping, no latest-pose fallback");
          continue;
        }
        return;
      }
      if (!aligned_) {
        map_to_lio_ = map_chassis * chassis_to_imu_ * local_imu.inverse();
        alignment_stamp_ = stamp;
        aligned_ = true;
        geometry_msgs::msg::TransformStamped transform;
        transform.header = msg->header;
        transform.header.frame_id = "map";
        transform.child_frame_id = "pointlio_odom";
        transform.transform = tf2::toMsg(map_to_lio_);
        // Launch remaps /tf_static to the private pointlio/tf_static topic.
        static_tf_->sendTransform(transform);
        RCLCPP_INFO(get_logger(), "Display alignment locked once at %.9f s; map/odom "
          "origin is unchanged. Initial pose error is zero by construction.", stamp * 1e-9);
      }
      const auto estimate_chassis = map_to_lio_ * local_imu * chassis_to_imu_.inverse();
      publishPair(*msg, map_chassis, estimate_chassis);
      pending_.pop_front();
    }
  }

  static Marker marker(int id, int type, const builtin_interfaces::msg::Time & stamp,
    float red, float green, float blue)
  {
    Marker out;
    out.header.frame_id = "map"; out.header.stamp = stamp;
    out.ns = "point_lio_comparison"; out.id = id; out.type = type; out.action = Marker::ADD;
    out.pose.orientation.w = 1.;
    out.color.r = red; out.color.g = green; out.color.b = blue; out.color.a = 1.f;
    return out;
  }

  Markers origin(const builtin_interfaces::msg::Time & stamp)
  {
    Markers result;
    auto zero = marker(0, Marker::SPHERE, stamp, 1.f, .85f, 0.f);
    zero.scale.x = zero.scale.y = zero.scale.z = .22;
    result.markers.push_back(zero);
    auto name = marker(1, Marker::TEXT_VIEW_FACING, stamp, 1.f, .85f, 0.f);
    name.pose.position.x = -.6; name.pose.position.y = .6; name.pose.position.z = .8;
    name.scale.z = .3; name.text = "MAP = ODOM ORIGIN\n(0, 0, 0)";
    result.markers.push_back(name);
    return result;
  }

  void body(Markers & result, int id, const tf2::Transform & pose,
    const builtin_interfaces::msg::Time & stamp, bool is_truth)
  {
    const float r = .1f, g = is_truth ? 1.f : .5f, b = is_truth ? .2f : 1.f;
    auto ring = marker(id, Marker::LINE_STRIP, stamp, r, g, b);
    ring.scale.x = .045;
    const double radius = is_truth ? .31 : .19;
    for (int i = 0; i <= 40; ++i) {
      geometry_msgs::msg::Point p;
      p.x = pose.getOrigin().x() + radius * std::cos(i * 2 * 3.141592653589793 / 40);
      p.y = pose.getOrigin().y() + radius * std::sin(i * 2 * 3.141592653589793 / 40);
      p.z = pose.getOrigin().z() + .08;
      ring.points.push_back(p);
    }
    ring.lifetime = rclcpp::Duration::from_seconds(.75);
    result.markers.push_back(ring);
    auto arrow = marker(id + 1, Marker::ARROW, stamp, r, g, b);
    tf2::toMsg(pose, arrow.pose); arrow.pose.position.z += .08;
    arrow.scale.x = is_truth ? .9 : .6;
    arrow.scale.y = is_truth ? .12 : .07; arrow.scale.z = is_truth ? .16 : .1;
    arrow.lifetime = ring.lifetime;
    result.markers.push_back(arrow);
    auto label = marker(id + 2, Marker::TEXT_VIEW_FACING, stamp, r, g, b);
    tf2::toMsg(pose, label.pose); label.pose.orientation = geometry_msgs::msg::Quaternion();
    label.pose.orientation.w = 1.;
    // Default near-overhead Orbit has X along screen height: put labels on
    // opposite sides vertically, so coincident poses still have readable names.
    label.pose.position.x += is_truth ? -1. : 1.;
    label.pose.position.y += is_truth ? .4 : -.4;
    label.pose.position.z += .8;
    std::ostringstream text;
    text << (is_truth ? "TRUTH chassis" : "POINT-LIO chassis") << std::fixed <<
      std::setprecision(2) << "\n(" << pose.getOrigin().x() << ", " << pose.getOrigin().y() <<
      ", " << pose.getOrigin().z() << ")";
    label.text = text.str(); label.scale.z = .25; label.lifetime = ring.lifetime;
    result.markers.push_back(label);
  }

  void publishWaiting()
  {
    const builtin_interfaces::msg::Time stamp = now();
    auto markers = origin(stamp);
    if (!truth_.empty() && !failed_) {
      body(markers, 10, truth_.rbegin()->second, stamp, true);
    }
    auto status = marker(30, Marker::TEXT_VIEW_FACING, stamp, 1.f, 1.f, 1.f);
    status.pose.position.x = -2.4; status.pose.position.y = .6; status.pose.position.z = .8;
    status.scale.z = .25;
    status.text = failed_ ? "TIME RESET: restart shadow" :
      "Waiting for time-matched truth + Point-LIO\nNo aligned Point-LIO marker yet";
    markers.markers.push_back(status);
    markers_pub_->publish(markers);
  }

  void publishPair(const Odom & msg, const tf2::Transform & truth, const tf2::Transform & estimate)
  {
    geometry_msgs::msg::PoseStamped actual, predicted;
    actual.header = predicted.header = msg.header;
    actual.header.frame_id = predicted.header.frame_id = "map";
    tf2::toMsg(truth, actual.pose); tf2::toMsg(estimate, predicted.pose);
    truth_pose_pub_->publish(actual); estimate_pose_pub_->publish(predicted);
    const auto delta = estimate.getOrigin() - truth.getOrigin();
    const auto rotation = truth.getRotation().inverse() * estimate.getRotation();
    const double angle = 2 * std::acos(std::min(1., std::abs(rotation.w()))) * 180 / 3.141592653589793;
    std::ostringstream values;
    values << std::fixed << std::setprecision(6) << "{\"stamp_sec\":" <<
      rclcpp::Time(msg.header.stamp).seconds() << ",\"xy_error_m\":" <<
      std::hypot(delta.x(), delta.y()) << ",\"position_error_m\":" << delta.length() <<
      ",\"attitude_error_deg\":" << angle << ",\"alignment_stamp_sec\":" <<
      alignment_stamp_ * 1e-9 << "}";
    std_msgs::msg::String error; error.data = values.str(); error_pub_->publish(error);
    auto markers = origin(msg.header.stamp);
    body(markers, 10, truth, msg.header.stamp, true);
    body(markers, 20, estimate, msg.header.stamp, false);
    auto status = marker(30, Marker::TEXT_VIEW_FACING, msg.header.stamp, 1.f, 1.f, 1.f);
    status.pose.position.x = -2.4; status.pose.position.y = .6; status.pose.position.z = .8;
    status.scale.z = .25;
    std::ostringstream text;
    text << std::fixed << std::setprecision(3) << "One-time aligned comparison\nXY error: " <<
      std::hypot(delta.x(), delta.y()) << " m  Angle: " << std::setprecision(2) << angle << " deg";
    status.text = text.str(); status.lifetime = rclcpp::Duration::from_seconds(.75);
    markers.markers.push_back(status);
    markers_pub_->publish(markers);
    const auto stamp = rclcpp::Time(msg.header.stamp).nanoseconds();
    if (last_path_ == 0 || stamp - last_path_ >= 100000000) {
      last_path_ = stamp;
      truth_path_.header = estimate_path_.header = actual.header;
      truth_path_.poses.push_back(actual); estimate_path_.poses.push_back(predicted);
      if (truth_path_.poses.size() > 1000) {
        truth_path_.poses.erase(truth_path_.poses.begin());
        estimate_path_.poses.erase(estimate_path_.poses.begin());
      }
      truth_path_pub_->publish(truth_path_); estimate_path_pub_->publish(estimate_path_);
    }
  }

  double max_gap_;
  bool aligned_{false}, failed_{false};
  int64_t alignment_stamp_{0}, last_estimate_{0}, last_path_{0};
  tf2::Transform chassis_to_imu_, map_to_lio_;
  std::map<int64_t, tf2::Transform> truth_;
  std::deque<Pending> pending_;
  nav_msgs::msg::Path truth_path_, estimate_path_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::Subscription<Odom>::SharedPtr truth_sub_, estimate_sub_;
  rclcpp::Publisher<Markers>::SharedPtr markers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr truth_pose_pub_, estimate_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr truth_path_pub_, estimate_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimPointLioEvaluation>());
  rclcpp::shutdown();
  return 0;
}
