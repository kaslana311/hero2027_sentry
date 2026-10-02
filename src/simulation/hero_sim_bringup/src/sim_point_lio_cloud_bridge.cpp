#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Transform.h"

// Cloud-only deployment: Point-LIO already registered these points in its fixed
// local world. Apply ONE initial rigid alignment, not a moving base-link TF.
// No truth/odom subscriptions, motor output, global TF or truth-cloud fallback.
class SimPointLioCloudBridge : public rclcpp::Node
{
  using Cloud = sensor_msgs::msg::PointCloud2;
  using Alignment = geometry_msgs::msg::TransformStamped;
  using WallClock = std::chrono::steady_clock;
  struct Pending {Cloud::ConstSharedPtr cloud; WallClock::time_point received;};
  struct Layout {
    std::array<uint32_t, 3> xyz;
    std::array<uint32_t, 3> normal;
    bool has_normal{false};
  };
  static constexpr size_t MAX_FRAME_BYTES = 16 * 1024 * 1024;

public:
  SimPointLioCloudBridge() : Node("sim_point_lio_cloud_bridge")
  {
    const auto depth = declare_parameter<int>("cloud_queue_size", 5);
    wait_sec_ = declare_parameter<double>("alignment_wait_sec", 1.);
    if (depth < 1 || depth > 20 || !std::isfinite(wait_sec_) || wait_sec_ <= 0. ||
      wait_sec_ > 10.)
    {throw std::runtime_error("cloud_queue_size must be 1..20; alignment_wait_sec in (0,10]");}
    queue_size_ = static_cast<size_t>(depth);
    // DOG map requests reliable registered clouds, like the truth bridge output.
    // A best-effort publisher cannot connect to that subscriber. Keep the raw
    // Point-LIO input sensor QoS, but honor the downstream contract here.
    cloud_pub_ = create_publisher<Cloud>(
      "cloud_registered_map", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "cloud_deployment/status", rclcpp::QoS(1).transient_local());
    alignment_sub_ = create_subscription<Alignment>(
      "evaluation/cloud_alignment", rclcpp::QoS(1).transient_local(),
      [this](Alignment::ConstSharedPtr msg) {align(*msg);});
    cloud_sub_ = create_subscription<Cloud>("cloud_registered", rclcpp::SensorDataQoS(),
      [this](Cloud::ConstSharedPtr msg) {receive(msg);});
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        const auto now = get_clock()->now().nanoseconds();
        if (last_clock_ > 0 && now < last_clock_ - 500000000) {fail("simulation clock reset");}
        last_clock_ = now;
        drain();
        publishStatus();
      });
    RCLCPP_INFO(get_logger(), "Point-LIO cloud deployment waiting for ONE initial alignment; "
      "output=cloud_registered_map. No truth fallback; navigation pose is NOT replaced.");
    publishStatus();
  }

private:
  static int64_t timestamp(const builtin_interfaces::msg::Time & stamp)
  {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) {return 0;}
    return static_cast<int64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
  }

  static bool offsets(const Cloud & cloud, const std::array<std::string, 3> & names,
    std::array<uint32_t, 3> & result)
  {
    for (size_t axis = 0; axis < 3; ++axis) {
      const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [&](const sensor_msgs::msg::PointField & f) {return f.name == names[axis];});
      if (field == cloud.fields.end() ||
        std::count_if(cloud.fields.begin(), cloud.fields.end(),
        [&](const sensor_msgs::msg::PointField & f) {return f.name == names[axis];}) != 1 ||
        field->count != 1 || field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
        static_cast<uint64_t>(field->offset) + sizeof(float) > cloud.point_step)
      {return false;}
      result[axis] = field->offset;
    }
    return true;
  }

  static bool layout(const Cloud & cloud, Layout & out)
  {
    const uint16_t endian_probe = 1;
    if (cloud.is_bigendian || *reinterpret_cast<const uint8_t *>(&endian_probe) != 1 ||
      cloud.width == 0 || cloud.height == 0 || cloud.point_step == 0 ||
      cloud.data.size() > MAX_FRAME_BYTES ||
      static_cast<uint64_t>(cloud.row_step) <
      static_cast<uint64_t>(cloud.width) * cloud.point_step ||
      static_cast<uint64_t>(cloud.row_step) * cloud.height != cloud.data.size() ||
      !offsets(cloud, {{"x", "y", "z"}}, out.xyz))
    {return false;}
    const std::array<std::string, 3> normals{{"normal_x", "normal_y", "normal_z"}};
    out.has_normal = std::any_of(cloud.fields.begin(), cloud.fields.end(),
      [&](const sensor_msgs::msg::PointField & f) {
        return std::find(normals.begin(), normals.end(), f.name) != normals.end();
      });
    if (out.has_normal && !offsets(cloud, normals, out.normal)) {return false;}
    std::array<uint32_t, 6> all{};
    std::copy(out.xyz.begin(), out.xyz.end(), all.begin());
    const size_t count = out.has_normal ? 6 : 3;
    if (out.has_normal) {std::copy(out.normal.begin(), out.normal.end(), all.begin() + 3);}
    std::sort(all.begin(), all.begin() + count);
    for (size_t i = 1; i < count; ++i) {
      if (static_cast<uint64_t>(all[i - 1]) + sizeof(float) > all[i]) {return false;}
    }
    return true;
  }

  void align(const Alignment & msg)
  {
    if (failed_) {return;}
    const int64_t stamp = timestamp(msg.header.stamp);
    const auto & t = msg.transform.translation;
    const auto & q = msg.transform.rotation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (msg.header.frame_id != "map" || msg.child_frame_id != "pointlio_odom" ||
      stamp <= 0 || !std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z) ||
      !std::isfinite(norm) || norm < 1e-12)
    {++dropped_; return;}
    tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
    rotation.normalize();
    const tf2::Transform pose(rotation, tf2::Vector3(t.x, t.y, t.z));
    if (aligned_) {
      const auto delta = map_to_lio_.inverse() * pose;
      if (stamp != alignment_stamp_ || delta.getOrigin().length() > 1e-9 ||
        delta.getRotation().getAngleShortestPath() > 1e-9)
      {fail("alignment changed: restart the complete experiment");}
      return;  // Ignore identical latched messages; never update the transform.
    }
    map_to_lio_ = pose;
    alignment_stamp_ = stamp;
    aligned_ = true;
    RCLCPP_INFO(get_logger(), "Cloud alignment frozen at %.9f s; future LIO drift is retained",
      stamp * 1e-9);
    drain();
    publishStatus();
  }

  void receive(Cloud::ConstSharedPtr msg)
  {
    if (failed_) {return;}
    Layout fields;
    const auto stamp = timestamp(msg->header.stamp);
    if (msg->header.frame_id != "pointlio_odom" || stamp <= 0 || !layout(*msg, fields)) {
      ++dropped_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Rejecting invalid Point-LIO cloud stamp/frame/layout (FLOAT32 little-endian required)");
      return;
    }
    if (last_input_ > 0 && stamp < last_input_ - 500000000) {
      fail("Point-LIO cloud time reset"); return;
    }
    if (stamp <= last_input_) {++dropped_; return;}
    last_input_ = stamp;
    pending_.push_back({msg, WallClock::now()});
    if (pending_.size() > queue_size_) {pending_.pop_front(); ++dropped_;}
    drain();
  }

  static bool transformVector(uint8_t * bytes, const std::array<uint32_t, 3> & fields,
    const tf2::Transform & pose, bool normal, bool & dense)
  {
    std::array<float, 3> values{};
    for (size_t i = 0; i < 3; ++i) {std::memcpy(&values[i], bytes + fields[i], sizeof(float));}
    if (!std::all_of(values.begin(), values.end(), [](float v) {return std::isfinite(v);})) {
      dense = false; return true;  // Preserve missing returns; do not invent points.
    }
    const tf2::Vector3 input(values[0], values[1], values[2]);
    const auto output = normal ? pose.getBasis() * input : pose * input;
    for (size_t i = 0; i < 3; ++i) {
      if (!std::isfinite(output[i]) || std::abs(output[i]) > std::numeric_limits<float>::max())
      {return false;}
      values[i] = static_cast<float>(output[i]);
      std::memcpy(bytes + fields[i], &values[i], sizeof(float));
    }
    return true;
  }

  void drain()
  {
    if (failed_) {return;}
    while (!pending_.empty()) {
      if (std::chrono::duration<double>(WallClock::now() - pending_.front().received).count()
        > wait_sec_)
      {pending_.pop_front(); ++dropped_; continue;}
      if (!aligned_) {return;}
      const auto input = pending_.front().cloud;
      pending_.pop_front();
      if (timestamp(input->header.stamp) < alignment_stamp_) {++dropped_; continue;}
      Layout fields;
      if (!layout(*input, fields)) {++dropped_; continue;}
      Cloud output = *input;
      bool valid = true;
      for (uint32_t row = 0; row < output.height && valid; ++row) {
        for (uint32_t col = 0; col < output.width && valid; ++col) {
          auto * bytes = output.data.data() + static_cast<size_t>(row) * output.row_step +
            static_cast<size_t>(col) * output.point_step;
          valid = transformVector(bytes, fields.xyz, map_to_lio_, false, output.is_dense);
          if (valid && fields.has_normal) {
            valid = transformVector(bytes, fields.normal, map_to_lio_, true, output.is_dense);
          }
        }
      }
      if (!valid) {++dropped_; continue;}
      output.header.frame_id = "map";
      // Header acquisition stamp, intensity, dimensions and row padding stay unchanged.
      cloud_pub_->publish(output);
      ++published_;
      last_output_ = timestamp(output.header.stamp);
      last_publish_wall_ = WallClock::now();
    }
  }

  void fail(const std::string & reason)
  {
    if (failed_) {return;}
    failed_ = true;
    pending_.clear();
    reason_ = reason;
    RCLCPP_ERROR(get_logger(), "%s; cloud output STOPPED, no truth fallback. "
      "Cancel navigation and restart the whole algorithm launch.", reason.c_str());
    publishStatus();
  }

  void publishStatus()
  {
    const double age = published_ == 0 ? -1. :
      std::chrono::duration<double>(WallClock::now() - last_publish_wall_).count();
    const std::string state = failed_ ? "restart_required" : !aligned_ ?
      "waiting_for_alignment" : published_ == 0 ? "waiting_for_cloud" :
      age > 1. ? "stale" : "publishing";
    std::ostringstream stream;
    stream << "{\"state\":\"" << state << "\",\"published\":" << published_ <<
      ",\"dropped\":" << dropped_ << ",\"queued\":" << pending_.size() <<
      ",\"alignment_stamp_sec\":" << alignment_stamp_ * 1e-9 <<
      ",\"last_cloud_stamp_sec\":" << last_output_ * 1e-9 <<
      ",\"wall_age_sec\":" << age << ",\"reason\":\"" << reason_ << "\"}";
    std_msgs::msg::String msg;
    msg.data = stream.str();
    status_pub_->publish(msg);
  }

  size_t queue_size_{5}, published_{0}, dropped_{0};
  double wait_sec_{1.};
  bool aligned_{false}, failed_{false};
  int64_t alignment_stamp_{0}, last_input_{0}, last_output_{0}, last_clock_{0};
  tf2::Transform map_to_lio_;
  std::string reason_;
  std::deque<Pending> pending_;
  WallClock::time_point last_publish_wall_{WallClock::now()};
  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  rclcpp::Subscription<Alignment>::SharedPtr alignment_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SimPointLioCloudBridge>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("sim_point_lio_cloud_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
