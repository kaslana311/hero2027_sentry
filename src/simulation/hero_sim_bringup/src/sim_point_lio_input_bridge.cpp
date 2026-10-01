#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"

// Input-only adapter. No odometry, TF or motor subscriptions/publications.
// Gazebo gpu_lidar is a snapshot: each ray has the header acquisition time.
class SimPointLioInputBridge : public rclcpp::Node
{
  using Cloud = sensor_msgs::msg::PointCloud2;
  using Scan = livox_ros_driver2::msg::CustomMsg;
  using Cache = std::map<int64_t, Cloud::ConstSharedPtr>;

public:
  SimPointLioInputBridge() : Node("sim_point_lio_input_bridge")
  {
    const auto left = pose(declare_parameter<std::vector<double>>(
      "left_lidar_pose", {-0.031029689, 0.207956386, 0.103822859,
        0.004491681, 0.593489259, 1.574538158}));
    const auto right = pose(declare_parameter<std::vector<double>>(
      "right_lidar_pose", {0., -0.217, 0.121, 0., 0.61086, -1.57079}));
    const auto ray = pose(declare_parameter<std::vector<double>>(
      "lidar_sensor_pose", {0., 0., 0.03, 0., 0., 0.}));
    // Origin is the right IMU; axes are chassis-aligned. Rotating measured
    // vectors does not move the IMU, so no lever-arm correction is fabricated.
    const tf2::Transform virtual_imu(tf2::Quaternion(0., 0., 0., 1.), right.getOrigin());
    left_to_imu_ = virtual_imu.inverse() * left * ray;
    right_to_imu_ = virtual_imu.inverse() * right * ray;
    imu_rotation_ = right.getBasis();
    frame_ = declare_parameter<std::string>("imu_frame", "pointlio_imu");
    if (frame_.empty()) {throw std::runtime_error("imu_frame must not be empty");}
    scan_pub_ = create_publisher<Scan>("input/fused_cloud", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("input/imu", 100);
    left_sub_ = create_subscription<Cloud>(
      declare_parameter<std::string>("left_topic", "livox/lidar_left"),
      rclcpp::SensorDataQoS(), [this](Cloud::ConstSharedPtr msg) {cloud(msg, true);});
    right_sub_ = create_subscription<Cloud>(
      declare_parameter<std::string>("right_topic", "livox/lidar_right"),
      rclcpp::SensorDataQoS(), [this](Cloud::ConstSharedPtr msg) {cloud(msg, false);});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter<std::string>("imu_topic", "livox/imu_right"),
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {imu(msg);});
    RCLCPP_INFO(get_logger(), "Point-LIO snapshot adapter: exact-stamp dual scans, "
      "zero per-ray offsets; SI IMU at right mount, chassis-aligned axes. No truth input.");
  }

private:
  static tf2::Transform pose(const std::vector<double> & value)
  {
    if (value.size() != 6 || !std::all_of(value.begin(), value.end(),
        [](double v) {return std::isfinite(v);}))
    {throw std::runtime_error("Mount must contain six finite x/y/z/roll/pitch/yaw values");}
    tf2::Quaternion q;
    q.setRPY(value[3], value[4], value[5]);
    return tf2::Transform(q, tf2::Vector3(value[0], value[1], value[2]));
  }

  static bool sensorFrame(const std::string & frame, const std::string & expected)
  {
    return frame.substr(frame.find_last_of('/') + 1) == expected;
  }

  static int field(const Cloud & msg, const std::string & name, bool required)
  {
    for (const auto & f : msg.fields) {
      if (f.name == name && f.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
        f.count == 1 && static_cast<uint64_t>(f.offset) + 4 <= msg.point_step)
      {return static_cast<int>(f.offset);}
    }
    if (required) {throw std::runtime_error("Missing/invalid FLOAT32 field: " + name);}
    return -1;
  }

  static float readFloat(const uint8_t * data, int offset)
  {
    float value;
    std::memcpy(&value, data + offset, sizeof(value));
    return value;
  }

  static void append(Scan & output, const Cloud & msg, const tf2::Transform & transform)
  {
    const uint64_t count = static_cast<uint64_t>(msg.width) * msg.height;
    if (msg.is_bigendian || count == 0 || count > 50000 || msg.point_step == 0 ||
      static_cast<uint64_t>(msg.row_step) < static_cast<uint64_t>(msg.width) * msg.point_step ||
      msg.data.size() < static_cast<uint64_t>(msg.row_step) * msg.height)
    {throw std::runtime_error("Invalid/oversized little-endian snapshot layout");}
    const int x = field(msg, "x", true), y = field(msg, "y", true);
    const int z = field(msg, "z", true), intensity = field(msg, "intensity", false);
    output.points.reserve(output.points.size() + count);
    for (uint32_t row = 0; row < msg.height; ++row) {
      for (uint32_t col = 0; col < msg.width; ++col) {
        const auto * data = msg.data.data() + static_cast<size_t>(row) * msg.row_step +
          static_cast<size_t>(col) * msg.point_step;
        const tf2::Vector3 raw(readFloat(data, x), readFloat(data, y), readFloat(data, z));
        if (!std::isfinite(raw.x()) || !std::isfinite(raw.y()) || !std::isfinite(raw.z())) {
          continue;
        }
        const auto point = transform * raw;
        livox_ros_driver2::msg::CustomPoint out;
        out.x = point.x(); out.y = point.y(); out.z = point.z();
        const float reflectivity = intensity >= 0 ? readFloat(data, intensity) : 0.f;
        out.reflectivity = std::isfinite(reflectivity) ?
          static_cast<uint8_t>(std::max(0.f, std::min(reflectivity, 255.f))) : 0;
        out.line = 0;  // Synthetic channel, not a Livox scan-pattern emulator.
        out.tag = 0x10;
        out.offset_time = 0;  // All points belong to this one rendered snapshot.
        output.points.push_back(out);
      }
    }
  }

  void cloud(Cloud::ConstSharedPtr msg, bool is_left)
  {
    const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
    if (stamp <= last_published_ || !sensorFrame(msg->header.frame_id,
        is_left ? "left_mid360" : "right_mid360"))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Rejecting zero/old stamp or unexpected lidar frame; restart after simulator time reset");
      return;
    }
    auto & pending = is_left ? left_ : right_;
    pending[stamp] = msg;
    if (pending.size() > 5) {
      pending.erase(pending.begin());
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Dual snapshot queue full: scans require exactly equal acquisition stamps");
    }
    const auto l = left_.find(stamp), r = right_.find(stamp);
    if (l == left_.end() || r == right_.end()) {return;}
    Scan output;
    output.header = l->second->header;
    output.header.frame_id = frame_;
    output.timebase = static_cast<uint64_t>(stamp);
    output.lidar_id = 0;
    try {
      append(output, *l->second, left_to_imu_);
      append(output, *r->second, right_to_imu_);
      if (!output.points.empty()) {
        output.point_num = output.points.size();
        scan_pub_->publish(output);
        last_published_ = stamp;
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Dropping snapshot pair: %s", error.what());
    }
    left_.erase(left_.begin(), left_.upper_bound(stamp));
    right_.erase(right_.begin(), right_.upper_bound(stamp));
  }

  void imu(sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if (rclcpp::Time(msg->header.stamp).nanoseconds() <= 0 ||
      !sensorFrame(msg->header.frame_id, "right_mid360")) {return;}
    auto output = *msg;
    output.header.frame_id = frame_;
    auto rotate = [this](const auto & vector) {
        return imu_rotation_ * tf2::Vector3(vector.x, vector.y, vector.z);
      };
    const auto acc = rotate(msg->linear_acceleration), gyro = rotate(msg->angular_velocity);
    if (!std::isfinite(acc.length2()) || !std::isfinite(gyro.length2())) {return;}
    output.linear_acceleration.x = acc.x(); output.linear_acceleration.y = acc.y();
    output.linear_acceleration.z = acc.z();
    output.angular_velocity.x = gyro.x(); output.angular_velocity.y = gyro.y();
    output.angular_velocity.z = gyro.z();
    auto covariance = [this](const auto & input, auto & result) {
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            result[i * 3 + j] = 0.;
            for (int k = 0; k < 3; ++k) {
              for (int l = 0; l < 3; ++l) {
                result[i * 3 + j] += imu_rotation_[i][k] * input[k * 3 + l] *
                  imu_rotation_[j][l];
              }
            }
          }
        }
      };
    covariance(msg->linear_acceleration_covariance, output.linear_acceleration_covariance);
    covariance(msg->angular_velocity_covariance, output.angular_velocity_covariance);
    // Never feed Gazebo's absolute orientation to the estimator as a shortcut.
    output.orientation = geometry_msgs::msg::Quaternion();
    output.orientation.w = 1.;
    output.orientation_covariance.fill(0.);
    output.orientation_covariance[0] = -1.;
    imu_pub_->publish(output);
  }

  std::string frame_;
  tf2::Transform left_to_imu_, right_to_imu_;
  tf2::Matrix3x3 imu_rotation_;
  Cache left_, right_;
  int64_t last_published_{0};
  rclcpp::Publisher<Scan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<Cloud>::SharedPtr left_sub_, right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimPointLioInputBridge>());
  rclcpp::shutdown();
  return 0;
}
