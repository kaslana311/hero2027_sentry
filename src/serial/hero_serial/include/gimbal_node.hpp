#include <cmath>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "boost_asio.hpp"
#include "send_gimbal_protocol_tool.hpp"
#include "serial_interfaces/msg/gimbaldecision.hpp"
// 修改头文件引用方式
#include <action_msgs/msg/goal_status_array.hpp>  // 使用尖括号替代双引号
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class GimbalNode : public rclcpp::Node {
 public:
  GimbalNode();
  ~GimbalNode() = default;

 private:
  // 订阅预瞄点信息
  void GoalCallBack(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  // 发布到达目标点信息
  // void PublishArrival(bool arrived);
  // 订阅导航状态
  void NavStatusCallback(
      const action_msgs::msg::GoalStatusArray::SharedPtr msg);
  // 发送云台控制信息
  void GimbalCmdTimerCallback();
  // 订阅决策信息
  void DecisionCallBack(
      const serial_interfaces::msg::Gimbaldecision::SharedPtr msg);
  void YawTimerCallback();

  // 成员变量
  BoostAsio boost_asio_;
  geometry_msgs::msg::PointStamped::SharedPtr current_goal_ = nullptr;

  double dist_;
  double theta_;

  bool has_arrived_ = false;
  bool is_gimbal_patrol_ = true;

  // ROS2接口
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<serial_interfaces::msg::Gimbaldecision>::SharedPtr
      decision_sub_;
//   rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr
//       nav_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reach_subscription_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrival_pub_;
  rclcpp::TimerBase::SharedPtr gimbal_cmd_timer_;

  rclcpp::TimerBase::SharedPtr yaw_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  SendGimbalBuilder builder_;
};
