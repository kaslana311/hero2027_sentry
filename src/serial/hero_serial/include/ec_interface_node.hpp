#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <interfaces/msg/detail/readpack__struct.hpp>
#include <interfaces/msg/detail/sendpack__struct.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/bool.hpp>

#include "boost_asio.hpp"
#include "cv_to_ec_protocol_tool.hpp"
#include "ec_to_cv_protocol_tool.hpp"
#include "interfaces/msg/readpack.hpp"
#include "interfaces/msg/sendpack.hpp"
#include "interfaces/msg/timeconvert.hpp"

// ec 电控
// cv或vision 视觉

class EcInterfaceNode : public rclcpp::Node { 
 public:
  // 构造函数
  EcInterfaceNode();
  // 析构函数
  ~EcInterfaceNode() = default;

 private:
  // 成员变量
  // 串口
  BoostAsio boost_asio_;
  // 电控到视觉协议解析器
  combat_protocol::ProtocolParser ec_to_cv_parser_;
  // 视觉到电控协议构建器
  gimbal_protocol::ProtocolBuilder cv_to_ec_builder_;

  // ROS2接口
  // 发布电控到视觉协议
  rclcpp::Publisher<interfaces::msg::Readpack>::SharedPtr pack_pub_;
  // 订阅视觉到电控协议
  rclcpp::Subscription<interfaces::msg::Sendpack>::SharedPtr pack_sub_;

  // 电控到视觉协议消息
  interfaces::msg::Readpack::SharedPtr read_pack_;
  // 视觉到电控协议消息
  interfaces::msg::Sendpack::SharedPtr send_pack_;

  // 定时器
  // 电控到视觉协议定时器，收
  rclcpp::TimerBase::SharedPtr ec_to_vision_timer_;
  // 视觉到电控协议定时器，发
  rclcpp::TimerBase::SharedPtr vision_to_ec_timer_;

  // 回调函数
  // 订阅自瞄的数据，存入视觉到电控的协议构建器
  void SendpackCallback(const interfaces::msg::Sendpack::SharedPtr msg);
  // 电控到视觉协议定时器回调函数
  void EcToVisionTimerCallback();
  // 视觉到电控协议定时器回调函数
  void VisionToEcTimerCallback();
  // 查找帧头
  size_t FindFrameHeader(const std::vector<uint8_t>& data);

  // 时间转换相关，把电控时间转换为视觉时间，或者把视觉时间转换为电控时间
  double time_scale_factor_ = 1.0;  // 时间比例系数 a
  int64_t time_offset_ = 0;         // 时间偏移量 b
  
  // 时间同步点记录
  struct TimePoint {
    int64_t mcu_time;    // 电控时间
    int64_t vision_time; // 视觉时间
  };
  std::vector<TimePoint> sync_points_;  // 存储同步点
  
  // 时间转换相关函数
  // 整合电控时间，电控高位加电控低位
  int64_t CombineMcuTime(int64_t high_time, int64_t low_time);
  // 更新时间校准参数
  void UpdateTimeCalibration();
  // 把电控时间转换为视觉时间
  int64_t ConvertMcuToVisionTime(int64_t mcu_time);
  // 把视觉时间转换为电控时间
  int64_t ConvertVisionToMcuTime(int64_t vision_time);
  void RecordSyncPoint(int64_t mcu_time, int64_t vision_time);
  
  static constexpr size_t MIN_SYNC_POINTS = 2;   // 最少需要的同步点数量
  static constexpr size_t MAX_SYNC_POINTS = 10;  // 最大同步点数量
  static constexpr int64_t MAX_TIME_DIFF = 1000; // 最大时间差异阈值（毫秒）
};
