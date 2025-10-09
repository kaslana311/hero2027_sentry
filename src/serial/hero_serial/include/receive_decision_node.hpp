#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "boost_asio.hpp"
#include "send_gimbal_protocol_tool.hpp"
#include "serial_interfaces/msg/gimbaldecision.hpp"

class ReceiveDecisionNode : public rclcpp::Node {
 public:
  ReceiveDecisionNode();
  ~ReceiveDecisionNode() = default;

 private:
  BoostAsio boost_asio_;
  SendGimbalParser parser_;

  // serial_interfaces::msg::Gimbaldecision::SharedPtr gimbal_decision_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<serial_interfaces::msg::Gimbaldecision>::SharedPtr decision_pub_;
  void TimerCallback();
    // 查找帧头
  size_t FindFrameHeader(const std::vector<uint8_t>& data);


  


};