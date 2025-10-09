#include <cmath>
#include "receive_decision_node.hpp"


ReceiveDecisionNode::ReceiveDecisionNode() : Node("receive_decision_node")
{
  if (!boost_asio_.Init("/dev/to_chassis_nuc", 115200)) {
    RCLCPP_ERROR(this->get_logger(), "与导航电脑的串口初始化失败");
    rclcpp::shutdown();  // 结束程序
    return;
  }

  timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ReceiveDecisionNode::TimerCallback,
                                             this));

  decision_pub_ = this->create_publisher<serial_interfaces::msg::Gimbaldecision>(
      "decision", 10);

  RCLCPP_INFO(this->get_logger(), "ReceiveDecisionNode");
}

void ReceiveDecisionNode::TimerCallback()
{
  if (boost_asio_.Read()) {

    const auto& data = boost_asio_.GetReadBuffer();
    size_t processed = 0;

    while (true) {
      size_t header_pos = FindFrameHeader(data);
      if (header_pos == -1 || data.size() - header_pos < 27)
        break;

      
      // 提取数据包
      std::vector<uint8_t> packet(data.begin() + header_pos,
                                  data.begin() + header_pos + 27);

      // 解析协议
      if (parser_.parse(packet)) {

        serial_interfaces::msg::Gimbaldecision gimbal_decision_msg;
        gimbal_decision_msg.targets.resize(7);
        

        parser_.autoaim_mode();
        parser_.has_reached();
        parser_.is_patrolling();
        parser_.yaw_degrees();
        parser_.targets();

        gimbal_decision_msg.is_auto_aim = parser_.autoaim_enabled();
        gimbal_decision_msg.auto_aim_mode = parser_.autoaim_mode();
        gimbal_decision_msg.is_reached = parser_.has_reached();
        gimbal_decision_msg.is_gimbal_patrol = parser_.is_patrolling();
        gimbal_decision_msg.yaw_degrees = parser_.yaw_degrees();

        std::cout << "是否巡逻:" << (int)gimbal_decision_msg.is_gimbal_patrol << std::endl;
        std::cout << "是否到达:" << (int)gimbal_decision_msg.is_reached << std::endl;
        std::cout << "是否自瞄:" << (int)gimbal_decision_msg.is_auto_aim << std::endl;
        std::cout << "自瞄模式:" << (int)gimbal_decision_msg.auto_aim_mode << std::endl;
        std::cout << "云台角度:" << gimbal_decision_msg.yaw_degrees << std::endl;
        // gimbal_decision_msg.gimbal_target_angle = parser_.yaw_degrees();
        auto targets = parser_.targets();
        for (int i = 0; i < targets.size(); i++) {
          gimbal_decision_msg.targets[i].type = targets[i].type;
          gimbal_decision_msg.targets[i].priority = targets[i].priority;
          std::cout << "读到的目标: 类型=" << (int)targets[i].type
          << ", 优先级=" << (int)targets[i].priority << std::endl;
        }
        decision_pub_->publish(gimbal_decision_msg);
        
        processed = header_pos + 27;
      } else {
        // 校验失败，跳过错误帧头
        processed = header_pos + 1;
      }

      // 移除已处理数据
      boost_asio_.Consume(processed);
    }
  }
}

size_t ReceiveDecisionNode::FindFrameHeader(const std::vector<uint8_t>& data) {
  if (data.size() < 4)
    return -1;
  for (size_t i = 0; i <= data.size() - 4; ++i) {
    if (data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x80 &&
        data[i + 3] == 0x7F) {
      return i;
    }
  }
  return -1;
}
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReceiveDecisionNode>());
  rclcpp::shutdown();
  return 0;
}
