#include "gimbal_node.hpp"

GimbalNode::GimbalNode() : Node("gimbal_node") {
  // 初始化串口
  if (!boost_asio_.Init("/dev/to_gimbal_nuc", 115200)) {
    RCLCPP_ERROR(this->get_logger(), "与视觉电脑通信串口初始化失败");
    rclcpp::shutdown();  // 结束程序
    return;
  }
  RCLCPP_INFO(this->get_logger(), "与视觉电脑通信串口初始化成功");

  reach_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "arrived_at_goal", 1,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        has_arrived_ = msg->data;
      });
  // 指针初始化
  current_goal_ = std::make_shared<geometry_msgs::msg::PointStamped>();
  decision_sub_ =this->create_subscription<serial_interfaces::msg::Gimbaldecision>(
      "gimbal_decision", 10,
      std::bind(&GimbalNode::DecisionCallBack, this,std::placeholders::_1));
  // 订阅目标点
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "lookahead_point", 10,
      std::bind(&GimbalNode::GoalCallBack, this, std::placeholders::_1));
  gimbal_cmd_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&GimbalNode::GimbalCmdTimerCallback, this));
  yaw_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&GimbalNode::YawTimerCallback, this));
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  RCLCPP_INFO(this->get_logger(), "节点已启动，等待目标点和定位数据...");
}

// 解决云台朝向
void GimbalNode::GoalCallBack(
    const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  // base_link系下的
  try{

    geometry_msgs::msg::PointStamped current_goal_ = *msg;
    // RCLCPP_INFO(this->get_logger(), );
    // std::cout << "msg_frame" << point_in_fake_base.header.frame_id<< std::endl;

    // point_in_fake_base.header.frame_id = "fake_base_link";
    // // point_in_fake_base.header.stamp = this->now();

    // geometry_msgs::msg::PointStamped point_in_base;
    // tf_buffer_->transform (point_in_fake_base,point_in_base,"base_link");
    // current_goal_->point.x = point_in_base.point.x;
    // current_goal_->point.y = point_in_base.point.y;
  }
 catch (const tf2::TransformException& ex) {
  RCLCPP_INFO(this->get_logger(), "无法获取变换: %s", ex.what());
  }

}

// 定时器完成对theta角的实时赋值
void GimbalNode::YawTimerCallback()
{
  dist_ = std::hypot(current_goal_->point.x, current_goal_->point.y);
  theta_ = std::atan2(-current_goal_->point.x, current_goal_->point.y);
  theta_ = theta_ * 180 / M_PI;
  // std::cout << "has_arrived:" << has_arrived_ << std::endl;
  if (!has_arrived_) {
    // theta_ *= dist_;
  } else {
   theta_ = 0.0;
  }
  theta_ = std::max(-2.0,std::min(2.0,theta_));
}

// 接受决策端消息，完成对自瞄数据的构建
void GimbalNode::DecisionCallBack(
  const serial_interfaces::msg::Gimbaldecision::SharedPtr msg) {
  std::cout << "是否巡逻:" << (int)msg->is_gimbal_patrol << std::endl;
  std::cout << "是否到达:" << (int)msg->is_reached << std::endl;
  std::cout << "是否自瞄:" << (int)msg->is_auto_aim << std::endl;
  std::cout << "自瞄模式:" << (int)msg->auto_aim_mode << std::endl;
  std::cout << "云台角度:" << msg->yaw_degrees << std::endl;
  //   "Hero",       // 0x01
  //   "Infantry3",  // 0x02
  //   "Infantry4",  // 0x03
  //   "Engineer",   // 0x04
  //   "Sentry",     // 0x05
  //   "Outpost",    // 0x06
  //   "Base"        // 0x07
  is_gimbal_patrol_ = msg->is_gimbal_patrol;
  builder_.clearTargets();
  for (auto target : msg->targets) {
    builder_.addTarget(target.type, target.priority);
    std::cout << "读到的目标: 类型=" << (int)target.type
              << ", 优先级=" << (int)target.priority << std::endl;
  }
}

// 只负责发送串口数据，不再参与构建数据内容
void GimbalNode::GimbalCmdTimerCallback() {
  builder_.setAutoAim(true, static_cast<GimbalProtocol::AutoAimMode>(1));
  builder_.setYaw(theta_);
  builder_.setReached(has_arrived_);
  builder_.setPatrol(is_gimbal_patrol_);
  theta_ = 0.0;
  auto buffer = builder_.build();
  boost::asio::mutable_buffer send_buffer = boost::asio::buffer(buffer, 27);
  // std::cout << "发送给云台数据" << std::endl;
  boost_asio_.Write(send_buffer);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GimbalNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
