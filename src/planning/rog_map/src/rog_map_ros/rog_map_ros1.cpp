
#include "rog_map_ros/rog_map_ros2.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rog_map_ros1");
  auto rog_map_ros = std::make_shared<rog_map::ROGMapROS>(node);
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}