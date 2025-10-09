#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "serial_interfaces/msg/refree.hpp"
#include <thread>
#include <atomic>
#include <memory>

using namespace std::chrono_literals;

// 3.定义节点类；
class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      // 3-1.创建发布方；
      refree_pub_ = this->create_publisher<serial_interfaces::msg::Refree>("refree_msg",1);
      reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("reached_msg",1);
      // 3-2.创建定时器；
      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
      // 3-3.创建参数更新回调
      param_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&MinimalPublisher::parameters_callback, this, std::placeholders::_1));
      // 3-4.参数声明
      this->declare_parameter("competition_type", 0);
      this->declare_parameter("competition_stage", 0);
      this->declare_parameter("stage_time", 0);
      this->declare_parameter("current_hp",0);
      this->declare_parameter("blue_hero", 0);
      this->declare_parameter("is_reached", true);
      this->declare_parameter("blue_outpost", 0);
      this->declare_parameter("red_outpost", 0);
      this->declare_parameter("bullet_17mm",0);
      this->declare_parameter("hero_x",0.0);
      this->declare_parameter("hero_y",0.0);
    }

  private:
    void timer_callback()
    {
      // 3-3.组织消息并发布。
      auto message = serial_interfaces::msg::Refree();
      message.competition_type = this->get_parameter("competition_type").as_int();
      message.competition_stage = this->get_parameter("competition_stage").as_int();
      message.stage_time = this->get_parameter("stage_time").as_int();
      message.current_hp = this->get_parameter("current_hp").as_int();
      message.blue_hero = this->get_parameter("blue_hero").as_int();
      message.blue_outpost = this->get_parameter("blue_outpost").as_int();
      message.red_outpost = this->get_parameter("red_outpost").as_int();
      message.bullet_17mm = this->get_parameter("bullet_17mm").as_int();
      message.hero_x = this->get_parameter("hero_x").as_double();
      message.hero_y = this->get_parameter("hero_y").as_double();

      auto is_reached = std_msgs::msg::Bool();
      is_reached.data = this->get_parameter("is_reached").as_bool();

      // RCLCPP_INFO(this->get_logger(), "发布裁判端消息！！");
      refree_pub_->publish(message);
      reached_pub_->publish(is_reached);
    }

    // 参数动态调整回调
    rcl_interfaces::msg::SetParametersResult parameters_callback(
      const std::vector<rclcpp::Parameter> &parameters)
    {
      rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        for (const auto &param : parameters) {
            RCLCPP_INFO(this->get_logger(), 
                "参数更新: %s => %s", 
                param.get_name().c_str(),
                param.value_to_string().c_str());
        }
        return result;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<serial_interfaces::msg::Refree>::SharedPtr refree_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  // 2.初始化 ROS2 客户端；
  rclcpp::init(argc, argv);
  // 4.调用spin函数，并传入节点对象指针。
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  // 5.释放资源；
  rclcpp::shutdown();
  return 0;
}