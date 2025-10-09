#include "rclcpp/rclcpp.hpp"
#include "serial_interfaces/msg/radar.hpp"

using namespace std::chrono_literals;

class RadarPublisher : public rclcpp::Node {
public:
    RadarPublisher() : Node("radar_publisher") {
        // 创建雷达消息发布器
        radar_pub_ = this->create_publisher<serial_interfaces::msg::Radar>("radar_msg", 1);
        
        // 创建定时器（500ms发布周期）
        timer_ = this->create_wall_timer(500ms, std::bind(&RadarPublisher::timer_callback, this));
        
        // 添加参数更新回调
        param_callback_handle_ = add_on_set_parameters_callback(
            std::bind(&RadarPublisher::parameters_callback, this, std::placeholders::_1));
        
        // 声明参数
        this->declare_parameter("robot_1_x", 5.0);
        this->declare_parameter("robot_1_y", 0.0);
    }

private:
    void timer_callback() {
        auto message = serial_interfaces::msg::Radar();
        
        // 设置robot_1_position坐标
        message.robot_1_position.x = this->get_parameter("robot_1_x").as_double();
        message.robot_1_position.y = this->get_parameter("robot_1_y").as_double();
        message.robot_1_position.z = 0.0;  // 二维坐标z设为0
        
        radar_pub_->publish(message);
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
    rclcpp::Publisher<serial_interfaces::msg::Radar>::SharedPtr radar_pub_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarPublisher>());
    rclcpp::shutdown();
    return 0;
}