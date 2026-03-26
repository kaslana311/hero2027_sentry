#pragma once
#include <atomic>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <thread>

#include "boost_asio.hpp"
#include "drv_cmd_com.h"
#include "visual_data.h"

class GimbalNode : public rclcpp::Node {
   public:
    GimbalNode();
    ~GimbalNode();

    void IsAimingDataCallback(void* data);

   private:
    BoostAsio boost_asio_;

    // ========== 接收线程相关 ==========
    std::thread receive_thread_;               // 接收线程
    std::atomic<bool> thread_running_{false};  // 线程运行标志
    void ReceiveThreadFunc();                  // 接收线程函数

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_aiming_pub_;

    LinuxDevice* device;
    rt_device_t serial_dev_;

    CV_to_Nav_Massage_t cv_to_nav_massage_data = {false};
};
