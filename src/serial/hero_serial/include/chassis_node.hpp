// 该节点负责向底盘发送控制指令，接受来自底盘发送的裁判系统信息
#pragma once
#include <drv_cmd_com.h>
#include <visual_data.h>

#include <atomic>
#include <bitset>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <string>
#include <thread>

#include "boost_asio.hpp"
#include "cmd_protocol_tool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "refree_protocol_tool.hpp"
#include "serial_interfaces/msg/refree.hpp"
#include "std_msgs/msg/bool.hpp"
// #include "serial_interfaces/msg/robot.hpp"

// 机器人姿态状态枚举
enum class RobotPosture : uint8_t {
    ATTACK = 1,   // 进攻姿态
    DEFENSE = 2,  // 防御姿态
    MOVE = 3      // 移动姿态
};

class ChassisNode : public rclcpp::Node {
   public:
    ChassisNode();
    ~ChassisNode() = default;

    void VelOdometry_Callback(void* data);

   private:
    BoostAsio boost_asio_;
    // 修改：为每个定时器使用独立的、有意义的变量名
    rclcpp::TimerBase::SharedPtr chassis_cmd_timer_;
    rclcpp::TimerBase::SharedPtr referee_read_timer_;
    rclcpp::TimerBase::SharedPtr referee_pub_timer_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        vel_subscription_;
    rclcpp::Publisher<serial_interfaces::msg::Refree>::SharedPtr refree_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reach_subscription_;
    // 触发式消息
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr trigger_pub_;
    // 血量危险状态订阅者
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        hp_danger_subscription_;
    // 自瞄状态订阅者 (isAiming)
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr aiming_subscription_;
    // 轮式里程计速度发布器
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_vel_pub_;

    // 修改：为高频读写创建专用的回调组
    rclcpp::CallbackGroup::SharedPtr read_cb_group_;
    rclcpp::CallbackGroup::SharedPtr write_cb_group_;

    cmd_protocol::ProtocolBuilder cmd_builder;
    refree_protocol::ProtocolParser referee_parser;

    // 机器人姿态状态机
    RobotPosture current_posture_ = RobotPosture::MOVE;  // 默认移动姿态

    // serial_interfaces::msg::Robot robot_reach_;
    bool has_arrived_ = true;
    // 该部分是为了生成正弦小陀螺
    rclcpp::Time start_time_;      // 添加时间记录
    double amplitude_ = 800;       // 振幅 (度)
    double frequency_ = 1;         // 频率 (Hz)
                                   // 用于保存数据的相关变量
    int save_counter_ = 0;         // 计数器，累加至阈值时保存
    int save_interval_ms_ = 5000;  // 保存间隔(毫秒)，默认5秒
    std::string save_path_ = "./referee_data/";  // 保存数据的文件夹路径

    // 修改：移除 spin_speed_ 和其互斥锁
    std::atomic<bool>
        is_danger_mode_;  // 是否处于危险模式（使用原子变量避免竞争）

    // 随机数生成器
    std::mt19937 random_engine_;
    std::uniform_int_distribution<int> spin_speed_distribution_;

    // 串口数据包相关
    char tx_buffer[1024];
    LinuxDevice* device = new LinuxDevice();
    rt_device_t serial_dev_;
    Ctrl_info_t ctrl_info_t_data = {0, 0, 1200, 1, 0};
    sentry_cmd_info_t sentry_cmd_info_t_data = {1, 0, 0, 0, 0, 3, 0};

    Game_info_t game_info_t_data = {0, 0, 0};
    Event_data_t event_data_t_data = {0};
    Robot_info_t robot_info_t_data = {0, 0, 0, 0, 0, 0, 0};
    Robot_position_t robot_position_t_data = {0, 0, 0};
    Bullet_remaining_t bullet_remaining_t_data = {0, 0, 0, 0};
    Ground_robot_position_t ground_robot_position_t_data = {0, 0, 0, 0,
                                                            0, 0, 0, 0};
    Sentry_info_t sentry_info_t_data = {0, 0};
    Game_robot_survivors_t game_robot_survivors = {400, 400, 400,  400,
                                                   400, 400, 1500, 5000};
    Robot_command_map_t robot_command_map_data = {0, 0, 0, 0, 0};
    Robot_command_map_t last_robot_command_map_data = {0, 0, 0, 0, 0};
    Robot_wheel_odometry_t robot_wheel_odometry_data = {0.0, 0.0};
    geometry_msgs::msg::Point trigger_position;
    geometry_msgs::msg::Point last_trigger_position;

    // 暂存线速度
    float vel_x_mms;
    float vel_y_mms;

    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void HpDangerCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void AimingCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void ChassisCmdTimerCallback();
    void RefreeTimerCallback();
    void RefreePubTimerCallback();

    void ReachCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void PrintRefereeData(const auto& referee_parser);

    // 姿态切换辅助函数
    void SetPosture(RobotPosture posture);
};