// 该节点负责向底盘发送控制指令，接受来自底盘发送的裁判系统信息
#pragma once
#include <drv_CRC.h>
#include <drv_cmd_com.h>
#include <sys/types.h>
#include <visual_data.h>

#include <atomic>
#include <bitset>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <string>
#include <thread>
#include <unordered_map>

#include "boost_asio.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "interfaces/msg/global_target_array.hpp"
#include "interfaces/msg/tracker_output.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_interfaces/msg/refree.hpp"
#include "std_msgs/msg/bool.hpp"

// 机器人姿态状态枚举
enum class RobotPosture : uint8_t {
    ATTACK = 1,   // 进攻姿态
    DEFENSE = 2,  // 防御姿态
    MOVE = 3      // 移动姿态
};

// ==================== 通用优先级仲裁框架 ====================
// 多源命令仲裁：多个影响源通过统一接口注册期望输出，
// 仲裁器按优先级选择激活中的最高优先级源输出。
// 模板参数 T: 仲裁的输出类型 (float, uint8_t, ...)

template <typename T>
class PriorityArbiterBase {
   public:
    virtual void update(int source_id, int priority, T value, bool active) = 0;
    virtual T evaluate() const = 0;
    virtual void setDefault(int priority, T value) = 0;
    virtual ~PriorityArbiterBase() = default;
};

template <typename T>
class PriorityArbiter : public PriorityArbiterBase<T> {
   public:
    void update(int source_id, int priority, T value, bool active) override {
        std::lock_guard<std::mutex> lock(mutex_);
        commands_[source_id] = {priority, value, active};
    }

    T evaluate() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        int best_priority = std::numeric_limits<int>::max();
        T best_value = default_.value;
        for (const auto& [id, cmd] : commands_) {
            if (cmd.active && cmd.priority < best_priority) {
                best_priority = cmd.priority;
                best_value = cmd.value;
            }
        }
        return best_value;
    }

    void setDefault(int priority, T value) override {
        std::lock_guard<std::mutex> lock(mutex_);
        default_ = {priority, value, true};
    }

   private:
    struct Command {
        int priority;
        T value;
        bool active;
    };
    std::unordered_map<int, Command> commands_;
    Command default_{std::numeric_limits<int>::max(), T{}, true};
    mutable std::mutex mutex_;
};

class ChassisNode : public rclcpp::Node {
   public:
    ChassisNode();
    ~ChassisNode();  // 需要自定义析构函数来停止线程

    void VelOdometry_Callback(void* data);
    void OpponentPosition_Callback(void* data);
    void Trigger_Callback(void* data);

   private:
    BoostAsio boost_asio_;

    // ========== 接收线程相关 ==========
    std::thread receive_thread_;               // 接收线程
    std::atomic<bool> thread_running_{false};  // 线程运行标志
    void ReceiveThreadFunc();                  // 接收线程函数

    // 修改：为每个定时器使用独立的、有意义的变量名
    rclcpp::TimerBase::SharedPtr chassis_cmd_timer_;
    // referee_read_timer_ 已移除，改用独立线程
    rclcpp::TimerBase::SharedPtr referee_pub_timer_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        vel_subscription_;
    rclcpp::Publisher<serial_interfaces::msg::Refree>::SharedPtr refree_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reach_subscription_;
    // 触发式消息
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr trigger_pub_;
    // 血量危险状态订阅者
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        hp_danger_subscription_;
    // pitch和roll角度订阅者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        pitch_roll_angle_subscription_;
    // 能量机关激活命令订阅者
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        rune_activate_subscription_;
    // 自瞄状态订阅者 (TrackerOutput)
    rclcpp::Subscription<interfaces::msg::TrackerOutput>::SharedPtr
        aiming_subscription_;
    // 轮式里程计速度发布器
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_vel_pub_;

    // 雷达(全局敌方)位置发布器
    rclcpp::Publisher<interfaces::msg::GlobalTargetArray>::SharedPtr radar_pub_;

    // 修改：为高频读写创建专用的回调组
    rclcpp::CallbackGroup::SharedPtr read_cb_group_;
    rclcpp::CallbackGroup::SharedPtr write_cb_group_;
    rclcpp::CallbackGroup::SharedPtr refree_cb_group_;

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
    std::atomic<bool> is_guarding{true};
    uint8_t last_game_stage = 0;

    // ==================== 优先级仲裁器 ====================
    // 小陀螺角速度多源仲裁：各影响源通过统一接口注册，仲裁器按优先级输出
    PriorityArbiter<float> spin_speed_arbiter_;

    // 各影响源的 source_id，用于向仲裁器注册
    static constexpr int SRC_GAME_STAGE = 0;
    static constexpr int SRC_RUNE_ATTACKING = 1;
    static constexpr int SRC_POSTURE_TILT = 2;
    static constexpr int SRC_DANGER_MODE = 3;
    static constexpr int SRC_DEFAULT = 4;

    // 对应优先级值 (越小优先级越高)
    static constexpr int PRI_GAME_STAGE = 0;
    static constexpr int PRI_RUNE_ATTACKING = 1;
    static constexpr int PRI_POSTURE_TILT = 2;
    static constexpr int PRI_DANGER_MODE = 3;
    static constexpr int PRI_DEFAULT = 4;

    // 危险模式线程防重入标志（仅由 HpDangerCallback 单线程访问，非 atomic）
    bool danger_mode_active_{false};

    // 倾斜滞回比较状态：记录当前是否处于倾斜关闭小陀螺的状态
    bool is_tilted_state_{false};

    // 里程计帧计数器：每10帧取1帧做z轴倾斜计算，避免高频抖动
    int odometry_frame_counter_{0};

    // 正常战斗期望转速（由比赛阶段等逻辑修改，作为 DEFAULT 源的基准值）
    float target_w_{1200.0f};

    // 随机数生成器
    std::mt19937 random_engine_;
    std::uniform_int_distribution<int> spin_speed_distribution_;

    // 串口数据包相关
    char tx_buffer[1024];
    LinuxDevice* device = new LinuxDevice();
    rt_device_t serial_dev_;
    Ctrl_info_t ctrl_info_t_data = {0, 0, 1200, 1, 0};
    sentry_cmd_info_t sentry_cmd_info_t_data = {0, 0, 0, 0, 0, 3, 0, 0};

    Game_info_t game_info_t_data = {0, 0, 0, 0, 0, 0};
    Event_data_t event_data_t_data = {0};
    Robot_info_t robot_info_t_data = {0, 0, 0, 0, 0, 0, 0};
    Robot_position_t robot_position_t_data = {0, 0, 0};
    Bullet_remaining_t bullet_remaining_t_data = {0, 0, 0, 0};
    Ground_robot_position_t ground_robot_position_t_data = {0, 0, 0, 0,
                                                            0, 0, 0, 0};
    // Sentry_info_t sentry_info_t_data = {0, 0};
    ext_sentry_info_t ext_sentry_info_data = {0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 1, 0};
    game_robot_HP_t game_robot_HP_data = {400, 400, 400,  400,
                                          0,   400, 1500, 5000};
    Robot_command_map_t robot_command_map_data = {0, 0, 0, 0, 0};
    Robot_command_map_t last_robot_command_map_data = {0, 0, 0, 0, 0};
    Robot_wheel_odometry_t robot_wheel_odometry_data = {0.0, 0.0};
    com_opponent_position_t com_opponent_position_data = {0, 0, 0, 0, 0, 0,
                                                          0, 0, 0, 0, 0, 0};
    geometry_msgs::msg::Point trigger_position;
    geometry_msgs::msg::Point last_trigger_position;

    // 暂存线速度
    float vel_x_mms;
    float vel_y_mms;

    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void HpDangerCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void RuneActivateCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void AimingCallback(const interfaces::msg::TrackerOutput::SharedPtr msg);
    void ChassisCmdTimerCallback();
    // RefreeTimerCallback 已被 ReceiveThreadFunc 替代
    void RefreePubTimerCallback();
    void PitchRollAngleCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void ReachCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void PrintRefereeData(const auto& referee_parser);

    // 姿态切换辅助函数
    void SetPosture(RobotPosture posture);
};