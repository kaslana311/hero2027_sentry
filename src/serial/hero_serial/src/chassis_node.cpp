#include "chassis_node.hpp"

// 测试用例回调函数
void Callback_func_data1(void* data) {
    // std::cout << "Callback_func_data1 called with data" << std::endl;
}

void Callback_func_data2(void* data) {
    // std::cout << "Callback_func_data1 called with data" << std::endl;
    // RCLCPP_INFO(rclcpp::get_logger("Robot"),
    //             "Callback_func_data1 called with data");
}

// extern ComReceiveCtrl_t receiveCtrl[10];   //
// 接收控制结构体（与接收命令对应）

// void Callback_func_data3(void* data)
// {
//   std::cout << receiveCtrl[4].frequency << std::endl;
// }

// 单位转换常量
constexpr double MPS_TO_MMPS = 1000.0;        // 米/秒 -> 毫米/秒
constexpr double RAD_TO_0_1DEG = 572.957795;  // 弧度 -> 0.1度

ChassisNode ::ChassisNode() : Node("chassis_node") {
    // 初始化串口
    if (!boost_asio_.Init("/dev/chassis", 115200)) {
        RCLCPP_ERROR(this->get_logger(), "底盘串口初始化失败");
        rclcpp::shutdown();  // 结束程序
        return;
    }
    auto native_handle = boost_asio_.serial_port.lowest_layer().native_handle();
    serial_dev_ = (rt_device_t)(intptr_t)native_handle; 

    RCLCPP_INFO(this->get_logger(), "底盘串口节点初始化成功，文件描述符: %d",
                static_cast<int>(native_handle));
    RCLCPP_INFO(this->get_logger(), "底盘串口节点初始化成功");

    // 创建回调组
    refree_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    chassis_cmd_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions refree_sub_options;
    refree_sub_options.callback_group = refree_cb_group_;
    rclcpp::PublisherOptions refree_pub_options;
    refree_pub_options.callback_group = refree_cb_group_;
    rclcpp::SubscriptionOptions chassis_cmd_sub_options;
    chassis_cmd_sub_options.callback_group = chassis_cmd_cb_group_;

    // 创建发送给底盘控制命令的定时器
    chassis_cmd_timer = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ChassisNode::ChassisCmdTimerCallback, this),
        chassis_cmd_cb_group_);

    // 发布裁判系统信息给决策节点
    refree_pub_ = this->create_publisher<serial_interfaces::msg::Refree>(
        "refree_msg", 1, refree_pub_options);
    // 触发式消息发送
    trigger_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
        "trigger_msg", 1, refree_pub_options);
    // 轮式里程计速度发布器（用于 EKF 融合）
    wheel_odom_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/wheel_odom_velocity", 10, refree_pub_options);

    // 创建来自底盘裁判系统的定时器
    referee_timer = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ChassisNode::RefreeTimerCallback, this), refree_cb_group_);

    reach_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "arrived_at_goal", 10,
        std::bind(&ChassisNode::ReachCallback, this, std::placeholders::_1),
        chassis_cmd_sub_options);

    // 订阅血量危险状态消息
    hp_danger_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "hp_danger_status", 1,
        std::bind(&ChassisNode::HpDangerCallback, this, std::placeholders::_1));

    start_time_ = this->now();
    // 创建订阅者
    vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "final_cmd_vel", 1,
        std::bind(&ChassisNode::CmdVelCallback, this, std::placeholders::_1),
        chassis_cmd_sub_options);

    // 初始化随机数生成器
    random_engine_ = std::mt19937(std::random_device()());
    spin_speed_distribution_ = std::uniform_int_distribution<int>(3600, 4800);

    // 初始化转速参数
    spin_speed_ = 1200;  // 默认转速
    is_danger_mode_ = false;

    // 注册串口
    Attach_Receive_callback(CMD_ID_ROBOT_INFO, sizeof(Robot_info_t),
                            (void*)&robot_info_t_data, Callback_func_data1);
    Attach_Receive_callback(CMD_ID_GAME_INFO, sizeof(Game_info_t),
                            (void*)&game_info_t_data, Callback_func_data1);
    Attach_Receive_callback(CMD_ID_Bullet_remaining, sizeof(Bullet_remaining_t),
                            (void*)&bullet_remaining_t_data,
                            Callback_func_data1);
    Attach_Receive_callback(CMD_ID_GAME_ROBOT_survivors,
                            sizeof(Game_robot_survivors_t),
                            (void*)&game_robot_survivors, Callback_func_data1);
    Attach_Receive_callback(
        CMD_ID_Robot_command_map, sizeof(Robot_command_map_t),
        (void*)&robot_command_map_data, Callback_func_data2);

    Attach_SendCmd(serial_dev_,               // 设备句柄
                   CMD_ID_VISUAL_CTRL,        // 命令ID
                   TRANS_FOREVER,             // 传输类型
                   1,                         // 几周期发送一次
                   sizeof(ctrl_info_t_data),  // 数据长度
                   &ctrl_info_t_data          // 数据指针
    );

    Attach_SendCmd(serial_dev_,                     // 设备句柄
                   CMD_ID_SENTRY_CMD_INFO,          // 命令ID
                   TRANS_FOREVER,                   // 传输类型
                   1,                               // 几周期发送一次
                   sizeof(sentry_cmd_info_t_data),  // 数据长度
                   &sentry_cmd_info_t_data          // 数据指针
    );
}

// 血量危险状态回调函数
void ChassisNode::HpDangerCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data &&
        !is_danger_mode_
             .load())  // 原子读操作，如果已有线程存在读取设置好的ture，非true即flase
    {
        RCLCPP_WARN(this->get_logger(),
                    "收到血量危险信号，启动底盘高速随机转动模式！");

        // 设置危险模式标志
        is_danger_mode_.store(true);  // 原子写

        // 创建独立线程处理危险模式
        std::thread danger_thread([this]() {
            auto start_time = std::chrono::steady_clock::now();
            int time_elapsed = 0;

            // 立即生成第一个随机转速
            {
                std::lock_guard<std::mutex> lock(spin_speed_mutex_);
                spin_speed_ = spin_speed_distribution_(random_engine_);
                RCLCPP_INFO(this->get_logger(), "底盘转速随机调整为：%d",
                            spin_speed_);
            }

            // 运行6秒，每2秒更新一次转速
            while (time_elapsed < 6000) {  // 6000毫秒 = 6秒
                // 睡眠2秒
                std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                time_elapsed += 1500;

                // 生成新的随机转速
                {
                    std::lock_guard<std::mutex> lock(spin_speed_mutex_);
                    spin_speed_ = spin_speed_distribution_(random_engine_);
                    RCLCPP_INFO(this->get_logger(), "底盘转速随机调整为：%d",
                                spin_speed_);
                }
            }

            // 恢复默认转速
            {
                std::lock_guard<std::mutex> lock(spin_speed_mutex_);
                spin_speed_ = 3600;
                RCLCPP_INFO(this->get_logger(), "危险模式结束，恢复默认转速");
            }

            // 重置危险模式标志
            is_danger_mode_.store(false);
        });

        // 分离线程，让它在后台运行
        danger_thread.detach();
    }
}

void ChassisNode::ReachCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    has_arrived_ = msg->data;
}

void ChassisNode::CmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
    vel_x_mms = msg->linear.x * MPS_TO_MMPS;
    vel_y_mms = msg->linear.y * MPS_TO_MMPS;
}

size_t ChassisNode::FindFrameHeader(const std::vector<uint8_t>& data) {
    if (data.size() < 4) return -1;
    for (size_t i = 0; i <= data.size() - 4; ++i) {
        if (data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x80 &&
            data[i + 3] == 0x7F) {
            return i;
        }
    }
    return -1;
}

void ChassisNode::ChassisCmdTimerCallback() {
    // 获取当前转速参数（线程安全）
    int current_spin_speed;
    {
        std::lock_guard<std::mutex> lock(spin_speed_mutex_);
        current_spin_speed = spin_speed_;
    }
    // 如果到了
    if (has_arrived_) {
        // RCLCPP_INFO(this->get_logger(), "到了，小陀螺速度：%d",
        // current_spin_speed);
        ctrl_info_t_data.is_tyro = 1;
    } else {
        // RCLCPP_INFO(this->get_logger(), "没到，小陀螺速度：%d",
        // current_spin_speed);
        if (vel_x_mms <= 0.05 && vel_y_mms <= 0.05) {
            ctrl_info_t_data.is_tyro = 1;
        } else {
            ctrl_info_t_data.is_tyro = 0;
        }
    }

    // 更新cmd_builder的转速参数
    ctrl_info_t_data.x = vel_x_mms;           // 更新线速度x
    ctrl_info_t_data.y = vel_y_mms;           // 更新线速度y
    ctrl_info_t_data.w = current_spin_speed;  // 更新转速
    Send_Cmd_Packet(serial_dev_, tx_buffer, sizeof(tx_buffer), 0);
    RCLCPP_INFO(this->get_logger(), "发送回调函数");
}

void ChassisNode::RefreeTimerCallback() {
    if (boost_asio_.Read()) {
        const auto& data = boost_asio_.GetReadBuffer();
        if (!Receive_Handle(const_cast<unsigned char*>(data.data()),
                            static_cast<unsigned int>(data.size()))) {
            RCLCPP_ERROR(this->get_logger(), "裁判系统数据解析失败,%d",
                         data.size());
            // return;
        }
        RCLCPP_INFO(this->get_logger(), "数据正常解析,%d", data.size());
        serial_interfaces::msg::Refree msg;
        std::cout << "比赛类型: "
                  << static_cast<int>(game_info_t_data.game_type) << std::endl;
        std::cout << "比赛阶段: "
                  << static_cast<int>(game_info_t_data.game_progress)
                  << std::endl;
        std::cout << "阶段剩余时间: " << game_info_t_data.stage_remain_time
                  << " 秒" << std::endl;
        std::cout << "机器人ID: "
                  << static_cast<int>(robot_info_t_data.robot_id) << std::endl;
        std::cout << "当前血量: " << robot_info_t_data.remain_HP << std::endl;
        std::cout << "17mm弹药剩余: "
                  << bullet_remaining_t_data.bullet_remaining_num_17mm
                  << std::endl;
        std::cout << "堡垒上允许发弹："
                  << bullet_remaining_t_data.projectile_allowance_fortress
                  << std::endl;
        std::cout << "红方哨兵：" << game_robot_survivors.red_7_robot_HP
                  << std::endl;
        std::cout << "蓝方哨兵：" << game_robot_survivors.blue_7_robot_HP
                  << std::endl;
        std::cout << "红方前哨：" << game_robot_survivors.red_outpost_HP
                  << std::endl;
        std::cout << "蓝方前哨：" << game_robot_survivors.blue_outpost_HP
                  << std::endl;
        std::cout << "报点x:" << robot_command_map_data.target_position_x
                  << std::endl;
        std::cout << "报点y:" << robot_command_map_data.target_position_y
                  << std::endl;

        // 发布轮式里程计速度（用于 EKF 融合）
        // target_position_x 和 target_position_y 现在代表 v_x 和 v_y (单位: m/s)
        geometry_msgs::msg::TwistStamped wheel_vel_msg;
        wheel_vel_msg.header.stamp = this->now();
        wheel_vel_msg.header.frame_id = "base_link";
        
        // 将底盘发来的速度信息填入消息
        wheel_vel_msg.twist.linear.x = robot_command_map_data.target_position_x;
        wheel_vel_msg.twist.linear.y = robot_command_map_data.target_position_y;
        wheel_vel_msg.twist.linear.z = 0.0;
        
        // 角速度暂时设为0（如果底盘有角速度数据可以添加）
        wheel_vel_msg.twist.angular.x = 0.0;
        wheel_vel_msg.twist.angular.y = 0.0;
        wheel_vel_msg.twist.angular.z = 0.0;
        
        wheel_odom_vel_pub_->publish(wheel_vel_msg);
        
        RCLCPP_INFO(this->get_logger(), 
                    "发布轮式速度: vx=%.3f, vy=%.3f", 
                    wheel_vel_msg.twist.linear.x,
                    wheel_vel_msg.twist.linear.y);

        msg.competition_type = game_info_t_data.game_type;
        msg.competition_stage = game_info_t_data.game_progress;
        msg.stage_time = game_info_t_data.stage_remain_time;

        msg.robot_id = robot_info_t_data.robot_id;
        msg.current_hp = robot_info_t_data.remain_HP;

        msg.bullet_17mm = bullet_remaining_t_data.bullet_remaining_num_17mm;
        msg.projectile_allowance_fortress =
            bullet_remaining_t_data.projectile_allowance_fortress;

        msg.red_sentry = game_robot_survivors.red_7_robot_HP;
        msg.blue_sentry = game_robot_survivors.blue_7_robot_HP;
        msg.red_outpost = game_robot_survivors.red_outpost_HP;
        msg.blue_outpost = game_robot_survivors.blue_outpost_HP;
        refree_pub_->publish(msg);

        if ((robot_command_map_data.target_position_x !=
                 last_robot_command_map_data.target_position_x ||
             robot_command_map_data.target_position_y !=
                 last_robot_command_map_data.target_position_y) &&
            (robot_command_map_data.target_position_x != 0.0 &&
             robot_command_map_data.target_position_y != 0.0)) {
            geometry_msgs::msg::Point point;
            point.x = robot_command_map_data.target_position_x;
            point.y = robot_command_map_data.target_position_y;
            point.z = 0.0;

            last_robot_command_map_data.target_position_x =
                robot_command_map_data.target_position_x;
            last_robot_command_map_data.target_position_y =
                robot_command_map_data.target_position_y;

            trigger_pub_->publish(point);

            std::cout << "收到触发消息！" << std::endl;
        }
        // 移除已处理数据
        boost_asio_.Consume(data.size());
    }
}

void ChassisNode::PrintRefereeData(const auto& referee_parser) {
    //---------------- 比赛状态 ----------------
    // RCLCPP_INFO(this->get_logger(), "stage_remaining_time:%d",
    // referee_parser.stage_remaining_time());

    std::cout << "\n===== 比赛状态 =====\n";
    std::cout << "competition_type: "
              << static_cast<int>(referee_parser.competition_type())
              << std::endl;
    std::cout << "competition_stage: "
              << static_cast<int>(referee_parser.competition_stage())
              << std::endl;
    std::cout << "stage_remaining_time: "
              << referee_parser.stage_remaining_time() << " 秒\n";

    // //---------------- 红方状态 ----------------
    // std::cout << "\n===== 红方状态 =====\n";
    // std::cout << "红方英雄血量: " << referee_parser.red_hero_hp() <<
    // std::endl; std::cout << "红方工程血量: " <<
    // referee_parser.red_engineer_hp()
    //           << std::endl;
    // std::cout << "红方步兵3血量: " << referee_parser.red_infantry3_hp()
    //           << std::endl;
    // std::cout << "红方步兵4血量: " << referee_parser.red_infantry4_hp()
    //           << std::endl;
    // std::cout << "红方哨兵血量: " << referee_parser.red_sentry_hp() <<
    // std::endl; std::cout << "红方前哨血量: " <<
    // referee_parser.red_outpost_hp() << std::endl; std::cout << "红方基地血量:
    // " << referee_parser.red_base_hp() << std::endl;

    // //---------------- 蓝方状态 ----------------
    // std::cout << "\n===== 蓝方状态 =====\n";
    // std::cout << "蓝方英雄血量: " << referee_parser.blue_hero_hp() <<
    // std::endl; std::cout << "蓝方工程血量: " <<
    // referee_parser.blue_engineer_hp()
    //           << std::endl;
    // std::cout << "蓝方步兵3血量: " << referee_parser.blue_infantry3_hp()
    //           << std::endl;
    // std::cout << "蓝方步兵4血量: " << referee_parser.blue_infantry4_hp()
    //           << std::endl;
    // std::cout << "蓝方哨兵血量: " << referee_parser.blue_sentry_hp() <<
    // std::endl; std::cout << "蓝方前哨血量: " <<
    // referee_parser.blue_outpost_hp()
    //           << std::endl;
    // std::cout << "蓝方基地血量: " << referee_parser.blue_base_hp() <<
    // std::endl;

    // //---------------- 战场状态 ----------------
    // std::cout << "\n===== 战场状态 =====\n";
    // std::cout << std::boolalpha;  // 布尔值显示为 true/false
    // std::cout << "补给区非重叠状态: " << referee_parser.supply_non_overlap()
    //           << std::endl;
    // std::cout << "补给区重叠状态: " << referee_parser.supply_overlap()
    //           << std::endl;
    // std::cout << "补给区占领状态: " << referee_parser.supply_occupied()
    //           << std::endl;
    // std::cout << "能量机关激活进度: "
    //           << static_cast<int>(referee_parser.energy_activation()) <<
    //           "%\n";
    // std::cout << "高地状态: "
    //           << static_cast<int>(referee_parser.highland_status()) <<
    //           std::endl;
    // std::cout << "梯形区域状态: "
    //           << static_cast<int>(referee_parser.trapezoid_status()) <<
    //           std::endl;
    // std::cout << "飞镖命中时间: " << referee_parser.dart_hit_time() << "
    // 秒\n"; std::cout << "飞镖目标: " <<
    // static_cast<int>(referee_parser.dart_target())
    //           << std::endl;
    // std::cout << "中央控制权: "
    //           << static_cast<int>(referee_parser.center_control()) <<
    //           std::endl;

    // //---------------- 本机状态 ----------------
    // std::cout << "\n===== 本机状态 =====\n";
    // std::cout << "机器人ID: " << static_cast<int>(referee_parser.robot_id())
    //           << std::endl;
    // std::cout << "机器人等级: " <<
    // static_cast<int>(referee_parser.robot_level())
    //           << std::endl;
    std::cout << "当前血量: " << referee_parser.current_hp() << std::endl;
    // std::cout << "最大血量: " << referee_parser.max_hp() << std::endl;
    // std::cout << "冷却速率: " << referee_parser.cooling_rate() << "
    // units/s\n"; std::cout << "热量上限: " << referee_parser.heat_limit() <<
    // std::endl; std::cout << "底盘热量上限: " <<
    // referee_parser.chassis_heat_limit()
    //           << std::endl;

    // //---------------- 位置信息 ----------------
    // std::cout << "\n===== 位置信息 =====\n";
    // std::cout << std::fixed << std::setprecision(2);
    // std::cout << "X坐标: " << referee_parser.position_x() << " 米\n";
    // std::cout << "Y坐标: " << referee_parser.position_y() << " 米\n";
    // std::cout << "朝向: " << referee_parser.orientation() << " 弧度\n";

    // //---------------- 弹药状态 ----------------
    // std::cout << "\n===== 弹药状态 =====\n";
    // std::cout << "17mm弹丸余量: " << referee_parser.bullet_17mm() << " 发\n";
    // std::cout << "42mm弹丸余量: " << referee_parser.bullet_42mm() << " 发\n";
    // std::cout << "剩余金币: " << referee_parser.remaining_gold() <<
    // std::endl;

    // //---------------- 队友位置 ----------------
    // std::cout << "\n===== 队友位置 =====\n";
    // auto print_pair = [](const std::string& name, const auto& pos) {
    //   std::cout << name << ": (" << pos.first << ", " << pos.second << ")\n";
    // };
    // print_pair("英雄位置", referee_parser.hero_position());
    // print_pair("工程位置", referee_parser.engineer_position());
    // print_pair("步兵3位置", referee_parser.infantry3_position());
    // print_pair("步兵4位置", referee_parser.infantry4_position());

    // //---------------- 哨兵状态 ----------------
    // std::cout << "\n===== 哨兵状态 =====\n";
    // std::cout << "哨兵子弹余量: " << referee_parser.sentry_bullet_allowance()
    //           << " 发\n";
    // std::cout << "远程补给子弹次数: "
    //           <<
    //           static_cast<int>(referee_parser.sentry_remote_bullet_count())
    //           << std::endl;
    // std::cout << "远程补给血量次数: "
    //           << static_cast<int>(referee_parser.sentry_remote_hp_count())
    //           << std::endl;
    // std::cout << "确认复活状态: " << referee_parser.sentry_confirm_revive()
    //           << std::endl;
    // std::cout << "立即复活状态: " << referee_parser.sentry_immediate_revive()
    //           << std::endl;
    // std::cout << "复活消耗: " << referee_parser.sentry_revive_cost() << "
    // 金币\n";

    // //---------------- 附加状态 ----------------
    // std::cout << "\n===== 附加状态 =====\n";
    // std::cout << "附加状态位0: " << referee_parser.additional_status_bit0()
    //           << std::endl;
    // std::cout << "附加状态位1-11: "
    //           <<
    //           static_cast<int>(referee_parser.additional_status_bits1_11())
    //           << std::dec << std::endl;
    // std::cout << "附加状态位12-15: "
    //           <<
    //           static_cast<int>(referee_parser.additional_status_bits12_15())
    //           << std::endl;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChassisNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
