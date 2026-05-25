#include "chassis_node.hpp"

// 单位转换常量
constexpr double MPS_TO_MMPS = 1000.0;        // 米/秒 -> 毫米/秒
constexpr double RAD_TO_0_1DEG = 572.957795;  // 弧度 -> 0.1度
// 全局指针，用于在静态回调中访问 ChassisNode 实例
static ChassisNode* g_chassis_node_instance = nullptr;

// 测试用例回调函数
void Callback_func_data1(void* data) {
    // std::cout << "Callback_func_data1 called with data" << std::endl;
}

// 全局回调函数，用于轮式里程计数据
void VelOdometry_Global_Callback(void* data) {
    if (g_chassis_node_instance) {
        g_chassis_node_instance->VelOdometry_Callback(data);
    }
}

// 全局回调函数，用于敌方机器人位置数据
void OpponentPosition_Global_Callback(void* data) {
    if (g_chassis_node_instance) {
        g_chassis_node_instance->OpponentPosition_Callback(data);
    }
}

// 全局回调函数，用于裁判系统触发式消息数据
void Trigger_Global_Callback(void* data) {
    if (g_chassis_node_instance) {
        g_chassis_node_instance->Trigger_Callback(data);
    }
}

void ChassisNode::VelOdometry_Callback(void* data) {
    // geometry_msgs::msg::Point point;
    // point.x = robot_wheel_odometry_data.vel_x;
    // point.y = robot_wheel_odometry_data.vel_y;
    // point.z = 0.0;

    // trigger_pub_->publish(point);
    // 发布轮式里程计速度（用于 EKF 融合）
    nav_msgs::msg::Odometry wheel_odom_msg;
    wheel_odom_msg.header.stamp = this->now();
    wheel_odom_msg.header.frame_id = "odom";
    wheel_odom_msg.child_frame_id = "base_link";

    // 填充 Twist 信息 (线速度和角速度)
    // 根据 robot_localization 要求，我们只需要填充 vx, vy
    wheel_odom_msg.twist.twist.linear.x =
        robot_wheel_odometry_data.vel_x / 1000.0;
    wheel_odom_msg.twist.twist.linear.y =
        robot_wheel_odometry_data.vel_y / 1000.0;

    // 如果没有实际测量的角速度，可以设为0
    wheel_odom_msg.twist.twist.linear.z = 0.0;
    wheel_odom_msg.twist.twist.angular.x = 0.0;
    wheel_odom_msg.twist.twist.angular.y = 0.0;
    wheel_odom_msg.twist.twist.angular.z = 0.0;

    // 填充协方差矩阵 (6x6)
    // 对应顺序：x, y, z, roll, pitch, yaw
    // 我们主要关注 linear.x (index 0) 和 linear.y (index 7) 对应的协方差
    // 假设速度测量有一定的噪声，例如方差为 0.01
    // 注意：Odometry message 中的 twist.covariance 是一个 36 元素的数组
    // 索引映射:
    // 0: x  7: y  14: z  21: roll  28: pitch  35: yaw

    // 初始化为全 0
    for (int i = 0; i < 36; i++) {
        wheel_odom_msg.twist.covariance[i] = 0.0;
    }

    // 设置对角线元素，表示方差
    // 较小的值表示较高的置信度
    wheel_odom_msg.twist.covariance[0] = 1e-3;   // vx variance
    wheel_odom_msg.twist.covariance[7] = 1e-3;   // vy variance
    wheel_odom_msg.twist.covariance[14] = 1e-6;  // vz variance (几乎为0)
    wheel_odom_msg.twist.covariance[21] = 1e-6;  // vroll variance
    wheel_odom_msg.twist.covariance[28] = 1e-6;  // vpitch variance
    wheel_odom_msg.twist.covariance[35] = 1e-3;  // vyaw variance

    // 注意：pose 信息这里不需要填充，因为我们只用它的速度
    // 但为了规范，pose 的协方差可以设为极其大，表示不可信
    for (int i = 0; i < 36; i++) {
        wheel_odom_msg.pose.covariance[i] = 1e9;  // 很大的值
    }

    wheel_odom_vel_pub_->publish(wheel_odom_msg);
}

void ChassisNode::OpponentPosition_Callback(void* data) {
    if (!data) return;
    com_opponent_position_t* pos_data =
        static_cast<com_opponent_position_t*>(data);

    interfaces::msg::GlobalTargetArray msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "global_red_map";

    // 辅助闭包函数，用于解析并压入数据（假设发送来的数据单位为 cm，需要除以 100
    // 转为 m） 如果实际单位是 mm，请将 100.0 改为 1000.0
    auto add_target = [&](uint16_t x, uint16_t y, const std::string& id_name) {
        if (x != 0 || y != 0) {  // 假设 0,0 意味着该机器人位置未知/无效
            interfaces::msg::GlobalTarget target;
            target.id = id_name;
            target.position.x = x / 100.0;  // 转换为米
            target.position.y = y / 100.0;  // 转换为米
            target.position.z = 0.0;
            msg.targets.push_back(target);
        }
    };

    add_target(pos_data->opponent_hero_position_x,
               pos_data->opponent_hero_position_y, "1");  // hero
    add_target(pos_data->opponent_engineer_position_x,
               pos_data->opponent_engineer_position_y, "2");  // engineer
    add_target(pos_data->opponent_infantry_3_position_x,
               pos_data->opponent_infantry_3_position_y, "3");  // infantry_3
    add_target(pos_data->opponent_infantry_4_position_x,
               pos_data->opponent_infantry_4_position_y, "4");  // infantry_4
    add_target(pos_data->opponent_sentry_position_x,
               pos_data->opponent_sentry_position_y, "SENTRY");  // sentry

    // 只有当有有效目标时才发布，避免发布空数组干扰决策
    if (!msg.targets.empty()) {
        radar_pub_->publish(msg);
    }
}

void ChassisNode::Trigger_Callback(void* data) {
    if (!data) return;
    Robot_command_map_t* cmd_data = static_cast<Robot_command_map_t*>(data);

    // 串口层仅负责转发，不做任何状态比对和判断
    geometry_msgs::msg::PointStamped point;
    point.header.stamp = this->now();
    point.header.frame_id = "global_red_map";
    point.point.x = cmd_data->target_position_x;
    point.point.y = cmd_data->target_position_y;
    point.point.z = 0.0;

    trigger_pub_->publish(point);
}

ChassisNode ::ChassisNode() : Node("chassis_node") {
    // 设置全局实例指针
    g_chassis_node_instance = this;

    // 初始化串口
    if (!boost_asio_.Init("/dev/chassis", 460800)) {
        RCLCPP_ERROR(this->get_logger(), "底盘串口初始化失败");
        rclcpp::shutdown();  // 结束程序
        return;
    }
    auto native_handle = boost_asio_.serial_port.lowest_layer().native_handle();
    serial_dev_ = (rt_device_t)(intptr_t)native_handle;

    RCLCPP_INFO(this->get_logger(), "底盘串口节点初始化成功，文件描述符: %d",
                static_cast<int>(native_handle));

    // 1. 为高频串口读取创建一个专用的回调组
    read_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    // 2. 为高频串口写入创建一个专用的回调组
    write_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    // 3. 其他低频的ROS订阅和发布将使用默认回调组，无需特殊设置
    refree_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // 创建发送给底盘控制命令的定时器 (100Hz)
    chassis_cmd_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ChassisNode::ChassisCmdTimerCallback, this),
        write_cb_group_);  // 使用写入专用组

    // 独立接收线程
    thread_running_ = true;
    receive_thread_ = std::thread(&ChassisNode::ReceiveThreadFunc, this);
    RCLCPP_INFO(this->get_logger(), "串口接收线程已启动");

    // 创建裁判系统广播定时器 (10Hz)
    referee_pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ChassisNode::RefreePubTimerCallback, this),
        refree_cb_group_);

    // 发布裁判系统信息给决策节点
    refree_pub_ = this->create_publisher<serial_interfaces::msg::Refree>(
        "refree_msg_back", 10);  // 使用默认组
    // 触发式消息发送
    trigger_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "trigger_msg", 10);  // 使用默认组
    // 轮式里程计速度发布器（用于 EKF 融合）
    // 注意：改用 Odometry 消息类型以包含协方差信息
    wheel_odom_vel_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odometry", 10);

    // 雷达(全局敌方)位置发布器
    radar_pub_ = this->create_publisher<interfaces::msg::GlobalTargetArray>(
        "/tracker/radar", rclcpp::SensorDataQoS());

    reach_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "arrived_at_goal", 10,
        std::bind(&ChassisNode::ReachCallback, this, std::placeholders::_1)
        // 不指定回调组，使用默认组
    );

    // 订阅血量危险状态消息
    hp_danger_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "hp_danger_status", 10,
        std::bind(&ChassisNode::HpDangerCallback, this, std::placeholders::_1)
        // 不指定回调组，使用默认组
    );

    // 订阅能量机关激活控制消息
    rune_activate_subscription_ =
        this->create_subscription<std_msgs::msg::Bool>(
            "RuneActivateCommand", 10,
            std::bind(&ChassisNode::RuneActivateCallback, this,
                      std::placeholders::_1));

    // 订阅自瞄状态消息 (TrackerOutput)
    const rclcpp::QoS qos(
        rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());
    aiming_subscription_ =
        this->create_subscription<interfaces::msg::TrackerOutput>(
            "/tracker/autoaim", qos,
            std::bind(&ChassisNode::AimingCallback, this, std::placeholders::_1)
            // 不指定回调组，使用默认组
        );

    start_time_ = this->now();
    // 创建订阅者
    vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_controller", 10,
        std::bind(&ChassisNode::CmdVelCallback, this, std::placeholders::_1)
        // 不指定回调组，使用默认组
    );

    // 初始化随机数生成器
    random_engine_ = std::mt19937(std::random_device()());
    spin_speed_distribution_ = std::uniform_int_distribution<int>(3600, 4800);

    // 初始化转速参数
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
                            sizeof(game_robot_HP_t), (void*)&game_robot_HP_data,
                            Callback_func_data1);
    Attach_Receive_callback(
        CMD_ID_Robot_wheel_odometry, sizeof(Robot_wheel_odometry_t),
        (void*)&robot_wheel_odometry_data, VelOdometry_Global_Callback);
    Attach_Receive_callback(
        CMD_ID_Robot_command_map, sizeof(Robot_command_map_t),
        (void*)&robot_command_map_data, Trigger_Global_Callback);
    Attach_Receive_callback(
        CMD_ID_com_opponent_position, sizeof(com_opponent_position_t),
        (void*)&com_opponent_position_data, OpponentPosition_Global_Callback);
    Attach_Receive_callback(
        CMD_ID_Ground_robot_position, sizeof(Ground_robot_position_t),
        (void*)&ground_robot_position_t_data, Callback_func_data1);
    Attach_Receive_callback(CMD_ID_Sentry_info, sizeof(ext_sentry_info_t),
                            (void*)&ext_sentry_info_data, Callback_func_data1);

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
    if (msg->data && !is_danger_mode_.load()) {
        RCLCPP_WARN(this->get_logger(),
                    "收到血量危险信号，启动底盘高速随机转动模式！");

        // 设置危险模式标志
        is_danger_mode_.store(true);

        // 创建独立线程处理危险模式
        std::thread danger_thread([this]() {
            // 立即生成第一个随机转速
            ctrl_info_t_data.w = spin_speed_distribution_(random_engine_);
            RCLCPP_INFO(this->get_logger(), "底盘转速随机调整为：%f",
                        ctrl_info_t_data.w);

            // 运行6秒，每1.5秒更新一次转速
            for (int i = 0; i < 4; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                ctrl_info_t_data.w = spin_speed_distribution_(random_engine_);
                RCLCPP_INFO(this->get_logger(), "底盘转速随机调整为：%f",
                            ctrl_info_t_data.w);
            }

            // 恢复战时常规转速
            ctrl_info_t_data.w = 2400;
            RCLCPP_INFO(this->get_logger(), "危险模式结束，恢复战时常规转速");

            // 重置危险模式标志
            is_danger_mode_.store(false);
        });

        // 分离线程，让它在后台运行
        danger_thread.detach();
    }
}

// 能量机关激活控制回调函数
void ChassisNode::RuneActivateCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        sentry_cmd_info_t_data.energy_mechanism_activate = 1;
    } else {
        sentry_cmd_info_t_data.energy_mechanism_activate = 0;
    }
    RCLCPP_INFO(this->get_logger(),
                "能量机关串口指令更新: energy_mechanism_activate = %d",
                sentry_cmd_info_t_data.energy_mechanism_activate);
}

void ChassisNode::ReachCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    // ctrl_info_t_data.is_tyro = msg->data;
    is_guarding = msg->data;

    // 只有在非进攻状态下才能切换移动/防御姿态
    if (current_posture_ != RobotPosture::ATTACK) {
        if (msg->data) {
            // 到达目标点，进入防御姿态
            SetPosture(RobotPosture::DEFENSE);
        } else {
            // 未到达目标点，进入移动姿态
            SetPosture(RobotPosture::MOVE);
        }
    }
}

void ChassisNode::CmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
    ctrl_info_t_data.x = msg->linear.x * MPS_TO_MMPS;
    ctrl_info_t_data.y = msg->linear.y * MPS_TO_MMPS;
}

// 自瞄状态回调函数 (基于 TrackerOutput)
void ChassisNode::AimingCallback(
    const interfaces::msg::TrackerOutput::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "收到消息");
    if (!msg->targets.empty()) {
        // targets 不为空，检验是否是 ATTACK 状态，不是则切入 ATTACK 状态
        if (current_posture_ != RobotPosture::ATTACK) {
            SetPosture(RobotPosture::ATTACK);
            RCLCPP_INFO(this->get_logger(), "检测到目标，进入进攻姿态");
        }
    } else {
        // targets 为空，检验是否是 ATTACK 状态，是才切换成 MOVE 状态
        if (current_posture_ == RobotPosture::ATTACK) {
            if (is_guarding == true) {
                SetPosture(RobotPosture::DEFENSE);
                RCLCPP_INFO(this->get_logger(),
                            "目标丢失，退出进攻姿态，进入防御姿态");
            } else {
                SetPosture(RobotPosture::MOVE);
                RCLCPP_INFO(this->get_logger(),
                            "目标丢失，退出进攻姿态，进入移动姿态");
            }
        }
    }
}

// 姿态切换辅助函数
void ChassisNode::SetPosture(RobotPosture posture) {
    if (current_posture_ != posture) {
        current_posture_ = posture;
        sentry_cmd_info_t_data.sentry_status =
            static_cast<uint8_t>(current_posture_);
        RCLCPP_DEBUG(this->get_logger(), "姿态切换至: %d",
                     static_cast<int>(current_posture_));
    }
}

// ========== 析构函数：安全停止接收线程 ==========
ChassisNode::~ChassisNode() {
    // 设置线程停止标志
    thread_running_ = false;

    // 等待线程结束
    if (receive_thread_.joinable()) {
        RCLCPP_INFO(this->get_logger(), "正在停止串口接收线程...");
        receive_thread_.join();
        RCLCPP_INFO(this->get_logger(), "串口接收线程已停止");
    }

    // 清理全局指针
    g_chassis_node_instance = nullptr;
}

// ========== 接收线程函数：高频串口读取 ==========
void ChassisNode::ReceiveThreadFunc() {
    RCLCPP_INFO(this->get_logger(), "接收线程开始运行");

    // 私有数据蓄水池，用于无损拼接不定长的粘包与半包
    std::vector<unsigned char> rx_buffer;

    while (rclcpp::ok() && thread_running_.load()) {
        if (boost_asio_.Read()) {
            const auto& data = boost_asio_.GetReadBuffer();

            if (!data.empty()) {
                // 1. 将串口底层读取到的残缺或超长数据全部灌入蓄水池
                rx_buffer.insert(rx_buffer.end(), data.begin(), data.end());
                // 2. 清空底层缓冲区
                boost_asio_.Consume(data.size());
            }
        }

        // 3. 不断循环剥离蓄水池中的完整数据包
        while (!rx_buffer.empty()) {
            // 步骤 A：匹配帧头标志位 SOF (0xAA)
            size_t sof_idx = 0;
            while (sof_idx < rx_buffer.size() &&
                   rx_buffer[sof_idx] != COM_HEADER_SOF) {
                sof_idx++;
            }

            // 丢弃匹配到有效标志位之前的乱码
            if (sof_idx > 0) {
                rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + sof_idx);
            }

            // 步骤 B：包头完整性检测 (包头定义为5个字节)
            if (rx_buffer.size() < sizeof(ComHeader_t)) {
                break;  // 这是典型的"半包"状态，退出当前循环等待下一次 Read
                        // 补齐
            }

            // 步骤 C：核心！包头 CRC8 预校验 —— 彻底消灭假帧头陷阱
            // 包头除了最后的 CRC8 本身，前 4 字节参与计算: SOF(1) +
            // DataLength(2) + Seq(1)
            unsigned char cal_crc8 =
                Get_CRC8_Check_Sum(rx_buffer.data(), 4, CRC8_INIT);
            if (cal_crc8 != rx_buffer[4]) {
                // ！！！ 这一步至关重要 ！！！
                // 哪怕第一个字节是0xAA，它也只是个藏在数据段里的"假李鬼"
                // 强行剔除这 1 个字节，逼迫蓄水池继续往后寻找真正的 0xAA 帧头
                rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + 1);
                continue;
            }

            // 步骤 D：提取极其安全可靠的有效数据长度
            uint16_t data_length =
                rx_buffer[1] | (static_cast<uint16_t>(rx_buffer[2]) << 8);

            // 步骤 E：根据底层协议精准计算一整包大小
            // 真实物理帧长 = 帧头(5) + 命令头CmdID(2) + 数据(data_length)
            // 没有 CRC16，总附加字节正好是 7
            size_t frame_total_len = 7 + data_length;

            if (rx_buffer.size() < frame_total_len) {
                break;  // 包头合法但数据区尚未全部抵达（也是半包），留着下个循环继续接
            }

            // 步骤 F：大功告成，将百分百合法且完整的内存块丢给学长的解析器
            Receive_Handle(rx_buffer.data(), frame_total_len);

            // 步骤 G：精准剥离刚才已经被解析消费掉的帧长，一毫米的字节都不多删
            rx_buffer.erase(rx_buffer.begin(),
                            rx_buffer.begin() + frame_total_len);
        }

        // 避免空跑吃满 CPU，线程轻微睡 1ms
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    RCLCPP_INFO(this->get_logger(), "接收线程退出");
}

void ChassisNode::ChassisCmdTimerCallback() {
    Send_Cmd_Packet(serial_dev_, tx_buffer, sizeof(tx_buffer), 0);
}

void ChassisNode::RefreePubTimerCallback() {
    serial_interfaces::msg::Refree msg;

    msg.competition_type = game_info_t_data.game_type;
    msg.competition_stage = game_info_t_data.game_progress;
    msg.stage_time = game_info_t_data.stage_remain_time;
    msg.dart_target = game_info_t_data.dart_target;
    msg.small_energy_state = game_info_t_data.small_energy_state;
    msg.big_energy_state = game_info_t_data.big_energy_state;

    msg.robot_id = robot_info_t_data.robot_id;
    msg.current_hp = robot_info_t_data.remain_HP;

    // 己方血量
    msg.friend_hero = game_robot_HP_data.ally_1_robot_HP;
    msg.friend_engineer = game_robot_HP_data.ally_2_robot_HP;
    msg.friend_infantry3 = game_robot_HP_data.ally_3_robot_HP;
    msg.friend_infantry4 = game_robot_HP_data.ally_4_robot_HP;
    msg.friend_sentry = game_robot_HP_data.ally_7_robot_HP;
    msg.friend_outpost = game_robot_HP_data.ally_outpost_HP;
    msg.friend_base = game_robot_HP_data.ally_base_HP;

    // 队友位置 (基于 global_red_map)
    msg.hero_x = ground_robot_position_t_data.hero_x;
    msg.hero_y = ground_robot_position_t_data.hero_y;
    msg.engineer_x = ground_robot_position_t_data.engineer_x;
    msg.engineer_y = ground_robot_position_t_data.engineer_y;
    msg.infantry3_x = ground_robot_position_t_data.standard_3_x;
    msg.infantry3_y = ground_robot_position_t_data.standard_3_y;
    msg.infantry4_x = ground_robot_position_t_data.standard_4_x;
    msg.infantry4_y = ground_robot_position_t_data.standard_4_y;

    if (robot_info_t_data.remain_HP == 0) {
        sentry_cmd_info_t_data.confirm_revive = 1;
    } else {
        sentry_cmd_info_t_data.confirm_revive = 0;
    }

    msg.bullet_17mm = robot_info_t_data.shooter_cooling_rate;
    msg.energy_mechanism_available =
        ext_sentry_info_data.sentry_info_2.energy_mechanism_available;
    // msg.projectile_allowance_fortress =
    //     bullet_remaining_t_data.projectile_allowance_fortress;
    refree_pub_->publish(msg);

    // if ((robot_command_map_data.target_position_x !=
    //          last_robot_command_map_data.target_position_x ||
    //      robot_command_map_data.target_position_y !=
    //          last_robot_command_map_data.target_position_y) &&
    //     (robot_command_map_data.target_position_x != 0.0 &&
    //      robot_command_map_data.target_position_y != 0.0)) {
    //     geometry_msgs::msg::Point point;
    //     point.x = robot_command_map_data.target_position_x;
    //     point.y = robot_command_map_data.target_position_y;
    //     point.z = 0.0;

    //     last_robot_command_map_data.target_position_x =
    //         robot_command_map_data.target_position_x;
    //     last_robot_command_map_data.target_position_y =
    //         robot_command_map_data.target_position_y;

    //     trigger_pub_->publish(point);

    //     std::cout << "收到触发消息！" << std::endl;
    // }
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
