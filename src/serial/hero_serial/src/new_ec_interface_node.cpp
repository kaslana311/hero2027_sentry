#include "new_ec_interface_node.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <rclcpp/time.hpp>

EcInterfaceNode ::EcInterfaceNode() : Node("ec_interface_node") {
    // 初始化串口
    if (!boost_asio_.Init("/dev/usbttl", 460800)) {
        RCLCPP_ERROR(this->get_logger(), "与云台的串口初始化失败");
        rclcpp::shutdown();  // 结束程序
        return;
    }
    InitDecisionMsg();
    // 初始化消息
    read_pack_ = std::make_shared<interfaces::msg::Readpack>();

    pack_pub_ =
        this->create_publisher<interfaces::msg::Readpack>("Readpack", 1);

    // 订阅发送消息
    pack_sub_ = this->create_subscription<interfaces::msg::Sendpack>(
        "Sendpack", 1,
        std::bind(&EcInterfaceNode::SendpackCallback, this,
                  std::placeholders::_1));

    ec_to_vision_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&EcInterfaceNode::EcToVisionTimerCallback, this));
    vision_to_ec_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&EcInterfaceNode::VisionToEcTimerCallback, this));

    decision_sub_ =
        this->create_subscription<serial_interfaces::msg::Gimbaldecision>(
            "decision", 1,
            std::bind(&EcInterfaceNode::DecisionCallback, this,
                      std::placeholders::_1));
    this->declare_parameter("is_sentry", false);
    this->get_parameter("is_sentry", is_sentry_);
    // 避免空指针解引用
    // decision_msg_->targets.resize(7);

    // 记录开始时间
    start_time_ = this->now();
}

void EcInterfaceNode::SendpackCallback(
    const interfaces::msg::Sendpack::SharedPtr msg) {
    target_found_ = msg->target_found;
    cv_to_ec_builder_.set_work_mode(
        static_cast<gimbal_protocol::WorkMode>(msg->mode));
    cv_to_ec_builder_.set_precision(msg->pitch_resolution, msg->yaw_resolution);
    // yaw的速度是反的
    cv_to_ec_builder_.set_prediction(msg->pred_pitch, msg->pred_yaw,
                                     msg->pitch_palstance,
                                     -(msg->yaw_palstance));
    cv_to_ec_builder_.set_status_flags(msg->target_found, msg->fire,
                                       msg->burst_shoot, 0, msg->rune_fire,
                                       msg->rune_burst_shoot, msg->isproperly);
    // 毫秒
    cv_to_ec_builder_.set_timestamps(
        ConvertVisionToMcuTime(this->now().nanoseconds() / 1e6),
        ConvertVisionToMcuTime(msg->wannfiretime));
    // cv_to_ec_builder_.set_timestamps(ConvertVisionToMcuTime(msg->clock),
    // ConvertVisionToMcuTime(msg->wannfiretime));
}

void EcInterfaceNode::VisionToEcTimerCallback() {
    // 如果是哨兵，自瞄的数据不直接给，而是会做处理
    if (is_sentry_) {
        std::cout << "is_sentry" << std::endl;
        if (decision_msg_->is_reached && decision_msg_->is_gimbal_patrol) {
            if (target_found_) {
                // 发现目标：重置状态，进入自瞄模式
                target_lost_time_ = rclcpp::Time(0);  // 清空丢失时间

                RCLCPP_INFO(this->get_logger(), "发现目标，开始打击");
            } else {
                // 目标丢失：记录时间并检查延时

                if (target_lost_time_.seconds() == 0) {
                    target_lost_time_ = this->now();  // 第一次丢失时记录时间
                }
                // 计算时间差（例如延时0.5秒）

                if ((this->now() - target_lost_time_).seconds() > 0.5) {
                    RCLCPP_INFO(this->get_logger(),
                                "目标丢失超过0.5秒，开始巡逻");
                    // is_patrolling_ = true;
                    // 启动巡逻逻辑
                    // 云台，yaw角，从云台头上向下看，逆正顺副，逆时针旋转，角度变换为0-180,-180-0);
                    // 云台，pitch角，抬头正，低头负。
                    // 获取预测的yaw和pitch
                    PatrolLogic(0, 60, 0, -20);
                    std::cout << "patrol_yaw_:" << patrol_yaw_ << std::endl;
                    std::cout << "patrol_pitch_:" << patrol_pitch_ << std::endl;
                    cv_to_ec_builder_.set_precision(1000, 1000);
                    cv_to_ec_builder_.set_prediction(patrol_pitch_, patrol_yaw_,
                                                     0, 0);
                    cv_to_ec_builder_.set_work_mode(
                        static_cast<gimbal_protocol::WorkMode>(1));
                    cv_to_ec_builder_.set_timestamps(
                        ConvertVisionToMcuTime(this->now().nanoseconds() / 1e6),
                        ConvertVisionToMcuTime(0));
                }
            }
        }
        // 非巡逻模式或未到达目标点的情况,行进过程中
        else {
            RCLCPP_INFO(this->get_logger(), "行进过程中");

            if (target_found_) {
                // 正常自瞄
                target_lost_time_ = rclcpp::Time(0);  // 清空丢失时间
                RCLCPP_INFO(this->get_logger(), "发现目标，开始打击");
            } else {
                if (target_lost_time_.seconds() == 0) {
                    target_lost_time_ = this->now();
                }
                if ((this->now() - target_lost_time_).seconds() > 0.5) {
                    PatrolLogic(decision_msg_->yaw_degrees, 0, 0, 0);
                    // PatrolLogic(6, 0, 0);

                    std::cout << "yaw:" << patrol_yaw_ << std::endl;
                    std::cout << "pitch:" << patrol_pitch_ << std::endl;
                    cv_to_ec_builder_.set_precision(1000, 1000);
                    cv_to_ec_builder_.set_prediction(patrol_pitch_, patrol_yaw_,
                                                     0, 0);
                    cv_to_ec_builder_.set_work_mode(
                        static_cast<gimbal_protocol::WorkMode>(1));
                    cv_to_ec_builder_.set_timestamps(
                        ConvertVisionToMcuTime(this->now().nanoseconds() / 1e6),
                        ConvertVisionToMcuTime(0));
                }
            }
        }
    }
    auto buffer = cv_to_ec_builder_.build();
    boost::asio::mutable_buffer send_buffer =
        boost::asio::buffer(buffer, buffer.size());
    std::cout << "发送给电控信息" << std::endl;
    boost_asio_.Write(send_buffer);
}

void EcInterfaceNode::EcToVisionTimerCallback() {
    if (boost_asio_.Read()) {
        const auto& data = boost_asio_.GetReadBuffer();
        size_t processed = 0;
        // std::cout << "du" <<std::endl;
        while (true) {
            size_t header_pos = FindFrameHeader(data);
            if (header_pos == static_cast<size_t>(-1)) {
                break;  // 未找到帧头，退出循环
            }

            // 确定帧头类型及对应包长度
            if (header_pos + 1 >= data.size()) {
                break;  // 数据不足以检查帧头类型
            }
            // 确定帧头类型及对应包长度，字节号为4的最高位表示帧头类型
            uint8_t frame_type = data[header_pos + 4] >> 7;
            size_t packet_length = 0;

            if (frame_type == 0x00) {  // TIMING包，19字节

                packet_length = 19;
            } else if (frame_type == 0x01) {  // POSE包，21字节

                packet_length = 21;
            } else {
                // 无效帧头类型，跳过当前帧头
                boost_asio_.Consume(header_pos + 1);
                continue;
            }

            // 检查数据是否足够完整
            if (data.size() - header_pos < packet_length) {
                break;  // 数据不完整，等待后续数据
            }

            // 提取对应长度的数据包
            std::vector<uint8_t> packet(
                data.begin() + header_pos,
                data.begin() + header_pos + packet_length);

            // 解析数据包
            if (ec_to_cv_parser_.parse(packet)) {
                // 声明所有需要的变量
                // int64_t high_time = 0;
                int64_t low_time = 0;
                int64_t full_mcu_time = 0;
                int64_t vision_time = 0;

                switch (ec_to_cv_parser_.packet_type()) {
                    case combat_protocol::PacketType::TIMING:
                        // 处理TIMING包字段
                        high_time_ = ec_to_cv_parser_.timing().timestamp;

                        read_pack_->mode = ec_to_cv_parser_.timing().aim_mode;
                        // 电控那边传的是千位数，实际上弹速应该位十位数
                        read_pack_->bullet_speed =
                            ec_to_cv_parser_.timing().bullet_speed / 1e2;
                        read_pack_->reload_delay_time =
                            ec_to_cv_parser_.timing().dial_delay;
                        read_pack_->enemy_number =
                            ec_to_cv_parser_.timing().enemy_number;
                        read_pack_->request_fire =
                            ec_to_cv_parser_.timing().fire_cmd;
                        read_pack_->enemy_color =
                            ec_to_cv_parser_.timing().is_red;
                        read_pack_->stage_remain_time =
                            ec_to_cv_parser_.timing().stage_remain_time;

                        break;

                    case combat_protocol::PacketType::POSE:
                        // 处理POSE包字段
                        low_time = ec_to_cv_parser_.pose().ec_timestamp;
                        full_mcu_time = CombineMcuTime(high_time_, low_time);
                        // 纳秒转毫秒
                        vision_time = this->now().nanoseconds() / 1e6;
                        RecordSyncPoint(full_mcu_time, vision_time);

                        read_pack_->pose_mcu_time = full_mcu_time;
                        std::cout << "发送给电控t:" << full_mcu_time
                                  << std::endl;

                        read_pack_->pose_visual_time =
                            ConvertMcuToVisionTime(full_mcu_time);
                        // 计算出的是毫秒，自瞄那边需要秒，带小数点
                        read_pack_->pose.visual_time =
                            ConvertMcuToVisionTime(full_mcu_time) / 1e3;
                        read_pack_->fired = ec_to_cv_parser_.pose().fired;
                        read_pack_->pose.ptz_pitch =
                            ec_to_cv_parser_.pose().pitch;
                        read_pack_->pose.ptz_yaw = ec_to_cv_parser_.pose().yaw;
                        read_pack_->pose.ptz_roll =
                            ec_to_cv_parser_.pose().roll;
                        read_pack_->rightclick =
                            ec_to_cv_parser_.pose().right_click;
                        // 记录当前云台角度
                        current_yaw_ = ec_to_cv_parser_.pose().yaw;
                        current_pitch_ = ec_to_cv_parser_.pose().pitch;
                        break;

                    default:
                        processed = header_pos + 1;
                        break;
                }
                // 发布消息
                pack_pub_->publish(*read_pack_);
                processed = header_pos + packet_length;
            } else {
                processed = header_pos + 1;
            }

            // 移除已处理的数据
            boost_asio_.Consume(processed);
        }
    }
}
void EcInterfaceNode::DecisionCallback(
    const serial_interfaces::msg::Gimbaldecision::SharedPtr msg) {
    decision_msg_ = msg;
    // std::cout << "decision_msg_->yaw_degrees: " << decision_msg_->yaw_degrees
    // << std::endl; std::cout << "decision_msg_->is_auto_aim: " <<
    // decision_msg_->is_auto_aim << std::endl; std::cout <<
    // "decision_msg_->auto_aim_mode: " << decision_msg_->auto_aim_mode <<
    // std::endl; std::cout << "decision_msg_->is_reached: " <<
    // decision_msg_->is_reached << std::endl; std::cout <<
    // "decision_msg_->is_gimbal_patrol: " << decision_msg_->is_gimbal_patrol <<
    // std::endl;
    // // std::cout << "decision_msg_->targets: " << decision_msg_->targets <<
    // std::endl; for (int i = 0; i < decision_msg_->targets.size(); i++) {
    //   std::cout << "decision_msg_->targets[" << i << "].type: " <<
    //   decision_msg_->targets[i].type << std::endl; std::cout <<
    //   "decision_msg_->targets[" << i << "].priority: " <<
    //   decision_msg_->targets[i].priority << std::endl;
    // }
}

size_t EcInterfaceNode::FindFrameHeader(const std::vector<uint8_t>& data) {
    if (data.size() < 4) return -1;
    for (size_t i = 0; i <= data.size() - 4; ++i) {
        if (data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x80 &&
            data[i + 3] == 0x7F) {
            return i;
        }
    }
    return -1;
}

int64_t EcInterfaceNode::CombineMcuTime(int64_t high_time, int64_t low_time) {
    // 合并高位时间（千位及以上）和低位时间（百位及以下）
    // high_time: 高位时间 t1
    // low_time: 低位时间 t2
    // 123456
    // high 123400
    // low 456
    // std::cout << "high" << high_time <<std::endl;
    // std::cout << "low" << low_time <<std::endl;

    return high_time / 1000 * 1000 + (low_time % 1000);
}

void EcInterfaceNode::RecordSyncPoint(int64_t mcu_time, int64_t vision_time) {
    TimePoint new_point{mcu_time, vision_time};

    // 如果已达到最大同步点数量，移除最旧的点
    if (sync_points_.size() >= MAX_SYNC_POINTS) {
        sync_points_.erase(sync_points_.begin());
    }

    // 添加新的同步点
    sync_points_.push_back(new_point);

    // 如果有足够的同步点，更新校准参数
    if (sync_points_.size() >= MIN_SYNC_POINTS) {
        UpdateTimeCalibration();
    }
}

void EcInterfaceNode::UpdateTimeCalibration() {
    if (sync_points_.size() < MIN_SYNC_POINTS) {
        return;
    }

    // 使用最新的两个点计算校准参数
    const TimePoint& p1 =
        sync_points_[sync_points_.size() - 2];  // T电控1, T视觉1
    const TimePoint& p2 =
        sync_points_[sync_points_.size() - 1];  // T电控2, T视觉2

    // 计算时间差
    int64_t delta_mcu = p2.mcu_time - p1.mcu_time;           // T电控2 - T电控1
    int64_t delta_vision = p2.vision_time - p1.vision_time;  // T视觉2 - T视觉1

    // 检查时间差是否在合理范围内
    if (std::abs(delta_mcu) > MAX_TIME_DIFF ||
        std::abs(delta_vision) > MAX_TIME_DIFF) {
        RCLCPP_WARN(this->get_logger(),
                    "Time difference too large, skipping calibration");
        return;
    }

    // 计算新的比例系数和偏移量
    if (delta_mcu != 0) {  // 避免除以零
        // a = (T视觉2 - T视觉1) / (T电控2 - T电控1)
        double new_scale =
            static_cast<double>(delta_vision) / static_cast<double>(delta_mcu);

        // 更新比例系数
        time_scale_factor_ = new_scale;

        // b = T视觉1 - a × T电控1
        time_offset_ = p1.vision_time -
                       static_cast<int64_t>(time_scale_factor_ * p1.mcu_time);

        RCLCPP_INFO(this->get_logger(),
                    "Updated time calibration: scale=%.6f, offset=%ld",
                    time_scale_factor_, time_offset_);
    }
}

int64_t EcInterfaceNode::ConvertMcuToVisionTime(int64_t mcu_time) {
    // t3 = a × T电控 + b
    return static_cast<int64_t>(time_scale_factor_ * mcu_time) + time_offset_;
}

int64_t EcInterfaceNode::ConvertVisionToMcuTime(int64_t vision_time) {
    // 从 t3 = a × T电控 + b
    // 解出 T电控 = (t3 - b) / a
    if (std::abs(time_scale_factor_) < 1e-6) {
        RCLCPP_INFO(
            this->get_logger(),
            "Time scale factor too close to zero, using default conversion");
        return vision_time;  // 如果比例因子接近0，直接返回输入时间
    }
    return static_cast<int64_t>((vision_time - time_offset_) /
                                time_scale_factor_);
}

void EcInterfaceNode::PatrolLogic(float delta_yaw, float yaw_range,
                                  float max_pitch, float min_pitch) {
    // yaw轴角度处理
    const float yaw_center = current_yaw_ + delta_yaw;  // yaw轴正弦运动的中心线
    const float yaw_amplitude = yaw_range / 2.0f;       // yaw轴摆动幅度
    const float yaw_period_sec = 3.6f;                  // yaw轴摆动周期1.2秒
    // 计算当前yaw角
    float t_yaw = (this->now() - start_time_).seconds();
    float yaw_sin_value = sinf(2 * M_PI * t_yaw / yaw_period_sec);
    patrol_yaw_ = yaw_center + yaw_amplitude * yaw_sin_value;
    // 处理360度周期
    patrol_yaw_ = std::fmod(patrol_yaw_, 360.0);
    if (patrol_yaw_ < 0) {
        patrol_yaw_ += 360.0;
    }
    if (patrol_yaw_ > 180.0) {
        patrol_yaw_ -= 360.0;
    }
    // pitch轴角度处理
    // 摆动中心
    const float ptich_mid = (max_pitch + min_pitch) / 2;
    // 摆动幅度
    const float ptich_amplitude = (max_pitch - min_pitch) / 2;
    // 摆动周期，0.6秒一个周期
    const float period_sec = 0.6f;

    // 计算当前俯仰角（带限幅保护）
    float t_pitch = (this->now() - start_time_).seconds();
    float sin_value = sinf(2 * 3.14f * t_pitch / period_sec);
    // sin_value = std::clamp(sin_value, -1.0f, 1.0f);  // 防止浮点误差越界
    patrol_pitch_ = ptich_mid + ptich_amplitude * sin_value;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EcInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}