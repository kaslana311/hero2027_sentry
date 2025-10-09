#include "ec_interface_node.hpp"

#include <cstdint>
#include <memory>

#include "ec_to_cv_protocol_tool.hpp"
#include "interfaces/msg/readpack.hpp"

EcInterfaceNode ::EcInterfaceNode() : Node("ec_interface_node") {
    // 初始化串口
    if (!boost_asio_.Init("/dev/usbttl", 460800)) {
        RCLCPP_ERROR(this->get_logger(), "与云台的串口初始化失败");
        rclcpp::shutdown();  // 结束程序
        return;
    }
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
}

void EcInterfaceNode::SendpackCallback(
    const interfaces::msg::Sendpack::SharedPtr msg) {
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

    cv_to_ec_builder_.set_timestamps(ConvertVisionToMcuTime(msg->clock),
                                     ConvertVisionToMcuTime(msg->wannfiretime));
}

void EcInterfaceNode::VisionToEcTimerCallback() {
    auto buffer = cv_to_ec_builder_.build();
    boost::asio::mutable_buffer send_buffer =
        boost::asio::buffer(buffer, buffer.size());
    // std::cout << "发送给电控信息" << std::endl;
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

            if (frame_type == 0x00) {  // TIMING包，21字节

                packet_length = 17;
            } else if (frame_type == 0x01) {  // POSE包，34字节

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
                int64_t high_time = 0;
                int64_t low_time = 0;
                int64_t full_mcu_time = 0;
                int64_t vision_time = 0;

                switch (ec_to_cv_parser_.packet_type()) {
                    case combat_protocol::PacketType::TIMING:
                        // 处理TIMING包字段
                        std::cout << "timing" << std::endl;
                        high_time = ec_to_cv_parser_.timing().timestamp;

                        read_pack_->mode = ec_to_cv_parser_.timing().aim_mode;
                        read_pack_->bullet_speed =
                            ec_to_cv_parser_.timing().bullet_speed * 0.01;
                        read_pack_->reload_delay_time =
                            ec_to_cv_parser_.timing().dial_delay;
                        read_pack_->enemy_number =
                            ec_to_cv_parser_.timing().enemy_number;
                        read_pack_->request_fire =
                            ec_to_cv_parser_.timing().fire_cmd;
                        read_pack_->enemy_color =
                            ec_to_cv_parser_.timing().is_red;
                        break;

                    case combat_protocol::PacketType::POSE:
                        // 处理POSE包字段

                        low_time = ec_to_cv_parser_.pose().ec_timestamp;
                        full_mcu_time = CombineMcuTime(high_time, low_time);
                        vision_time = this->now().nanoseconds() / 1e6;

                        RecordSyncPoint(full_mcu_time, vision_time);

                        read_pack_->pose_mcu_time = full_mcu_time;
                        read_pack_->pose_visual_time =
                            ConvertMcuToVisionTime(full_mcu_time);

                        read_pack_->pose.visual_time =
                            ConvertMcuToVisionTime(full_mcu_time);

                        read_pack_->fired = ec_to_cv_parser_.pose().fired;
                        read_pack_->pose.ptz_pitch =
                            ec_to_cv_parser_.pose().pitch;
                        read_pack_->pose.ptz_yaw = ec_to_cv_parser_.pose().yaw;
                        read_pack_->pose.ptz_roll =
                            ec_to_cv_parser_.pose().roll;
                        read_pack_->rightclick =
                            ec_to_cv_parser_.pose().right_click;
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
    return high_time * 1000 + (low_time % 1000);
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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EcInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}