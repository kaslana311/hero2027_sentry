// 云台电脑发给云台通信协议的解析与构建工具

#pragma once
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace combat_protocol {

enum class PacketType { TIMING = 0, POSE = 1 };

// ================== 协议结构定义 ==================
#pragma pack(push, 1)

// 对时包结构 (总长度17字节)
struct TimingPacket {
    uint8_t header[4] = {0x00, 0x00, 0x80, 0x7F};  // 固定帧头
    uint8_t type_length = 0x0E;                    // 类型0 + 长度14
    uint32_t timestamp;                            // 时间戳（5-8）
    uint8_t aim_bullet_high;  // 9: [7-4]子弹速度g [3-0]自瞄模式
    uint8_t bullet_low;       // 10: 子弹速度d8位
    uint8_t target_status;    // 11: [7]发弹 [6]保留 [5-2]敌方数字 [1]颜色
    uint32_t dial_delay;      // 12-15: 拨弹延时
    uint16_t stage_remain_time;
    uint8_t checksum;  // 16: 校验和
};

// 姿态包结构 (总长度21字节)
struct PosePacket {
    uint8_t header[4] = {0x00, 0x00, 0x80, 0x7F};  // 固定帧头
    uint8_t type_length = 0x90;                    // 类型1 + 长度16（0x10）
    float pitch;                                   // 5-8: 俯仰角
    float yaw;                                     // 9-12: 偏航角
    float roll;                                    // 13-16: 滚转角
    uint16_t ec_timestamp;                         // 17-18: 电控时间戳
    uint8_t status;                                // 19: [0]发弹 [1]右键状态
    uint8_t checksum;                              // 20: 校验和
};
#pragma pack(pop)

// ================== 构建工具 ==================
class ProtocolBuilder {
   public:
    // 构建对时包
    static std::vector<uint8_t> build_timing(uint32_t timestamp,
                                             uint8_t aim_mode,
                                             uint16_t bullet_speed, bool is_red,
                                             uint8_t enemy_number,
                                             bool fire_cmd, uint32_t delay_ms,
                                             uint16_t stage_remain_time) {
        TimingPacket pkt;
        pkt.timestamp = timestamp;
        pkt.aim_bullet_high =
            ((bullet_speed >> 8) & 0x0F) << 4 | (aim_mode & 0x0F);
        pkt.bullet_low = bullet_speed & 0xFF;
        pkt.target_status =
            (fire_cmd << 7) | ((enemy_number & 0x0F) << 2) | (is_red << 1);
        pkt.dial_delay = delay_ms;
        pkt.stage_remain_time = stage_remain_time;
        return finalize(pkt);
    }

    // 构建姿态包
    static std::vector<uint8_t> build_pose(float pitch, float yaw, float roll,
                                           uint16_t ec_timestamp, bool fired,
                                           bool right_click) {
        PosePacket pkt;
        pkt.pitch = pitch;
        pkt.yaw = yaw;
        pkt.roll = roll;
        pkt.ec_timestamp = ec_timestamp;
        pkt.status = (right_click << 1) | fired;
        return finalize(pkt);
    }

   private:
    template <typename T>
    static std::vector<uint8_t> finalize(T& packet) {
        std::vector<uint8_t> data;
        const uint8_t* p = reinterpret_cast<uint8_t*>(&packet);

        // 添加除校验和外的所有数据
        data.insert(data.end(), p, p + sizeof(packet) - 1);

        // 计算校验和
        packet.checksum = 0;
        size_t check_len = (sizeof(packet) - 1) - 5;  // 从type_length开始
        for (size_t i = 5; i < 5 + check_len; ++i) packet.checksum += data[i];

        data.push_back(packet.checksum);
        return data;
    }
};

// ================== 解析工具 ==================
class ProtocolParser {
   public:
    bool parse(const std::vector<uint8_t>& data) {
        reset_state();
        // std::cout << "1111111111111" << std::endl;
        // 基础校验
        if (data.size() < 5) return false;
        if (memcmp(data.data(), "\x00\x00\x80\x7F", 4) != 0) return false;
        // std::cout << "2222222222222" << std::endl;

        // 解析类型和长度
        type_ = static_cast<PacketType>(data[4] >> 7);
        // std::cout << "type_"<< (int)type_ << std::endl;

        uint8_t length = data[4] & 0x7F;
        // std::cout << "length"<< (int)length << std::endl;

        // std::cout << "33333333333333333333333333" << std::endl;

        // 长度验证
        if ((type_ == PacketType::TIMING &&
             (data.size() != 19 || length != 0x0E)) ||
            (type_ == PacketType::POSE &&
             (data.size() != 21 || length != 0x10)))
            return false;
        // std::cout << "4444444444444444444444444" << std::endl;
        // std::cout << "data.size()" << data.size() << std::endl;

        // 校验和验证
        uint8_t sum = 0;
        size_t check_end = (type_ == PacketType::TIMING) ? 18 : 20;
        // std::cout << "check_end" << check_end << std::endl;

        for (size_t i = 5; i < check_end; ++i) sum += data[i];
        if (sum != data[check_end]) {
            // std::cout << "jiaoyanhebutongguo" << std::endl;

            return false;
        }
        // std::cout <<
        // "5555555555555555555555555555555555555555555555555555555555555555555555"
        // << std::endl;

        // 分类型解析
        return type_ == PacketType::TIMING ? parse_timing(data)
                                           : parse_pose(data);
    }

    // 获取协议类型
    PacketType packet_type() const { return type_; }

    // 对时包数据
    struct TimingData {
        uint32_t timestamp;
        uint8_t aim_mode;
        uint16_t bullet_speed;
        bool is_red;
        uint8_t enemy_number;
        bool fire_cmd;
        uint32_t dial_delay;
        uint16_t stage_remain_time;
    };
    TimingData timing() const {
        validate_type(PacketType::TIMING);
        return timing_data_;
    }

    // 姿态包数据
    struct PoseData {
        float pitch;
        float yaw;
        float roll;
        uint16_t ec_timestamp;
        bool fired;
        bool right_click;
    };
    PoseData pose() const {
        validate_type(PacketType::POSE);
        return pose_data_;
    }

   private:
    void reset_state() {
        type_ = PacketType::TIMING;
        timing_data_ = {};
        pose_data_ = {};
    }

    bool parse_timing(const std::vector<uint8_t>& data) {
        const auto* pkt = reinterpret_cast<const TimingPacket*>(data.data());
        timing_data_ = {
            .timestamp = pkt->timestamp,
            .aim_mode = static_cast<uint8_t>(pkt->aim_bullet_high & 0x0F),
            .bullet_speed = static_cast<uint16_t>(
                (pkt->aim_bullet_high & 0xF0) << 4 | pkt->bullet_low),
            .is_red = (pkt->target_status >> 1) & 0x01,
            .enemy_number = (pkt->target_status >> 2) & 0x0F,
            .fire_cmd = (pkt->target_status >> 7) & 0x01,
            .dial_delay = pkt->dial_delay,
            .stage_remain_time = pkt->stage_remain_time};
        return true;
    }

    bool parse_pose(const std::vector<uint8_t>& data) {
        const auto* pkt = reinterpret_cast<const PosePacket*>(data.data());
        pose_data_ = {.pitch = pkt->pitch,
                      .yaw = pkt->yaw,
                      .roll = pkt->roll,
                      .ec_timestamp = pkt->ec_timestamp,
                      .fired = pkt->status & 0x01,
                      .right_click = (pkt->status >> 1) & 0x01};
        return true;
    }

    void validate_type(PacketType expected) const {
        if (type_ != expected) {
            std::cout << "type_" << (int)type_ << std::endl;
            std::cout << "Pexpected" << (int)expected << std::endl;

            // throw std::logic_error("Packet type mismatch");
            std::cout << "Packet type mismatch" << std::endl;
        }
    }

    PacketType type_;
    union {
        TimingData timing_data_;
        PoseData pose_data_;
    };
};

}  // namespace combat_protocol

// // 构建姿态包
// auto pose_pkt = combat_protocol::ProtocolBuilder::build_pose(
//     15.5f,   // pitch
//     30.2f,   // yaw
//     5.1f,    // roll
//     0xABCD,  // 电控时间戳
//     true,    // 已发弹
//     false    // 右键未按下
// );

// // 解析数据包
// combat_protocol::ProtocolParser parser;
// if (parser.parse(pose_pkt)) {
//     if (parser.packet_type() == combat_protocol::PacketType::POSE) {
//         auto data = parser.pose();
//         std::cout << "滚转角: " << data.roll << "°\n"
//                   << "电控时间: " << data.ec_timestamp << "ms"
//                   << std::endl;
//     }
// }