// #pragma pack(1)
// typedef struct {
//   uint32_t header;  // 扩展帧头（增强抗干扰）
//   uint8_t is_autoaim;
//   uint8_t autoaim_mode;

//   struct {
//     uint8_t type;       // 机器人类型 (1B)
//     uint8_t priority;   // 优先级 (1B)
//   } targets[7];         // 最大7目标
//   uint8_t is_patroll;   // 巡逻状态 (0x00/0x01)
//   float yaw_angle;    // 角度值
//   uint8_t has_reached;  // 终点标志 (0x00/0x01)
//   uint8_t checksum;     // 校验和
// } SendGimbalFrame;

// 类型名称	            编码值
// Hero（英雄）	        0x01
// Infantry3（步兵3号)  0x02
// Infantry4（步兵4号)  0x03
// Engineer（工程）     0x04
// Sentry	(哨兵)        0x05
// Outpost(前哨站)      0x06
// Base（基地）         0x07

// 导航电脑发给云台电脑的通信协议的解析与构建工具

// SendGimbalProtocol.hpp
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <optional>
#include <vector>

// 协议常量定义
namespace GimbalProtocol {
constexpr uint32_t FRAME_HEADER = 0x7F800000;
constexpr size_t MAX_TARGETS = 7;
constexpr size_t FRAME_SIZE = 27;  // 修改为27字节

// 自瞄模式，哨兵其实就一个自瞄模式
enum AutoAimMode : uint8_t {
    NO_AUTOAIM = 0x00,
    AIM_COMMON = 0x01,
};
}  // namespace GimbalProtocol

// 协议构建器
class SendGimbalBuilder {
   public:
    struct Target {
        uint8_t type;
        uint8_t priority;
    };

    SendGimbalBuilder() {
        targets_.fill({0, 0});
        reset();
    }

    void reset() {
        target_count_ = 0;
        is_autoaim_ = 0;
        autoaim_mode_ = GimbalProtocol::AIM_COMMON;
        is_patroll_ = 0;
        yaw_angle_ = 0.0f;  // 默认中位角度0度
        has_reached_ = 0;
    }

    void configure(bool autoaim, GimbalProtocol::AutoAimMode mode,
                   bool patrolling, float yaw, bool reached) {
        setAutoAim(autoaim, mode);
        setPatrol(patrolling);
        setYaw(yaw);
        setReached(reached);
    }

    void setAutoAim(bool enable, GimbalProtocol::AutoAimMode mode) {
        is_autoaim_ = enable ? 0x01 : 0x00;
        autoaim_mode_ = static_cast<uint8_t>(mode);
    }

    void setPatrol(bool enable) { is_patroll_ = enable ? 0x01 : 0x00; }

    void setYaw(float degrees) {
        yaw_angle_ = std::clamp(degrees, -180.0f, 180.0f);
    }

    void setReached(bool status) { has_reached_ = status ? 0x01 : 0x00; }

    bool addTarget(uint8_t type, uint8_t priority) {
        // 类型有效性校验
        if (type < 0x01 || type > 0x07) {
            printf("[ERROR] 无效的目标类型: 0x%02X\n", type);
            return false;
        }

        // 优先级有效性校验（示例范围0-255）
        if (priority > 100) {  // 假设优先级最高为100
            printf("[WARN] 非标准优先级值: %d (建议范围0-100)\n", priority);
        }

        if (target_count_ >= GimbalProtocol::MAX_TARGETS) {
            printf("[ERROR] 目标数量已达上限(%zu)\n",
                   GimbalProtocol::MAX_TARGETS);
            return false;
        }

        // 输出调试信息
        printf("添加目标: 类型=%-8s (0x%02X), 优先级=%3d\n", TYPE_NAMES[type],
               type, priority);
        // targets是一个长为7的数组，里面存储了每一个机器人的类型以及击打优先级，数组没有顺序，即targets_[0]并不一定是英雄机器人，这取决于存入数组的顺序
        targets_[target_count_++] = {type, priority};
        return true;
    }

    void updateTarget(size_t index, uint8_t type, uint8_t priority) {
        if (index < GimbalProtocol::MAX_TARGETS) {
            targets_[index] = {type, priority};
            target_count_ =
                std::max(target_count_, static_cast<uint8_t>(index + 1));
        }
    }

    void clearTargets() {
        target_count_ = 0;
        for (auto& t : targets_) t = {0, 0};
    }

    std::vector<uint8_t> build() const {
        std::vector<uint8_t> frame(GimbalProtocol::FRAME_SIZE, 0);

        // Header
        std::memcpy(frame.data(), &GimbalProtocol::FRAME_HEADER, 4);

        // Control fields
        frame[4] = is_autoaim_;
        frame[5] = autoaim_mode_;

        // Targets
        for (size_t i = 0; i < GimbalProtocol::MAX_TARGETS; ++i) {
            frame[6 + i * 2] = targets_[i].type;
            frame[7 + i * 2] = targets_[i].priority;
        }

        // Status
        frame[20] = is_patroll_;
        std::memcpy(frame.data() + 21, &yaw_angle_,
                    sizeof(float));  // 写入4字节浮点
        frame[25] = has_reached_;

        // Checksum
        frame.back() = calculateChecksum(frame);
        return frame;
    }

   private:
    std::array<Target, GimbalProtocol::MAX_TARGETS> targets_;
    uint8_t target_count_;
    uint8_t is_autoaim_;
    uint8_t autoaim_mode_;
    uint8_t is_patroll_;
    float yaw_angle_;  // 类型改为float
    uint8_t has_reached_;

    const std::array<const char*, 8> TYPE_NAMES = {
        "Unknown",    // 0x00
        "Hero",       // 0x01
        "Infantry3",  // 0x02
        "Infantry4",  // 0x03
        "Engineer",   // 0x04
        "Sentry",     // 0x05
        "Outpost",    // 0x06
        "Base"        // 0x07
    };

    uint8_t calculateChecksum(const std::vector<uint8_t>& frame) const {
        uint8_t sum = 0;
        for (size_t i = 4; i < frame.size() - 1; ++i) {
            sum += frame[i];
        }
        return sum;
    }
};

class SendGimbalParser {
   public:
    struct Target {
        uint8_t type;
        uint8_t priority;
    };

    bool parse(const std::vector<uint8_t>& data) {
        if (data.size() != GimbalProtocol::FRAME_SIZE) return false;

        if (!verify_header(data)) return false;

        if (!verify_checksum(data)) return false;

        parse_fields(data);
        return true;
    }

    // 获取接口
    bool autoaim_enabled() const { return autoaim_enabled_; }
    uint8_t autoaim_mode() const { return autoaim_mode_; }
    const std::array<Target, GimbalProtocol::MAX_TARGETS>& targets() const {
        return targets_;
    }
    bool is_patrolling() const { return is_patrolling_; }
    float yaw_degrees() const { return yaw_degrees_; }
    bool has_reached() const { return has_reached_; }

   private:
    bool verify_header(const std::vector<uint8_t>& data) const {
        uint32_t received_header;
        memcpy(&received_header, data.data(), 4);
        return received_header == GimbalProtocol::FRAME_HEADER;
    }

    bool verify_checksum(const std::vector<uint8_t>& data) const {
        uint8_t sum = 0;
        for (size_t i = 4; i < data.size() - 1; ++i) sum += data[i];
        return sum == data.back();
    }

    void parse_fields(const std::vector<uint8_t>& data) {
        // 基础控制字段
        autoaim_enabled_ = data[4] != 0;
        autoaim_mode_ = data[5];

        // 解析目标列表
        for (size_t i = 0; i < GimbalProtocol::MAX_TARGETS; ++i) {
            const size_t base = 6 + i * 2;
            targets_[i] = {data[base], data[base + 1]};
        }

        // 状态字段
        is_patrolling_ = data[20] != 0;
        std::memcpy(&yaw_degrees_, data.data() + 21,
                    sizeof(float));  // 读取4字节浮点
        has_reached_ = data[25] != 0;
    }

    // 解析状态存储
    bool autoaim_enabled_ = false;
    uint8_t autoaim_mode_ = 0;
    std::array<Target, GimbalProtocol::MAX_TARGETS> targets_;
    bool is_patrolling_ = false;
    float yaw_degrees_ = 0.0f;
    bool has_reached_ = false;
};