//云台电脑发给云台通信协议的解析与构建工具
#pragma once
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace gimbal_protocol {

// ================== 协议枚举定义 ==================
enum class WorkMode : uint8_t {
  MODE_1 = 0x01,
  MODE_2 = 0x02,
  MODE_3 = 0x03,
  MODE_4 = 0x04,
  MODE_5 = 0x05,
  MODE_6 = 0x06
};

// ================== 协议结构定义 ==================
#pragma pack(push, 1)
struct ProtocolPacket {
  // 固定帧头 (4字节)
  uint8_t header[4] = {0x00, 0x00, 0x80, 0x7F};

  // 数据长度 (1字节)
  uint8_t data_length = 29;  // 固定值

  // 状态标志位 (1字节)
  union {
    struct {
      uint8_t target_found : 1;
      uint8_t can_fire : 1;
      uint8_t can_burst_fire : 1;
      uint8_t exit_energy : 1;
      uint8_t energy_fire_enable : 1;
      uint8_t energy_burst_enable : 1;
      uint8_t vision_selfcheck : 1;
      uint8_t reserved : 1;
    } bits;
    uint8_t byte;
  } status_flags;

  // 工作模式 (1字节)
  WorkMode work_mode;

  // 精度数据 (8字节)
  float pitch_precision;  // 7-10
  float yaw_precision;    // 11-14

  // 时间数据 (8字节)
  uint32_t current_time;   // 15-18
  uint32_t expected_time;  // 19-22

  // 预测数据 (16字节)
  float pred_pitch;     // 23-26
  float pred_yaw;       // 27-30
  uint8_t pitch_speed;  // 31
  uint8_t yaw_speed;    // 32

  // 校验和 (1字节)
  uint8_t checksum;  // 33
};
#pragma pack(pop)

// ================== 构建工具 ==================
class ProtocolBuilder {
 public:
  ProtocolBuilder() { reset(); }

  // 重置为初始状态
  void reset() {
    memset(&packet_, 0, sizeof(packet_));
    memcpy(packet_.header, "\x00\x00\x80\x7F", 4);
    packet_.data_length = 29;
  }

  // 设置状态标志
  void set_status_flags(bool target_found, bool can_fire, bool can_burst_fire,
                        bool exit_energy, bool energy_fire_enable,
                        bool energy_burst_enable, bool vision_selfcheck) {
    packet_.status_flags.bits.target_found = target_found;
    packet_.status_flags.bits.can_fire = can_fire;
    packet_.status_flags.bits.can_burst_fire = can_burst_fire;
    packet_.status_flags.bits.exit_energy = exit_energy;
    packet_.status_flags.bits.energy_fire_enable = energy_fire_enable;
    packet_.status_flags.bits.energy_burst_enable = energy_burst_enable;
    packet_.status_flags.bits.vision_selfcheck = vision_selfcheck;
  }

  // 设置工作模式（带范围检查）
  void set_work_mode(WorkMode mode) {
    if (static_cast<uint8_t>(mode) < 0x01 ||
        static_cast<uint8_t>(mode) > 0x06) {
      throw std::invalid_argument("Invalid work mode");
    }
    packet_.work_mode = mode;
  }

  // 设置精度数据
  void set_precision(float pitch, float yaw) {
    packet_.pitch_precision = pitch;
    packet_.yaw_precision = yaw;
  }

  // 设置时间数据
  void set_timestamps(uint32_t current_ms, uint32_t expected_ms) {
    packet_.current_time = current_ms;
    packet_.expected_time = expected_ms;
  }

  // 设置预测数据
  void set_prediction(float pred_pitch, float pred_yaw, uint8_t pitch_speed,
                      uint8_t yaw_speed) {
    packet_.pred_pitch = pred_pitch;
    packet_.pred_yaw = pred_yaw;
    packet_.pitch_speed = pitch_speed;
    packet_.yaw_speed = yaw_speed;
  }

  // 构建数据包
  std::vector<uint8_t> build() {
    std::vector<uint8_t> packet;
    const uint8_t* ptr = reinterpret_cast<uint8_t*>(&packet_);

    // 添加除校验和外的所有数据
    packet.insert(packet.end(), ptr, ptr + sizeof(packet_) - 1);

    // 计算校验和（5~32字节）
    packet_.checksum = 0;
    for (size_t i = 5; i <= 32; ++i) {
      packet_.checksum += packet[i];
    }
    packet.push_back(packet_.checksum);

    return packet;
  }

 private:
  ProtocolPacket packet_;
};

// ================== 解析工具 ==================
class ProtocolParser {
 public:
  bool parse(const std::vector<uint8_t>& data) {
    if (data.size() != 34)
      return false;

    // 校验帧头
    if (memcmp(data.data(), "\x00\x00\x80\x7F", 4) != 0)
      return false;

    // 校验数据长度
    if (data[4] != 29)
      return false;

    // 计算校验和
    uint8_t sum = 0;
    for (size_t i = 5; i <= 32; ++i)
      sum += data[i];
    if (sum != data[33])
      return false;

    // 拷贝数据到结构体
    memcpy(&packet_, data.data(), sizeof(packet_));
    return true;
  }

  // 获取状态标志
  struct StatusFlags {
    bool target_found;
    bool can_fire;
    bool can_burst_fire;
    bool exit_energy;
    bool energy_fire_enable;
    bool energy_burst_enable;
    bool vision_selfcheck;
  };
  StatusFlags get_status() const {
    return {
        .target_found = packet_.status_flags.bits.target_found,
        .can_fire = packet_.status_flags.bits.can_fire,
        .can_burst_fire = packet_.status_flags.bits.can_burst_fire,
        .exit_energy = packet_.status_flags.bits.exit_energy,
        .energy_fire_enable = packet_.status_flags.bits.energy_fire_enable,
        .energy_burst_enable = packet_.status_flags.bits.energy_burst_enable,
        .vision_selfcheck = packet_.status_flags.bits.vision_selfcheck};
  }

  // 获取工作模式
  WorkMode work_mode() const { return packet_.work_mode; }

  // 获取精度数据
  std::pair<float, float> get_precision() const {
    return {packet_.pitch_precision, packet_.yaw_precision};
  }

  // 获取时间数据
  std::pair<uint32_t, uint32_t> get_timestamps() const {
    return {packet_.current_time, packet_.expected_time};
  }

  // 获取预测数据
  struct Prediction {
    float pitch;
    float yaw;
    uint8_t pitch_speed;
    uint8_t yaw_speed;
  };
  Prediction get_prediction() const {
    return {.pitch = packet_.pred_pitch,
            .yaw = packet_.pred_yaw,
            .pitch_speed = packet_.pitch_speed,
            .yaw_speed = packet_.yaw_speed};
  }

 private:
  ProtocolPacket packet_;
};

}  // namespace gimbal_protocol

// 构建数据包
// gimbal_protocol::ProtocolBuilder builder;
// builder.set_status_flags(true, false, true, false, true, false, true);
// builder.set_work_mode(gimbal_protocol::WorkMode::MODE_C);
// builder.set_precision(0.12f, 0.08f);
// builder.set_timestamps(0x12345678, 0x9ABCDEF0);
// builder.set_prediction(15.5f, 30.2f, 25, 18);
// auto packet = builder.build();

// // 解析数据包
// gimbal_protocol::ProtocolParser parser;
// if (parser.parse(packet)) {
//     auto status = parser.get_status();
//     std::cout << "目标发现: " << status.target_found << "\n"
//               << "云台转速: " << parser.get_prediction().pitch_speed << "°/s"
//               << std::endl;
// }