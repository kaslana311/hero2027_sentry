// 发给底盘控制信息的构建与解析工具
#pragma once
#include <algorithm>
#include <cstdint>
#include <cstring>  // 用于memcpy
#include <vector>

// 小端通信
namespace cmd_protocol {

#pragma pack(push, 1)
struct VehicleInfo {
  int32_t id;  // int型，4字节
  float x;     // float，4字节
  float y;     // float，4字节
};

struct ProtocolHeader {
  uint32_t frame_header;  // 0-3字节：帧头
  float linear_vel_x;  // 4-7字节：线速度x分量（float，单位mm/s）
  float linear_vel_y;  // 8-11字节：线速度y分量
  float angular_vel;   // 12-13字节：角速度（short，单位0.1度/s）
  uint8_t gyro_mode_flag;  // 14字节：小陀螺模式标志
  int16_t virtual_yaw;  // 15-16字节：虚拟云台朝向（short，单位0.1度）
  uint32_t sentinel_status;  // 17-20字节：哨兵状态
  uint8_t vehicle_count;     // 21-24字节：车辆数目（int）
};
#pragma pack(pop)

class ProtocolBuilder {
 public:
  void reset() {
    data_.clear();
    header_ = ProtocolHeader{};
    vehicles_.clear();
  }

  void set_motion_params(float vel_x_mms, float vel_y_mms,
                         float angular_vel_0_1deg_s) {
    header_.linear_vel_x = vel_x_mms;
    header_.linear_vel_y = vel_y_mms;
    header_.angular_vel = angular_vel_0_1deg_s;
  }

  // 获取当前设置的运动参数
  float get_linear_vel_x() const { return header_.linear_vel_x; }
  float get_linear_vel_y() const { return header_.linear_vel_y; }
  float get_angular_vel() const { return header_.angular_vel; }

  void set_gyro_mode(bool enable) {
    header_.gyro_mode_flag = enable ? 0x01 : 0x00;
  }

  void set_virtual_yaw(float degrees) {
    header_.virtual_yaw = static_cast<int16_t>(degrees * 10);
  }

  void set_sentinel_status(bool confirm_revive, bool immediate_revive,
                           uint16_t ammo_amount, uint8_t ammo_request,
                           uint8_t hp_request, uint16_t command_code = 0) {
    uint32_t status = 0;
    status |= (confirm_revive ? 0x01 : 0x00);
    status |= (immediate_revive ? 0x02 : 0x00);
    status |= (ammo_amount & 0x07FF) << 2;
    status |= (ammo_request & 0x0F) << 13;
    status |= (hp_request & 0x0F) << 17;
    status |= (command_code & 0x07FF) << 21;
    header_.sentinel_status = status;
  }

  void add_vehicle(int32_t id, float x_m, float y_m) {
    VehicleInfo vehicle{id, x_m, y_m};
    vehicles_.push_back(vehicle);
  }

  std::vector<uint8_t> build() {
    header_.frame_header = 0x7F800000;  // 小端帧头
    header_.vehicle_count = static_cast<uint8_t>(vehicles_.size());
    // std::cout <<"vehicle size:" << vehicles_.size() << std::endl;

    std::vector<uint8_t> packet;
    // 添加协议头（25字节）
    const uint8_t* header_ptr = reinterpret_cast<const uint8_t*>(&header_);
    packet.insert(packet.end(), header_ptr,
                  header_ptr + sizeof(ProtocolHeader));
    // std::cout <<"packet.size:" << packet.size() << std::endl;

    // 添加车辆信息（每个12字节）
    for (const auto& vehicle : vehicles_) {
      const uint8_t* id_ptr = reinterpret_cast<const uint8_t*>(&vehicle.id);
      packet.insert(packet.end(), id_ptr, id_ptr + sizeof(int32_t));
      const uint8_t* x_ptr = reinterpret_cast<const uint8_t*>(&vehicle.x);
      packet.insert(packet.end(), x_ptr, x_ptr + sizeof(float));
      const uint8_t* y_ptr = reinterpret_cast<const uint8_t*>(&vehicle.y);
      packet.insert(packet.end(), y_ptr, y_ptr + sizeof(float));
    }

    // 计算校验和（从第4字节到倒数第二字节）
    uint8_t checksum = 0;
    for (size_t i = 4; i < packet.size(); ++i) {
      checksum += packet[i];
    }
    packet.push_back(checksum);
    // int size = 0;
    // for (auto i : packet)
    // {

    //     std::cout << "字节" << size <<":";

    //     size ++;
    //     printf("%x \n", i);
    // }

    return packet;
  }

 private:
  ProtocolHeader header_;
  std::vector<VehicleInfo> vehicles_;
  std::vector<uint8_t> data_;
};

class ProtocolParser {
 public:
  bool parse(const std::vector<uint8_t>& data) {
    if (data.size() < sizeof(ProtocolHeader) + 1) {
      return false;
    }
    if (!verify_checksum(data)) {
      return false;
    }

    const ProtocolHeader* header =
        reinterpret_cast<const ProtocolHeader*>(data.data());
    if (header->frame_header != 0x7F800000) {  // 修正帧头检查
      return false;
    }

    // 解析基本参数
    linear_vel_x_ = header->linear_vel_x;
    linear_vel_y_ = header->linear_vel_y;
    angular_vel_ = header->angular_vel;
    virtual_yaw_ = static_cast<float>(header->virtual_yaw) / 10.0f;
    is_gyro_mode_ = header->gyro_mode_flag & 0x01;

    parse_sentinel_status(header->sentinel_status);

    // 车辆信息起始位置：sizeof(ProtocolHeader) = 25字节
    return parse_vehicles(data.data() + sizeof(ProtocolHeader),
                          header->vehicle_count,
                          data.size() - sizeof(ProtocolHeader) - 1);
  }

  // 获取解析结果...
  float get_virtual_yaw() const { return virtual_yaw_; }
  float get_linear_vel_x() const { return linear_vel_x_; }
  float get_linear_vel_y() const { return linear_vel_y_; }
  float get_angular_vel() const { return angular_vel_; }
  bool is_gyro_mode() const { return is_gyro_mode_; }
  bool is_confirm_revive() const { return confirm_revive_; }
  bool is_immediate_revive() const { return immediate_revive_; }
  uint16_t get_ammo_amount() const { return ammo_amount_; }
  uint8_t get_ammo_request() const { return ammo_request_; }
  uint8_t get_hp_request() const { return hp_request_; }

 private:
  bool verify_checksum(const std::vector<uint8_t>& data) {
    uint8_t sum = 0;
    for (size_t i = 4; i < data.size() - 1; ++i) {
      sum += data[i];
    }
    return sum == data.back();
  }

  void parse_sentinel_status(uint32_t status) {
    confirm_revive_ = status & 0x01;
    immediate_revive_ = status & 0x02;
    ammo_amount_ = (status >> 2) & 0x07FF;
    ammo_request_ = (status >> 13) & 0x0F;
    hp_request_ = (status >> 17) & 0x0F;
    command_code_ = (status >> 21) & 0x07FF;
  }

  bool parse_vehicles(const uint8_t* data, int32_t count, size_t remain_size) {
    vehicles_.clear();
    const size_t required = count * sizeof(VehicleInfo);
    if (remain_size < required + 1) {  // 剩余空间包括校验和
      return false;
    }

    for (int32_t i = 0; i < count; ++i) {
      const uint8_t* ptr = data + i * sizeof(VehicleInfo);
      VehicleInfo vehicle;
      memcpy(&vehicle.id, ptr, sizeof(int32_t));
      memcpy(&vehicle.x, ptr + 4, sizeof(float));
      memcpy(&vehicle.y, ptr + 8, sizeof(float));
      vehicles_.push_back(vehicle);
    }
    return true;
  }

  // 解析结果存储...
  std::vector<VehicleInfo> vehicles_;
  float linear_vel_x_ = 0.0f;
  float linear_vel_y_ = 0.0f;
  float angular_vel_ = 0.0f;
  float virtual_yaw_ = 0.0f;
  bool is_gyro_mode_ = false;
  // 哨兵状态
  bool confirm_revive_ = false;
  bool immediate_revive_ = false;
  uint16_t ammo_amount_ = 0;
  uint8_t ammo_request_ = 0;
  uint8_t hp_request_ = 0;
  uint16_t command_code_ = 0;
};
}  // namespace cmd_protocol