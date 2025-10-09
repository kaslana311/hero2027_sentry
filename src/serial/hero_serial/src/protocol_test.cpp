#include "chrono"
#include "cmd_protocol_tool.hpp"
#include "cv_to_ec_protocol_tool.hpp"
#include "ec_to_cv_protocol_tool.hpp"
#include "iostream"
#include "refree_protocol_tool.hpp"
#include "send_gimbal_protocol_tool.hpp"

// int main() {
//   SendGimbalBuilder builder;
//   builder.configure(true, GimbalProtocol::AIM_COMMON, false, 45.5, true);
//   //   builder.addTarget(0, 100);

//   //   "Hero",       // 0x01
//   //   "Infantry3",  // 0x02
//   //   "Infantry4",  // 0x03
//   //   "Engineer",   // 0x04
//   //   "Sentry",     // 0x05
//   //   "Outpost",    // 0x06
//   //   "Base"        // 0x07
//   //
//   builder.addTarget(1, 3);
//   builder.addTarget(2, 3);
//   builder.addTarget(3, 1);
//   builder.addTarget(4, 40);
//   builder.addTarget(5, 40);
//   builder.addTarget(6, 40);
//   builder.addTarget(7, 43);

//   auto frame = builder.build();

//   SendGimbalParser parser;

//   if (parser.parse(frame)) {
//     std::cout << "是否巡逻" << parser.is_patrolling() << std::endl;
//     std::cout << "是否启动自瞄" << parser.autoaim_enabled() << std::endl;
//     std::cout << "是否巡逻" << parser.has_reached() << std::endl;
//     std::cout << "角度" << parser.yaw_degrees() << std::endl;
//     // std::cout << "是否巡逻" << parser.targets() << std::endl;
//     auto targets = parser.targets();

//     std::cout << "机器人类型:" << (int)targets[0].type << std::endl;
//     std::cout << "机器人优先级:" << (int)targets[0].priority << std::endl;

//     std::cout << "机器人类型:" << (int)targets[1].type << std::endl;
//     std::cout << "机器人优先级:" << (int)targets[1].priority << std::endl;
//   }
// }

int main() {
  // 构建姿态包
  auto pose_pkt =
      combat_protocol::ProtocolBuilder::build_pose(15.5f,   // pitch
                                                   30.2f,   // yaw
                                                   5.1f,    // roll
                                                   0xABCD,  // 电控时间戳
                                                   true,    // 已发弹
                                                   false    // 右键未按下
      );

  // auto timing_pkt = combat_protocol::ProtocolBuilder::build_timing();
  auto timing_pkt =
      combat_protocol::ProtocolBuilder::build_timing(1, 1, 200, 1, 1, 1, 000);

  // 解析数据包
  combat_protocol::ProtocolParser parser;
  if (parser.parse(timing_pkt)) {
    if (parser.packet_type() == combat_protocol::PacketType::POSE) {
      std::cout << "这是位姿包" << std::endl;

      auto data = parser.pose();
      std::cout << "滚转角: " << data.roll << "°\n"
                << "电控时间: " << data.ec_timestamp << "ms" << std::endl;
    } else {
      std::cout << "这是对时包" << std::endl;
      auto data = parser.timing();

      std::cout << data.bullet_speed << std::endl;
    }
  } else {
    std::cout << "解析失败" << std::endl;
  }
}

// int main() {
//   // gimbal_protocol::ProtocolBuilder builder;
//   // builder.set_status_flags(true, false, true, false, true, false, true);
//   // builder.set_work_mode(gimbal_protocol::WorkMode::MODE_C);
//   // builder.set_precision(0.12f, 0.08f);
//   // builder.set_timestamps(1288, 3222);
//   // builder.set_prediction(15.5f, 30.2f, 28, 18);
//   // auto packet = builder.build();
//   // auto timer = ge
//   // // 解析数据包
//   // gimbal_protocol::ProtocolParser parser;
//   // if (parser.parse(packet)) {
//   //   // auto status = parser.get_status();
//   //   // std::cout << "目标发现: " << status.target_found << "\n"
//   //   //           << "云台转速: " << parser.get_prediction().pitch_speed <<
//   //   "°/s"
//   //   //           << std::endl;
//   //   std::cout << parser.get_timestamps().first << std::endl;
//   //   int s = parser.get_prediction().pitch_speed;
//   //   std::cout << s << std::endl;
//   //   std::cout << parser.get_precision().first << std::endl;
//   //   std::cout << parser.get_precision().second << std::endl;
//   // }
// }