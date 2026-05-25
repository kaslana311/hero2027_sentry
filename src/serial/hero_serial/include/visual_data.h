#ifndef _VISUAL_DATA_H_
#define _VISUAL_DATA_H_

/******************视觉发送，底盘接收*************************************** */
// 控制数据，包含x和y的线速度和角速度，以及小陀螺模式，跟随角度
#include <cstdint>
#pragma pack(push, 1)
typedef struct {
    float x;            // x线速度
    float y;            // y线速度
    float w;            // 角速度
    uint8_t is_tyro;    // 小陀螺模式
    uint16_t virt_yaw;  // 跟随虚拟角度【0~360】单位：0.1度
} Ctrl_info_t;
#pragma pack(pop)

// 哨兵自主决策指令信息子命令id:0x120(机器人间通信)
#pragma pack(push, 1)
// 哨兵自主决策指令信息子命令id:0x120(0x301)
typedef struct {
    uint32_t confirm_revive : 1;  // 0bit, 是否确认复活，0表示不复活，1表示复活
    uint32_t
        confirm_exchange_revive : 1;  // 1bit,
                                      // 确认是否兑换立即复活，0表示不兑换，1表示兑换
    uint32_t
        exchange_ammo_amount : 11;  // 2-12bit, 将要兑换的发弹量值，从0开始递增
    uint32_t
        exchange_ammo_request_count : 4;  // 13-16bit,
                                          // 远程兑换发弹量的请求次数，从0开始递增，每次变化只能是1
    uint32_t
        exchange_health_request_count : 4;  // 17-20bit,
                                            // 远程兑换血量的请求次数，从0开始递增，每次变化只能是1
    uint32_t sentry_status : 2;  // 21-22bit, 哨兵当前姿态
    uint32_t
        energy_mechanism_activate : 1;  // 23bit,
                                        // 是否激活能量机关，0为默认，1表示激活
    uint32_t reserved : 8;              // 24-31bit, 保留位，当前未使用
} sentry_cmd_info_t;
#pragma pack(pop)

// 底盘转发的雷达数据，从视觉接收
#pragma pack(push, 1)
typedef struct {
    uint8_t robot_ID;
    float x;      // 地图坐标x
    float y;      // 地图坐标y
} visual_data_t;  // 雷达数据：需要发送六个地面单位的数据
#pragma pack(pop)

/******************底盘发送，视觉接收*************************************** */
#pragma pack(push, 1)
typedef struct {
    uint8_t game_type : 4;       // 比赛类型
    uint8_t game_progress : 4;   // 当前比赛阶段
    uint16_t stage_remain_time;  // 当前阶段剩余时间
    uint8_t dart_target : 4;  // 当前飞镖靶标，0反应敌方前哨站存活，非0即被摧毁
    uint8_t
        small_energy_state : 2;    // 小能量机关状态 0为默认 1为激活 2为正在激活
    uint8_t big_energy_state : 2;  // 大能量机关状态 0为默认 1为激活 2为正在激活
} Game_info_t;                     // 比赛状态数据
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint16_t ally_1_robot_HP : 2;
    uint16_t ally_2_robot_HP : 2;
    uint16_t ally_3_robot_HP : 2;
    uint16_t ally_4_robot_HP : 2;
    uint16_t reserved : 2;
    uint16_t ally_7_robot_HP : 2;
    uint16_t ally_outpost_HP : 2;
    uint16_t ally_base_HP : 2;
} game_robot_HP_t;  // 比赛机器人血量数据
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint32_t event_type;

} Event_data_t;  // 场地事件数据
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint8_t robot_id;                // 机器人id 可用于进行数据校验
    uint8_t robot_level;             // 机器人等级
    uint16_t remain_HP;              // 剩余血量
    uint16_t max_HP;                 // 血量上限
    uint16_t shooter_cooling_rate;   // 机器人枪口每秒冷却值
    uint16_t shooter_cooling_limit;  // 机器人枪口热量上限
    uint16_t chassis_power_limit;    // 机器人底盘功率上限
} Robot_info_t;                      // 机器人状态数据
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    float x;  // 位置 x 坐标，单位 m
    float y;
    float yaw;  // 本机器人测速模块的朝向，单位：度。正北为 0 度

} Robot_position_t;  // 机器人位置相关数据
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint16_t bullet_remaining_num_17mm;      // 17mm 子弹剩余发射数目
    uint16_t bullet_remaining_num_42mm;      // 42mm 子弹剩余发射数目
    uint16_t coin_remaining_num;             // 剩余金币数量
    uint16_t projectile_allowance_fortress;  // 堡垒上允许发弹量

} Bullet_remaining_t;  // 子弹剩余发射数和剩余金币数量
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
} Ground_robot_position_t;  // 地面机器人位置数据
#pragma pack(pop)

// #pragma pack(push, 1)
// typedef struct  //
// 这里的该信息是指裁判系统反馈的数据，两个字节，和上边那个不一样（具体内容查看手册中id:0x20D）
// {
//     uint32_t sentry_info;
//     uint16_t sentry_info_2;
// } Sentry_info_t;  // 哨兵自主决策反馈数据
// #pragma pack(pop)

/* ID: 0x020D  Byte:  4   哨兵自主决策信息同步，固定以1Hz频率发送  */
#pragma pack(push, 1)
typedef struct {
    uint32_t exchanged_ammo_amount : 11;  // bit0~10,
                                          // 哨兵成功兑换的允许发弹量，开局为0
    uint32_t
        remote_exchange_ammo_count : 4;  // bit11~14,
                                         // 哨兵成功远程兑换允许发弹量的次数，开局为0
    uint32_t
        remote_exchange_health_count : 4;  // bit15~18,
                                           // 哨兵成功远程兑换血量的次数，开局为0
    uint32_t
        free_revive_available : 1;  // bit19, 当前是否可以确认免费复活，1=可以
    uint32_t exchange_revive_available : 1;  // bit20,
                                             // 当前是否可以兑换立即复活，1=可以
    uint32_t exchange_revive_cost : 10;      // bit21~30,
                                         // 当前若兑换立即复活需要花费的金币数
    uint32_t reserved : 1;  // bit31, 保留
} sentry_info_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint16_t disengage_status : 1;            // bit0, 脱战状态
    uint16_t exchange_17mm_amount : 11;       // bit1~11, 17mm 兑换数
    uint16_t sentry_posture : 2;              // bit12~13, 姿态
    uint16_t energy_mechanism_available : 1;  // bit14, 能量机关可激活
    uint16_t reserved : 1;                    // bit15, 保留
} sentry_info_2_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    sentry_info_t sentry_info;
    sentry_info_2_t sentry_info_2;
} ext_sentry_info_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    float target_position_x;  // 目标 x 位置坐标，单位 m
    float target_position_y;
    uint8_t
        commd_keyboard;  // 发送指令时，云台手按下的键盘信息 无按键按下则为 0
    uint8_t
        target_robot_ID;  // 要作用的目标机器人 ID   当发送位置信息时，该项为 0
    uint16_t cmd_source;  // 信息来源 ID

} Robot_command_map_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    float vel_x;
    float vel_y;
} Robot_wheel_odometry_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    bool is_aiming;
} CV_to_Nav_Massage_t;
#pragma pack(pop)

/* 多机通信-对方机器人位置数据 */
#pragma pack(push, 1)
typedef struct {
    uint16_t opponent_hero_position_x;
    uint16_t opponent_hero_position_y;
    uint16_t opponent_engineer_position_x;
    uint16_t opponent_engineer_position_y;
    uint16_t opponent_infantry_3_position_x;
    uint16_t opponent_infantry_3_position_y;
    uint16_t opponent_infantry_4_position_x;
    uint16_t opponent_infantry_4_position_y;
    uint16_t opponent_aerial_position_x;
    uint16_t opponent_aerial_position_y;
    uint16_t opponent_sentry_position_x;
    uint16_t opponent_sentry_position_y;
} com_opponent_position_t;
#pragma pack(pop)

/***********设置的CMDid,发送次数，发送周期********************************* */
#define CMD_ID_VISUAL_CTRL 0x01      // 视觉发送控制指令ID
#define CMD_ID_VISUAL_RADAR 0x02     // 视觉发送雷达数据ID
#define CMD_ID_SENTRY_CMD_INFO 0x03  // 视觉发送哨兵数据ID（视觉发送给底盘）
#define CMD_ID_GAME_INFO 0x04        // 向视觉发送比赛信息
#define CMD_ID_GAME_ROBOT_survivors 0x05  // 向视觉发送机器人血量信息
#define CMD_ID_EVENT_DATA 0x06            // 向视觉发送场地事件信息
#define CMD_ID_ROBOT_INFO 0x07            // 向视觉发送机器人信息（自身信息）
#define CMD_ID_ROBOT_POSITION \
    0x08  // 向视觉发送本机器人位置信息（自身位置信息）
#define CMD_ID_Bullet_remaining 0x09  // 向视觉发送剩余子弹数和剩余金币数量
#define CMD_ID_Ground_robot_position \
    0x0A  // 向视觉发送地面机器人位置信息（自家机器人信息）
#define CMD_ID_Sentry_info \
    0x0B  // 给视觉发送哨兵自主决策反馈数据（裁判系统反馈信息）
#define CMD_ID_Robot_command_map 0x0C
#define CMD_ID_Robot_wheel_odometry \
    0x0D  // 底盘向导航发送的轮式里程计速度数据ID
#define CMD_ID_com_opponent_position 0x0F  // 多机通信-对方机器人位置数据ID

// 设置 发送次数（有限次就写正整数，无限次发送写-1） 和 每几个周期发送一次
#define CMD_SET_VISUAL_CTRL -1, 2      // 视觉发送控制指令
#define CMD_SET_VISUAL_RADAR -1, 2     // 视觉发送雷达数据ID
#define CMD_SET_SENTRY_CMD_INFO -1, 2  // 视觉发送哨兵数据ID（视觉发送给底盘）
#define CMD_SET_GAME_INFO -1, 2        // 向视觉发送比赛信息
#define CMD_SET_GAME_ROBOT_survivors -1, 2  // 向视觉发送机器人血量信息
#define CMD_SET_EVENT_DATA -1, 2            // 向视觉发送场地事件信息
#define CMD_SET_ROBOT_INFO -1, 2            // 向视觉发送机器人信息（自身信息）
#define CMD_SET_ROBOT_POSITION \
    -1, 2  // 向视觉发送本机器人位置信息（自身位置信息）
#define CMD_SET_Bullet_remaining -1, 2  // 向视觉发送剩余子弹数和剩余金币数量
#define CMD_SET_Ground_robot_position \
    -1, 2  // 向视觉发送地面机器人位置信息（自家机器人信息）
#define CMD_SET_Sentry_info \
    -1, 2  // 给视觉发送哨兵自主决策反馈数据（裁判系统反馈信息）
#define CMD_SET_Robot_command_map \
    -1, 1000  // 给视觉发送云台手发送给哨兵的点位信息（云台手命令哨兵前往的位置）

#endif
