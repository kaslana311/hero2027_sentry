// 裁判系统信息的构建与解析工具
#pragma once
#include <algorithm>
#include <cstdint>
#include <vector>

namespace refree_protocol {
#pragma pack(push, 1)
struct ProtocolHeader {
    // 固定帧头 (0-3)
    uint32_t frame_header = 0x7F800000;

    // 比赛信息 (4-6)
    uint8_t competition_info;       // 4: [0-3]比赛类型, [4-7]比赛进程
    uint16_t stage_remaining_time;  // 5-6: 阶段剩余时间

    // 红方血量信息 (7-22)
    uint16_t red_hero;       // 7-8
    uint16_t red_engineer;   // 9-10
    uint16_t red_infantry3;  // 11-12
    uint16_t red_infantry4;  // 13-14
    uint16_t red_reserved1;  // 15-16
    uint16_t red_sentry;     // 17-18
    uint16_t red_outpost;    // 19-20
    uint16_t red_base;       // 21-22

    // 蓝方血量信息 (23-38)
    uint16_t blue_hero;       // 23-24
    uint16_t blue_engineer;   // 25-26
    uint16_t blue_infantry3;  // 27-28
    uint16_t blue_infantry4;  // 29-30
    uint16_t blue_reserved1;  // 31-32
    uint16_t blue_sentry;     // 33-34
    uint16_t blue_outpost;    // 35-36
    uint16_t blue_base;       // 37-38

    // 状态位字段 (39-42)
    uint32_t status_bits;

    // 本机信息 (43-54)
    uint8_t robot_id;             // 43
    uint8_t robot_level;          // 44
    uint16_t current_hp;          // 45-46
    uint16_t max_hp;              // 47-48
    uint16_t cooling_rate;        // 49-50
    uint16_t heat_limit;          // 51-52
    uint16_t chassis_heat_limit;  // 53-54

    // 位置信息 (55-72)
    float position_x;         // 55-58
    float position_y;         // 59-62
    float orientation;        // 63-66
    uint16_t bullet_17mm;     // 67-68
    uint16_t bullet_42mm;     // 69-70
    uint16_t remaining_gold;  // 71-72

    // 队友位置 (73-104)
    float hero_x, hero_y;            // 73-80
    float engineer_x, engineer_y;    // 81-88
    float infantry3_x, infantry3_y;  // 89-96
    float infantry4_x, infantry4_y;  // 97-104

    // 哨兵状态 (105-108)
    uint32_t sentry_status;

    // 附加状态 (109-110)
    uint16_t additional_status;
};
#pragma pack(pop)

class ProtocolBuilder {
   public:
    ProtocolBuilder() { reset(); }

    void reset() {
        header_ = ProtocolHeader();
        header_.frame_header = 0x7F800000;
    }

    // 设置比赛信息
    void set_competition(uint8_t type, uint8_t stage) {
        header_.competition_info = (stage << 4) | (type & 0x0F);
    }

    // 设置阶段剩余时间
    void set_stage_time(uint16_t seconds) {
        header_.stage_remaining_time = seconds;
    }

    // 设置红方血量
    void set_red_hp(uint16_t hero, uint16_t engineer, uint16_t infantry3,
                    uint16_t infantry4, uint16_t sentry, uint16_t outpost,
                    uint16_t base) {
        header_.red_hero = hero;
        header_.red_engineer = engineer;
        header_.red_infantry3 = infantry3;
        header_.red_infantry4 = infantry4;
        header_.red_sentry = sentry;
        header_.red_outpost = outpost;
        header_.red_base = base;
    }

    // 设置蓝方血量
    void set_blue_hp(uint16_t hero, uint16_t engineer, uint16_t infantry3,
                     uint16_t infantry4, uint16_t sentry, uint16_t outpost,
                     uint16_t base) {
        header_.blue_hero = hero;
        header_.blue_engineer = engineer;
        header_.blue_infantry3 = infantry3;
        header_.blue_infantry4 = infantry4;
        header_.blue_sentry = sentry;
        header_.blue_outpost = outpost;
        header_.blue_base = base;
    }

    // 设置状态位字段
    void set_status_bits(bool supply_non_overlap, bool supply_overlap,
                         bool supply_occupied, uint8_t energy_activation,
                         uint8_t highland_status, uint8_t trapezoid_status,
                         uint16_t dart_hit_time, uint8_t dart_target,
                         uint8_t center_control) {
        uint32_t bits = 0;
        bits |= supply_non_overlap ? 0x01 : 0x00;
        bits |= supply_overlap ? 0x02 : 0x00;
        bits |= supply_occupied ? 0x04 : 0x00;
        bits |= (energy_activation & 0x07) << 3;
        bits |= (highland_status & 0x03) << 6;
        bits |= (trapezoid_status & 0x03) << 8;
        bits |= (dart_hit_time & 0x01FF) << 9;
        bits |= (dart_target & 0x07) << 18;
        bits |= (center_control & 0x03) << 21;
        header_.status_bits = bits;
    }

    // 设置本机信息
    void set_robot_info(uint8_t id, uint8_t level, uint16_t current_hp,
                        uint16_t max_hp, uint16_t cooling_rate,
                        uint16_t heat_limit, uint16_t chassis_heat_limit) {
        header_.robot_id = id;
        header_.robot_level = level;
        header_.current_hp = current_hp;
        header_.max_hp = max_hp;
        header_.cooling_rate = cooling_rate;
        header_.heat_limit = heat_limit;
        header_.chassis_heat_limit = chassis_heat_limit;
    }

    // 设置位置信息
    void set_position(float x, float y, float orientation, uint16_t bullet_17mm,
                      uint16_t bullet_42mm, uint16_t remaining_gold) {
        header_.position_x = x;
        header_.position_y = y;
        header_.orientation = orientation;
        header_.bullet_17mm = bullet_17mm;
        header_.bullet_42mm = bullet_42mm;
        header_.remaining_gold = remaining_gold;
    }

    // 设置队友位置
    void set_teammate_positions(float hero_x, float hero_y, float engineer_x,
                                float engineer_y, float infantry3_x,
                                float infantry3_y, float infantry4_x,
                                float infantry4_y) {
        header_.hero_x = hero_x;
        header_.hero_y = hero_y;
        header_.engineer_x = engineer_x;
        header_.engineer_y = engineer_y;
        header_.infantry3_x = infantry3_x;
        header_.infantry3_y = infantry3_y;
        header_.infantry4_x = infantry4_x;
        header_.infantry4_y = infantry4_y;
    }

    // 设置哨兵状态
    void set_sentry_status(uint16_t bullet_allowance,
                           uint8_t remote_bullet_count, uint8_t remote_hp_count,
                           bool confirm_revive, bool immediate_revive,
                           uint16_t revive_cost) {
        uint32_t status = 0;
        status |= bullet_allowance & 0x07FF;
        status |= (remote_bullet_count & 0x0F) << 11;
        status |= (remote_hp_count & 0x0F) << 15;
        status |= confirm_revive ? (0x01 << 19) : 0;
        status |= immediate_revive ? (0x01 << 20) : 0;
        status |= (revive_cost & 0x03FF) << 21;
        header_.sentry_status = status;
    }

    // 设置附加状态（完整实现）
    void set_additional_status(bool status_bit0, uint16_t status_bits1_11,
                               uint8_t status_bits12_15) {
        uint16_t status = 0;
        status |= status_bit0 ? 0x0001 : 0x0000;
        status |= (status_bits1_11 & 0x07FF) << 1;
        status |= (status_bits12_15 & 0x0F) << 12;
        header_.additional_status = status;
    }

    // 构建数据包
    std::vector<uint8_t> build() {
        std::vector<uint8_t> packet;
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&header_);
        packet.insert(packet.end(), ptr, ptr + sizeof(ProtocolHeader));

        // 计算校验和,这里不对，校验和不包括前四位，这里一直没用所以没发现
        uint8_t checksum = 0;
        for (auto b : packet) checksum += b;
        packet.push_back(checksum);

        return packet;
    }

   private:
    ProtocolHeader header_;
};

class ProtocolParser {
   public:
    bool parse(const std::vector<uint8_t>& data) {
        if (data.size() != 112) return false;
        std::cout << "数据足够" << std::endl;
        if (!verify_checksum(data)) return false;
        std::cout << "校验和正确" << std::endl;

        const ProtocolHeader* header =
            reinterpret_cast<const ProtocolHeader*>(data.data());
        if (header->frame_header != 0x7F800000) return false;
        std::cout << "解析底盘数据中" << std::endl;

        // 解析比赛信息
        competition_type_ = header->competition_info & 0x0F;
        competition_stage_ = (header->competition_info >> 4) & 0x0F;
        stage_time_ = header->stage_remaining_time;

        // 解析红方血量
        red_hero_ = header->red_hero;
        red_engineer_ = header->red_engineer;
        red_infantry3_ = header->red_infantry3;
        red_infantry4_ = header->red_infantry4;
        red_sentry_ = header->red_sentry;
        red_outpost_ = header->red_outpost;
        red_base_ = header->red_base;

        // 解析蓝方血量
        blue_hero_ = header->blue_hero;
        blue_engineer_ = header->blue_engineer;
        blue_infantry3_ = header->blue_infantry3;
        blue_infantry4_ = header->blue_infantry4;
        blue_sentry_ = header->blue_sentry;
        blue_outpost_ = header->blue_outpost;
        blue_base_ = header->blue_base;

        // 解析状态位字段
        status_supply_non_overlap_ = header->status_bits & 0x01;
        status_supply_overlap_ = (header->status_bits >> 1) & 0x01;
        status_supply_occupied_ = (header->status_bits >> 2) & 0x01;
        energy_activation_ = (header->status_bits >> 3) & 0x07;
        highland_status_ = (header->status_bits >> 6) & 0x03;
        trapezoid_status_ = (header->status_bits >> 8) & 0x03;
        dart_hit_time_ = (header->status_bits >> 9) & 0x01FF;
        dart_target_ = (header->status_bits >> 18) & 0x07;
        center_control_ = (header->status_bits >> 21) & 0x03;

        // 解析本机信息
        robot_id_ = header->robot_id;
        robot_level_ = header->robot_level;
        current_hp_ = header->current_hp;
        max_hp_ = header->max_hp;
        cooling_rate_ = header->cooling_rate;
        heat_limit_ = header->heat_limit;
        chassis_heat_limit_ = header->chassis_heat_limit;

        // 解析位置信息
        position_x_ = header->position_x;
        position_y_ = header->position_y;
        orientation_ = header->orientation;
        bullet_17mm_ = header->bullet_17mm;
        bullet_42mm_ = header->bullet_42mm;
        remaining_gold_ = header->remaining_gold;

        // 解析队友位置
        hero_x_ = header->hero_x;
        hero_y_ = header->hero_y;
        engineer_x_ = header->engineer_x;
        engineer_y_ = header->engineer_y;
        infantry3_x_ = header->infantry3_x;
        infantry3_y_ = header->infantry3_y;
        infantry4_x_ = header->infantry4_x;
        infantry4_y_ = header->infantry4_y;

        // 解析哨兵状态
        sentry_bullet_allowance_ = header->sentry_status & 0x07FF;
        sentry_remote_bullet_count_ = (header->sentry_status >> 11) & 0x0F;
        sentry_remote_hp_count_ = (header->sentry_status >> 15) & 0x0F;
        sentry_confirm_revive_ = (header->sentry_status >> 19) & 0x01;
        sentry_immediate_revive_ = (header->sentry_status >> 20) & 0x01;
        sentry_revive_cost_ = (header->sentry_status >> 21) & 0x03FF;

        // 解析附加状态（完整实现）
        additional_status_bit0_ = header->additional_status & 0x0001;
        additional_status_bits1_11_ = (header->additional_status >> 1) & 0x07FF;
        additional_status_bits12_15_ =
            (header->additional_status >> 12) & 0x000F;

        return true;
    }

    // 校验和验证
    bool verify_checksum(const std::vector<uint8_t>& data) {
        uint8_t sum = 0;
        for (size_t i = 4; i < data.size() - 1; ++i) sum += data[i];
        return sum == data.back();
    }

    // 完整获取接口
    // 比赛信息
    /*
    • 1：RoboMaster 机甲大师超级对抗赛
    • 2：RoboMaster 机甲大师高校单项赛
    • 3：ICRA RoboMaster 高校人工智能挑战赛
    • 4：RoboMaster 机甲大师高校联盟赛 3V3 对抗
    • 5：RoboMaster 机甲大师高校联盟赛步兵对抗
    */
    uint8_t competition_type() const { return competition_type_; }
    /*
    • 0：未开始比赛
    • 1：准备阶段
    • 2：十五秒裁判系统自检阶段
    • 3：五秒倒计时
    • 4：比赛中
    • 5：比赛结算中
    */
    uint8_t competition_stage() const { return competition_stage_; }
    uint16_t stage_remaining_time() const { return stage_time_; }

    // 红方状态
    uint16_t red_hero_hp() const { return red_hero_; }
    uint16_t red_engineer_hp() const { return red_engineer_; }
    uint16_t red_infantry3_hp() const { return red_infantry3_; }
    uint16_t red_infantry4_hp() const { return red_infantry4_; }
    uint16_t red_sentry_hp() const { return red_sentry_; }
    uint16_t red_outpost_hp() const { return red_outpost_; }
    uint16_t red_base_hp() const { return red_base_; }

    // 蓝方状态
    uint16_t blue_hero_hp() const { return blue_hero_; }
    uint16_t blue_engineer_hp() const { return blue_engineer_; }
    uint16_t blue_infantry3_hp() const { return blue_infantry3_; }
    uint16_t blue_infantry4_hp() const { return blue_infantry4_; }
    uint16_t blue_sentry_hp() const { return blue_sentry_; }
    uint16_t blue_outpost_hp() const { return blue_outpost_; }
    uint16_t blue_base_hp() const { return blue_base_; }

    // 战场状态
    bool supply_non_overlap() const { return status_supply_non_overlap_; }
    bool supply_overlap() const { return status_supply_overlap_; }
    bool supply_occupied() const { return status_supply_occupied_; }
    uint8_t energy_activation() const { return energy_activation_; }
    uint8_t highland_status() const { return highland_status_; }
    uint8_t trapezoid_status() const { return trapezoid_status_; }
    uint16_t dart_hit_time() const { return dart_hit_time_; }
    uint8_t dart_target() const { return dart_target_; }
    uint8_t center_control() const { return center_control_; }

    // 本机状态
    uint8_t robot_id() const { return robot_id_; }
    uint8_t robot_level() const { return robot_level_; }
    uint16_t current_hp() const { return current_hp_; }
    uint16_t max_hp() const { return max_hp_; }
    uint16_t cooling_rate() const { return cooling_rate_; }
    uint16_t heat_limit() const { return heat_limit_; }
    uint16_t chassis_heat_limit() const { return chassis_heat_limit_; }

    // 位置信息
    float position_x() const { return position_x_; }
    float position_y() const { return position_y_; }
    float orientation() const { return orientation_; }
    // 17mm 弹丸允许发弹量
    uint16_t bullet_17mm() const { return bullet_17mm_; }
    // 42mm弹丸允许发弹量
    uint16_t bullet_42mm() const { return bullet_42mm_; }
    // 剩余金币数量
    uint16_t remaining_gold() const { return remaining_gold_; }

    // 队友位置
    std::pair<float, float> hero_position() const { return {hero_x_, hero_y_}; }
    std::pair<float, float> engineer_position() const {
        return {engineer_x_, engineer_y_};
    }
    std::pair<float, float> infantry3_position() const {
        return {infantry3_x_, infantry3_y_};
    }
    std::pair<float, float> infantry4_position() const {
        return {infantry4_x_, infantry4_y_};
    }

    // 哨兵状态
    uint16_t sentry_bullet_allowance() const {
        return sentry_bullet_allowance_;
    }
    uint8_t sentry_remote_bullet_count() const {
        return sentry_remote_bullet_count_;
    }
    uint8_t sentry_remote_hp_count() const { return sentry_remote_hp_count_; }
    bool sentry_confirm_revive() const { return sentry_confirm_revive_; }
    bool sentry_immediate_revive() const { return sentry_immediate_revive_; }
    uint16_t sentry_revive_cost() const { return sentry_revive_cost_; }

    // 附加状态（补全接口）
    bool additional_status_bit0() const { return additional_status_bit0_; }
    uint16_t additional_status_bits1_11() const {
        return additional_status_bits1_11_;
    }
    uint8_t additional_status_bits12_15() const {
        return additional_status_bits12_15_;
    }

   private:
    // 解析状态存储（补全所有成员变量）
    // 比赛信息
    uint8_t competition_type_;
    uint8_t competition_stage_;
    uint16_t stage_time_;

    // 红方血量
    uint16_t red_hero_;
    uint16_t red_engineer_;
    uint16_t red_infantry3_;
    uint16_t red_infantry4_;
    uint16_t red_sentry_;
    uint16_t red_outpost_;
    uint16_t red_base_;

    // 蓝方血量
    uint16_t blue_hero_;
    uint16_t blue_engineer_;
    uint16_t blue_infantry3_;
    uint16_t blue_infantry4_;
    uint16_t blue_sentry_;
    uint16_t blue_outpost_;
    uint16_t blue_base_;

    // 战场状态
    bool status_supply_non_overlap_;
    bool status_supply_overlap_;
    bool status_supply_occupied_;
    uint8_t energy_activation_;
    uint8_t highland_status_;
    uint8_t trapezoid_status_;
    uint16_t dart_hit_time_;
    uint8_t dart_target_;
    uint8_t center_control_;

    // 本机状态
    uint8_t robot_id_;
    uint8_t robot_level_;
    uint16_t current_hp_;
    uint16_t max_hp_;
    uint16_t cooling_rate_;
    uint16_t heat_limit_;
    uint16_t chassis_heat_limit_;

    // 位置信息
    float position_x_;
    float position_y_;
    float orientation_;
    uint16_t bullet_17mm_;
    uint16_t bullet_42mm_;
    uint16_t remaining_gold_;

    // 队友位置
    float hero_x_, hero_y_;
    float engineer_x_, engineer_y_;
    float infantry3_x_, infantry3_y_;
    float infantry4_x_, infantry4_y_;

    // 哨兵状态
    uint16_t sentry_bullet_allowance_;
    uint8_t sentry_remote_bullet_count_;
    uint8_t sentry_remote_hp_count_;
    bool sentry_confirm_revive_;
    bool sentry_immediate_revive_;
    uint16_t sentry_revive_cost_;

    // 附加状态
    // bit 0：哨兵当前是否处于脱战状态，处于脱战状态时为 1，否则为 0。
    // bit 1-11：队伍 17mm 允许发弹量的剩余可兑换数。
    // bit 12-15：保留

    bool additional_status_bit0_;
    uint16_t additional_status_bits1_11_;
    uint8_t additional_status_bits12_15_;
};
}  // namespace refree_protocol