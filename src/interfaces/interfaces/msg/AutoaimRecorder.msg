# time_pair
int64 pose_mcu_time
float64 pose_ptz_yaw
float64 pose_ptz_pitch
float64 pose_ptz_roll
int8 mode
float64 bullet_speed
int32 shoot_delay
int8 enemy_color
int8 enemy_number
bool is_right_clicked
bool fired
bool request_fire

# armor_detector.run
int8 track_test
int8 armor_num1
int8 armor_num2
int8 armor_num3
int8 dir
bool is_searching
bool armor_found
bool target_searched
bool flag_track_car_failed
bool is_big_armor
bool is_balance_infantry
int8 next_state
int32 judge_armor_switch
bool if_two_armor



# armor_getCoord
float64 pnp_x
float64 pnp_y
float64 pnp_z
float64 pnp_z_x
float64 pnp_z_y
float64 pnp_z_z # 装甲板法向量
float64 target_ptz_x
float64 target_ptz_y
float64 target_ptz_z
float64 target_land_x
float64 target_land_y
float64 target_land_z
float64 target_xy_plane_distance
float64 target_xyz_distance
float64 target_yaw
float64 target_pitch
float64 delta_time


# armor_predictor.update
int32 kf_reset
bool flag_switch_armor
float64 descriptior_posterior_yaw
float64 descriptior_yaw_velocity
float64 descriptior_yaw_acceleration
float64 descriptior_posterior_xy_plane_distance
float64 descriptior_xy_plane_distance_velocity
float64 descriptior_xy_plane_distance_acceleration
float64 descriptior_posterior_pitch
float64 descriptior_pitch_velocity

int32 clearmode
int32 failedmode
float64 ratio
int32 armor_lost_cnt
int32 car_lost_cnt
float64 period
float64 curr_yaw
float64 last_yaw
float64 delta_yaw_abs

# ekf
float64 ekf_x_center
float64 ekf_y_center
float64 ekf_z_center
float64 ekf_v_x_center
float64 ekf_v_y_center
float64 ekf_v_z_center
float64 ekf_theta
float64 ekf_omega
float64 ekf_r



# anti_gyror.update
float64 t_ratio
float64 t_ratio2
float64 del_t_ratio
float64 delta_delta_time
float64 predict_time
int32 antigyro_test
float64 left_switch_yaw
float64 right_switch_yaw

# setArmorPredictdata
bool is_antigyro
## antigyro
## armor

builtin_interfaces/Time time



# dynamic_hit_outpost
float64 ayaw
float64 byaw
int32 switch_num
float64 target_clock
int32 length_ofsaveyaw
float64 center_yaw
float64 center_z
float64 center_dis
float64 center_time0
float64 center_time1
float64 center_time2
float64 center_time3
int32 title_switch
float64 del_time_three0
float64 del_time_three1
float64 del_time_three2


Tovision tov


