// #include <../include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp>
#include "Estimator.h"

#include <nav_msgs/msg/odometry.hpp>

// ======================= 轮速计 (Wheel Odometry) 相关外部变量声明
// ======================= 轮速计外参平移向量 (从 parameters.cpp 引入) 物理意义:
// 轮速计/底盘中心 在 IMU 坐标系下的位置
extern std::vector<double> extrinT_wheel;
// 轮速计外参旋转矩阵 (从 parameters.cpp 引入)
// 物理意义: 将轮速计坐标系下的向量变换到 IMU 坐标系
extern std::vector<double> extrinR_wheel;
// 轮速计速度测量噪声协方差 (从 parameters.cpp 引入)
extern double wheel_vel_cov;
// 是否启用轮速计融合 (从 parameters.cpp 引入)
extern bool wheel_en;

// ======================= 轮速计数据容器 (供 laserMapping.cpp 填充)
// ======================= 当前帧需要处理的轮速计数据 在每次 EKF update 之前，由
// laserMapping.cpp 将 Measures.wheel_odom 拷贝到此处 h_model_output
// 函数会遍历此容器构建轮速残差
std::deque<nav_msgs::msg::Odometry> current_wheel_data;

PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
std::vector<int> time_seq;
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI(10000, 1));
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI(10000, 1));
std::vector<V3D> pbody_list;
std::vector<PointVector> Nearest_Points;
std::shared_ptr<IVoxType> ivox_ = nullptr;  // localmap in ivox
std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
bool point_selected_surf[100000] = {0};
std::vector<M3D> crossmat_list;
int effct_feat_num = 0;
int k = 0;
int idx = -1;
esekfom::esekf<state_input, 24, input_ikfom> kf_input;
esekfom::esekf<state_output, 30, input_ikfom> kf_output;
input_ikfom input_in;
V3D angvel_avr, acc_avr, acc_avr_norm;
int feats_down_size = 0;
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);
double G_m_s2 = 9.81;

Eigen::Matrix<double, 24, 24> process_noise_cov_input() {
    Eigen::Matrix<double, 24, 24> cov;
    cov.setZero();
    cov.block<3, 3>(3, 3).diagonal() << gyr_cov_input, gyr_cov_input,
        gyr_cov_input;
    cov.block<3, 3>(12, 12).diagonal() << acc_cov_input, acc_cov_input,
        acc_cov_input;
    cov.block<3, 3>(15, 15).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
    cov.block<3, 3>(18, 18).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
    // MTK::get_cov<process_noise_input>::type cov =
    // MTK::get_cov<process_noise_input>::type::Zero();
    // MTK::setDiagonal<process_noise_input, vect3, 0>(cov,
    // &process_noise_input::ng, gyr_cov_input);// 0.03
    // MTK::setDiagonal<process_noise_input, vect3, 3>(cov,
    // &process_noise_input::na, acc_cov_input); // *dt 0.01 0.01 * dt * dt 0.05
    // MTK::setDiagonal<process_noise_input, vect3, 6>(cov,
    // &process_noise_input::nbg, b_gyr_cov); // *dt 0.00001 0.00001 * dt *dt
    // 0.3 //0.001 0.0001 0.01 MTK::setDiagonal<process_noise_input, vect3,
    // 9>(cov, &process_noise_input::nba, b_acc_cov);   //0.001 0.05 0.0001/out
    // 0.01
    return cov;
}

Eigen::Matrix<double, 30, 30> process_noise_cov_output() {
    Eigen::Matrix<double, 30, 30> cov;
    cov.setZero();
    cov.block<3, 3>(12, 12).diagonal() << vel_cov, vel_cov, vel_cov;
    cov.block<3, 3>(15, 15).diagonal() << gyr_cov_output, gyr_cov_output,
        gyr_cov_output;
    cov.block<3, 3>(18, 18).diagonal() << acc_cov_output, acc_cov_output,
        acc_cov_output;
    cov.block<3, 3>(24, 24).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
    cov.block<3, 3>(27, 27).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
    return cov;
}

Eigen::Matrix<double, 24, 1> get_f_input(state_input& s,
                                         const input_ikfom& in) {
    Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
    vect3 omega;
    in.gyro.boxminus(omega, s.bg);
    vect3 a_inertial = s.rot * (in.acc - s.ba);  // .normalized()
    for (int i = 0; i < 3; i++) {
        res(i) = s.vel[i];
        res(i + 3) = omega[i];
        res(i + 12) = a_inertial[i] + s.gravity[i];
    }
    return res;
}

// 运动方程：惯性导航解算
Eigen::Matrix<double, 30, 1> get_f_output(state_output& s,
                                          const input_ikfom& in) {
    Eigen::Matrix<double, 30, 1> res = Eigen::Matrix<double, 30, 1>::Zero();
    vect3 a_inertial = s.rot * s.acc;  // .normalized()
    for (int i = 0; i < 3; i++) {
        res(i) = s.vel[i];      // 位置的变化率 = 速度
        res(i + 3) = s.omg[i];  // 角度的变化率 = 角速度
        // 速度的变化率 = 加速度 + 重力
        res(i + 12) = a_inertial[i] + s.gravity[i];
    }
    return res;
}

Eigen::Matrix<double, 24, 24> df_dx_input(state_input& s,
                                          const input_ikfom& in) {
    Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
    cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
    vect3 acc_;
    in.acc.boxminus(acc_, s.ba);
    vect3 omega;
    in.gyro.boxminus(omega, s.bg);
    cov.template block<3, 3>(12, 3) =
        -s.rot * MTK::hat(acc_);  // .normalized().toRotationMatrix()
    cov.template block<3, 3>(12, 18) =
        -s.rot;  //.normalized().toRotationMatrix();
    // Eigen::Matrix<state_ikfom::scalar, 2, 1> vec =
    // Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
    // Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
    // s.S2_Mx(grav_matrix, vec, 21);
    cov.template block<3, 3>(12, 21) =
        Eigen::Matrix3d::Identity();  // grav_matrix;
    cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
    return cov;
}

// 误差传播矩阵：描述误差随惯性导航传播而扩大的递推逻辑
Eigen::Matrix<double, 30, 30> df_dx_output(state_output& s,
                                           const input_ikfom& in) {
    // 30维状态矩阵
    Eigen::Matrix<double, 30, 30> cov = Eigen::Matrix<double, 30, 30>::Zero();
    // 1. 位置误差 受 速度误差 的影响
    // 位置 = 速度 * 时间。所以位置对速度的导数是 I (经过积分后就是 dt)
    cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
    // 2. 速度误差 受 姿态误差 的影响
    cov.template block<3, 3>(12, 3) =
        -s.rot * MTK::hat(s.acc);  // .normalized().toRotationMatrix()
    // 3. 速度误差 受 加速度误差 的影响
    cov.template block<3, 3>(12, 18) =
        s.rot;  //.normalized().toRotationMatrix();
    // Eigen::Matrix<state_ikfom::scalar, 2, 1> vec =
    // Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
    // Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
    // s.S2_Mx(grav_matrix, vec, 21);
    cov.template block<3, 3>(12, 21) =
        Eigen::Matrix3d::Identity();  // grav_matrix;
    cov.template block<3, 3>(3, 15) = Eigen::Matrix3d::Identity();
    return cov;
}

void h_model_input(state_input& s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R,
                   esekfom::dyn_share_modified<double>& ekfom_data) {
    bool match_in_map = false;
    VF(4) pabcd;
    pabcd.setZero();
    normvec->resize(time_seq[k]);
    int effect_num_k = 0;
    for (int j = 0; j < time_seq[k]; j++) {
        PointType& point_body_j = feats_down_body->points[idx + j + 1];
        PointType& point_world_j = feats_down_world->points[idx + j + 1];
        pointBodyToWorld(&point_body_j, &point_world_j);
        V3D p_body = pbody_list[idx + j + 1];
        double p_norm = p_body.norm();
        V3D p_world;
        p_world << point_world_j.x, point_world_j.y, point_world_j.z;
        {
            auto& points_near = Nearest_Points[idx + j + 1];
            ivox_->GetClosestPoint(point_world_j, points_near,
                                   NUM_MATCH_POINTS);  //
            if ((points_near.size() <
                 NUM_MATCH_POINTS))  // || pointSearchSqDis[NUM_MATCH_POINTS -
                                     // 1] > 5) // 5)
            {
                point_selected_surf[idx + j + 1] = false;
            } else {
                point_selected_surf[idx + j + 1] = false;
                if (esti_plane(pabcd, points_near, plane_thr))  //(planeValid)
                {
                    float pd2 = fabs(pabcd(0) * point_world_j.x +
                                     pabcd(1) * point_world_j.y +
                                     pabcd(2) * point_world_j.z + pabcd(3));
                    // V3D norm_vec;
                    // M3D Rpf, pf;
                    // pf = crossmat_list[idx+j+1];
                    // // pf << SKEW_SYM_MATRX(p_body);
                    // Rpf = s.rot * pf;
                    // norm_vec << pabcd(0), pabcd(1), pabcd(2);
                    // double noise_state = norm_vec.transpose() *
                    // (cov_p+Rpf*cov_R*Rpf.transpose())  * norm_vec +
                    // sqrt(p_norm) * 0.001;
                    // // if (p_norm > match_s * pd2 * pd2)
                    // double epsilon = pd2 / sqrt(noise_state);
                    // // std::cout << "check epsilon:" << epsilon << '\n';
                    // double weight = 1.0; // epsilon / sqrt(epsilon *
                    // epsilon+1); if (epsilon > 1.0)
                    // {
                    // 	weight = sqrt(2 * epsilon - 1) / epsilon;
                    // 	pabcd(0) = weight * pabcd(0);
                    // 	pabcd(1) = weight * pabcd(1);
                    // 	pabcd(2) = weight * pabcd(2);
                    // 	pabcd(3) = weight * pabcd(3);
                    // }
                    if (p_norm > match_s * pd2 * pd2) {
                        point_selected_surf[idx + j + 1] = true;
                        normvec->points[j].x = pabcd(0);
                        normvec->points[j].y = pabcd(1);
                        normvec->points[j].z = pabcd(2);
                        normvec->points[j].intensity = pabcd(3);
                        effect_num_k++;
                    }
                }
            }
        }
    }
    if (effect_num_k == 0) {
        ekfom_data.valid = false;
        return;
    }
    ekfom_data.M_Noise = laser_point_cov;
    ekfom_data.h_x.resize(effect_num_k, 12);
    ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 12);
    ekfom_data.z.resize(effect_num_k);
    int m = 0;

    for (int j = 0; j < time_seq[k]; j++) {
        // ekfom_data.converge = false;
        if (point_selected_surf[idx + j + 1]) {
            V3D norm_vec(normvec->points[j].x, normvec->points[j].y,
                         normvec->points[j].z);

            if (extrinsic_est_en) {
                V3D p_body = pbody_list[idx + j + 1];
                M3D p_crossmat, p_imu_crossmat;
                p_crossmat << SKEW_SYM_MATRX(p_body);
                V3D point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
                p_imu_crossmat << SKEW_SYM_MATRX(point_imu);
                V3D C(s.rot.transpose() * norm_vec);
                V3D A(p_imu_crossmat * C);
                V3D B(p_crossmat * s.offset_R_L_I.transpose() * C);
                ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1),
                    norm_vec(2), VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B),
                    VEC_FROM_ARRAY(C);
            } else {
                M3D point_crossmat = crossmat_list[idx + j + 1];
                V3D C(s.rot.transpose() *
                      norm_vec);  // conjugate().normalized()
                V3D A(point_crossmat * C);
                ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1),
                    norm_vec(2), VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0;
            }
            ekfom_data.z(m) =
                -norm_vec(0) * feats_down_world->points[idx + j + 1].x -
                norm_vec(1) * feats_down_world->points[idx + j + 1].y -
                norm_vec(2) * feats_down_world->points[idx + j + 1].z -
                normvec->points[j].intensity;

            m++;
        }
    }
    effct_feat_num += effect_num_k;
}

// 基于点云与地图匹配构建观测模型
void h_model_output(state_output& s, Eigen::Matrix3d cov_p,
                    Eigen::Matrix3d cov_R,
                    esekfom::dyn_share_modified<double>& ekfom_data) {
    bool match_in_map = false;
    VF(4) pabcd;
    pabcd.setZero();
    normvec->resize(time_seq[k]);
    int effect_num_k = 0;
    for (int j = 0; j < time_seq[k]; j++) {
        // 1. 我们假设当前 KF 预测的状态是对的，先把雷达点变到世界坐标系下。
        PointType& point_body_j = feats_down_body->points[idx + j + 1];
        PointType& point_world_j = feats_down_world->points[idx + j + 1];
        pointBodyToWorld(&point_body_j, &point_world_j);
        V3D p_body = pbody_list[idx + j + 1];
        double p_norm = p_body.norm();
        V3D p_world;
        p_world << point_world_j.x, point_world_j.y, point_world_j.z;
        {
            auto& points_near = Nearest_Points[idx + j + 1];

            // 2. 在地图里找邻居，拟合平面
            ivox_->GetClosestPoint(point_world_j, points_near,
                                   NUM_MATCH_POINTS);  //

            if ((points_near.size() < NUM_MATCH_POINTS))
            // pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5)
            {
                point_selected_surf[idx + j + 1] = false;
            } else {
                point_selected_surf[idx + j + 1] = false;
                if (esti_plane(pabcd, points_near, plane_thr))  //(planeValid)
                {
                    float pd2 = fabs(pabcd(0) * point_world_j.x +
                                     pabcd(1) * point_world_j.y +
                                     pabcd(2) * point_world_j.z + pabcd(3));
                    // V3D norm_vec;
                    // M3D Rpf, pf;
                    // pf = crossmat_list[idx+j+1];
                    // // pf << SKEW_SYM_MATRX(p_body);
                    // Rpf = s.rot * pf;
                    // norm_vec << pabcd(0), pabcd(1), pabcd(2);
                    // double noise_state = norm_vec.transpose() *
                    // (cov_p+Rpf*cov_R*Rpf.transpose())  * norm_vec +
                    // sqrt(p_norm) * 0.001;
                    // // if (p_norm > match_s * pd2 * pd2)
                    // double epsilon = pd2 / sqrt(noise_state);
                    // double weight = 1.0; // epsilon / sqrt(epsilon *
                    // epsilon+1); if (epsilon > 1.0)
                    // {
                    // 	weight = sqrt(2 * epsilon - 1) / epsilon;
                    // 	pabcd(0) = weight * pabcd(0);
                    // 	pabcd(1) = weight * pabcd(1);
                    // 	pabcd(2) = weight * pabcd(2);
                    // 	pabcd(3) = weight * pabcd(3);
                    // }
                    if (p_norm > match_s * pd2 * pd2) {
                        // 拟合平面 Ax + By + Cz + D = 0 成功且满足距离条件，
                        // 认为该点是有效的平面点，纳入残差计算
                        point_selected_surf[idx + j + 1] = true;
                        normvec->points[j].x = pabcd(0);
                        normvec->points[j].y = pabcd(1);
                        normvec->points[j].z = pabcd(2);
                        normvec->points[j].intensity = pabcd(3);
                        effect_num_k++;
                    }
                }
            }
        }
    }
    if (effect_num_k == 0) {
        ekfom_data.valid = false;
        return;
    }
    ekfom_data.M_Noise = laser_point_cov;
    ekfom_data.h_x.resize(effect_num_k, 12);
    // 12维：Pos(3) + Rot(3) + ExtR(3) + ExtT(3)
    ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 12);
    ekfom_data.z.resize(effect_num_k);
    int m = 0;
    for (int j = 0; j < time_seq[k]; j++) {
        // ekfom_data.converge = false;
        if (point_selected_surf[idx + j + 1]) {
            V3D norm_vec(normvec->points[j].x, normvec->points[j].y,
                         normvec->points[j].z);
            if (extrinsic_est_en) {
                V3D p_body = pbody_list[idx + j + 1];
                M3D p_crossmat, p_imu_crossmat;
                p_crossmat << SKEW_SYM_MATRX(p_body);
                V3D point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
                p_imu_crossmat << SKEW_SYM_MATRX(point_imu);
                V3D C(s.rot.transpose() * norm_vec);
                V3D A(p_imu_crossmat * C);
                V3D B(p_crossmat * s.offset_R_L_I.transpose() * C);
                ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1),
                    norm_vec(2), VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B),
                    VEC_FROM_ARRAY(C);
            } else {
                M3D point_crossmat = crossmat_list[idx + j + 1];
                V3D C(s.rot.transpose() *
                      norm_vec);  // conjugate().normalized()
                V3D A(point_crossmat * C);
                ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1),
                    norm_vec(2), VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0;
            }
            ekfom_data.z(m) =
                -norm_vec(0) * feats_down_world->points[idx + j + 1].x -
                norm_vec(1) * feats_down_world->points[idx + j + 1].y -
                norm_vec(2) * feats_down_world->points[idx + j + 1].z -
                normvec->points[j].intensity;

            m++;
        }
    }
    effct_feat_num += effect_num_k;
}

void h_model_IMU_output(state_output& s,
                        esekfom::dyn_share_modified<double>& ekfom_data) {
    std::memset(ekfom_data.satu_check, false, 6);
    ekfom_data.z_IMU.block<3, 1>(0, 0) = angvel_avr - s.omg - s.bg;
    ekfom_data.z_IMU.block<3, 1>(3, 0) =
        acc_avr * G_m_s2 / acc_norm - s.acc - s.ba;
    ekfom_data.R_IMU << imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_omg_cov,
        imu_meas_acc_cov, imu_meas_acc_cov, imu_meas_acc_cov;
    if (check_satu) {
        if (fabs(angvel_avr(0)) >= 0.99 * satu_gyro) {
            ekfom_data.satu_check[0] = true;
            ekfom_data.z_IMU(0) = 0.0;
        }

        if (fabs(angvel_avr(1)) >= 0.99 * satu_gyro) {
            ekfom_data.satu_check[1] = true;
            ekfom_data.z_IMU(1) = 0.0;
        }

        if (fabs(angvel_avr(2)) >= 0.99 * satu_gyro) {
            ekfom_data.satu_check[2] = true;
            ekfom_data.z_IMU(2) = 0.0;
        }

        if (fabs(acc_avr(0)) >= 0.99 * satu_acc) {
            ekfom_data.satu_check[3] = true;
            ekfom_data.z_IMU(3) = 0.0;
        }

        if (fabs(acc_avr(1)) >= 0.99 * satu_acc) {
            ekfom_data.satu_check[4] = true;
            ekfom_data.z_IMU(4) = 0.0;
        }

        if (fabs(acc_avr(2)) >= 0.99 * satu_acc) {
            ekfom_data.satu_check[5] = true;
            ekfom_data.z_IMU(5) = 0.0;
        }
    }
}

void pointBodyToWorld(PointType const* const pi, PointType* const po) {
    V3D p_body(pi->x, pi->y, pi->z);

    V3D p_global;
    if (extrinsic_est_en) {
        if (!use_imu_as_input) {
            p_global = kf_output.x_.rot * (kf_output.x_.offset_R_L_I * p_body +
                                           kf_output.x_.offset_T_L_I) +
                       kf_output.x_.pos;
        } else {
            p_global = kf_input.x_.rot * (kf_input.x_.offset_R_L_I * p_body +
                                          kf_input.x_.offset_T_L_I) +
                       kf_input.x_.pos;
        }
    } else {
        if (!use_imu_as_input) {
            p_global = kf_output.x_.rot *
                           (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) +
                       kf_output.x_.pos;  // .normalized()
        } else {
            p_global =
                kf_input.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) +
                kf_input.x_.pos;  // .normalized()
        }
    }

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

// ======================= 轮速计观测模型实现 =======================
//
// 观测模型推导：
// =============
// 假设轮速计（底盘中心）在 IMU 坐标系下的位置为 P_WI，旋转为 R_WI (wheel ->
// IMU)
//
// 刚体运动学约束：
//   v_IMU = R_GI * v_G                              (1) IMU原点在IMU系下的速度
//   v_wheel_in_IMU = v_IMU + ω × P_WI               (2) 杠杆臂效应
//   v_wheel = R_WI^T * v_wheel_in_IMU               (3) 变换到轮速计坐标系
//
// 整理得：
//   v_wheel = R_WI^T * (R_GI * v_G + ω × P_WI)
//
// 其中：
//   - v_G = state.vel (全局坐标系下的速度)
//   - R_GI = state.rot^T (全局到IMU的旋转)
//   - ω = state.omg (IMU系下的角速度)
//   - P_WI = 外参平移 (轮速计在IMU系下的位置)
//   - R_WI = 外参旋转 (轮速计系到IMU系的旋转)
//
// 残差定义：
//   r = v_meas - v_pred
//
// 雅可比矩阵推导：
// ===============
// 对速度 v_G 的偏导：
//   ∂v_pred/∂v_G = R_WI^T * R_GI = R_WI^T * rot^T
//
// 对姿态 rot 的偏导（使用右扰动）：
//   ∂v_pred/∂δθ = R_WI^T * ∂(rot^T * v_G)/∂δθ
//               = R_WI^T * [rot^T * v_G]×         (由 R^T v 对 δθ 的导数公式)
//               = R_WI^T * [v_IMU_from_vel]×
//
// 对角速度 ω 的偏导：
//   ∂v_pred/∂ω = R_WI^T * ∂(ω × P_WI)/∂ω
//              = R_WI^T * (-[P_WI]×)
//              = -R_WI^T * [P_WI]×
//
void h_model_wheel_output(state_output& s,
                          const nav_msgs::msg::Odometry& wheel_msg,
                          const Eigen::Vector3d& P_wheel_in_IMU,
                          const Eigen::Matrix3d& R_wheel_to_IMU,
                          double wheel_noise_cov,
                          esekfom::dyn_share_modified<double>& ekfom_data) {
    // 初始化输出
    ekfom_data.wheel_valid = true;
    ekfom_data.z_wheel.setZero();
    ekfom_data.R_wheel.setZero();
    ekfom_data.H_wheel.setZero();

    // Step 1: 获取轮速计测量值 (底盘坐标系下的速度)
    // 注意：轮速计只提供 x, y 方向的速度，z 方向通常为 0 或忽略
    double v_meas_x = wheel_msg.twist.twist.linear.x;
    double v_meas_y = wheel_msg.twist.twist.linear.y;
    Eigen::Vector2d v_meas(v_meas_x, v_meas_y);

    // Step 2: 获取当前状态估计
    // v_G: 全局坐标系下的速度
    Eigen::Vector3d v_G = s.vel;
    // rot: IMU到全局坐标系的旋转 (R_IG)
    // MTK::SO3 类型可以直接当作旋转矩阵使用
    // 注意: s.rot 是 SO3 类型，可以直接用 s.rot 作为 R_IG，用 s.rot.transpose()
    // 作为 R_GI
    Eigen::Matrix3d R_GI = s.rot.transpose();  // R_GI = R_IG^T
    // omg: IMU系下的角速度
    Eigen::Vector3d omg = s.omg;

    // Step 3: 计算预测速度 v_pred
    // v_IMU_from_vel = R_GI * v_G (全局速度变换到IMU系)
    Eigen::Vector3d v_IMU_from_vel = R_GI * v_G;
    // v_lever_arm = ω × P_WI (杠杆臂效应产生的附加速度)
    Eigen::Vector3d v_lever_arm = omg.cross(P_wheel_in_IMU);
    // v_wheel_in_IMU = v_IMU_from_vel + v_lever_arm
    // (轮速计位置在IMU系下的总速度)
    Eigen::Vector3d v_wheel_in_IMU = v_IMU_from_vel + v_lever_arm;
    // v_pred_3d = R_WI^T * v_wheel_in_IMU (变换到轮速计坐标系)
    Eigen::Vector3d v_pred_3d = R_wheel_to_IMU.transpose() * v_wheel_in_IMU;
    // 只取 x, y 分量
    Eigen::Vector2d v_pred(v_pred_3d(0), v_pred_3d(1));

    // Step 4: 计算残差 r = v_meas - v_pred
    ekfom_data.z_wheel = v_meas - v_pred;

    // Step 5: 计算雅可比矩阵 H (2 x 30)
    // state_output 状态向量排列 (对应 30 维):
    //   [0-2]:   pos (3)
    //   [3-5]:   rot (3, SO3的切空间)
    //   [6-8]:   offset_R_L_I (3)
    //   [9-11]:  offset_T_L_I (3)
    //   [12-14]: vel (3)
    //   [15-17]: omg (3)
    //   [18-20]: acc (3)
    //   [21-23]: gravity (3)
    //   [24-26]: bg (3)
    //   [27-29]: ba (3)

    // 辅助矩阵
    Eigen::Matrix3d R_WI_T = R_wheel_to_IMU.transpose();  // R_IW, 从IMU到wheel

// 斜对称矩阵宏
#ifndef SKEW_SYM_MATRX
#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0
#endif

    // ∂r/∂rot: 残差对姿态的偏导 (在状态向量位置 3-5)
    // ∂v_pred/∂δθ = R_WI^T * [v_IMU_from_vel]×
    // 因为 r = v_meas - v_pred, 所以 ∂r/∂δθ = -∂v_pred/∂δθ
    Eigen::Matrix3d skew_v_IMU;
    skew_v_IMU << SKEW_SYM_MATRX(v_IMU_from_vel);
    Eigen::Matrix3d dv_drot = R_WI_T * skew_v_IMU;  // 3x3
    // 取前两行 (只考虑 vx, vy)
    ekfom_data.H_wheel.block<2, 3>(0, 3) = -dv_drot.topRows(2);

    // ∂r/∂vel: 残差对速度的偏导 (在状态向量位置 12-14)
    // ∂v_pred/∂v_G = R_WI^T * R_GI
    Eigen::Matrix3d dv_dvel = R_WI_T * R_GI;  // 3x3
    ekfom_data.H_wheel.block<2, 3>(0, 12) = -dv_dvel.topRows(2);

    // ∂r/∂omg: 残差对角速度的偏导 (在状态向量位置 15-17)
    // ∂v_pred/∂ω = R_WI^T * (-[P_WI]×) = -R_WI^T * skew(P_WI)
    Eigen::Matrix3d skew_P;
    skew_P << SKEW_SYM_MATRX(P_wheel_in_IMU);
    Eigen::Matrix3d dv_domg = -R_WI_T * skew_P;  // 3x3
    ekfom_data.H_wheel.block<2, 3>(0, 15) = -dv_domg.topRows(2);

    // Step 6: 设置观测噪声协方差
    ekfom_data.R_wheel(0) = wheel_noise_cov;
    ekfom_data.R_wheel(1) = wheel_noise_cov;
}