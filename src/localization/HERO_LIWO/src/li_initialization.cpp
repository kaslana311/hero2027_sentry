#include "li_initialization.h"

#include <omp.h>
bool data_accum_finished = false, data_accum_start = false,
     online_calib_finish = false, refine_print = false;
int frame_num_init = 0;
double time_lag_IMU_wtr_lidar = 0.0, move_start_time = 0.0,
       online_calib_starts_time = 0.0;  //, mean_acc_norm = 9.81;
double imu_first_time = 0.0;
bool lose_lid = false;
double timediff_imu_wrt_lidar = 0.0;
bool timediff_set_flg = false;
V3D gravity_lio = V3D::Zero();
mutex mtx_buffer;
sensor_msgs::msg::Imu imu_last, imu_next;
// sensor_msgs::msg::Imu::ConstSharedPtr imu_last_ptr;
PointCloudXYZI::Ptr ptr_con(new PointCloudXYZI());
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];

condition_variable sig_buffer;
int scan_count = 0;
int frame_ct = 0, wait_num = 0;
std::mutex m_time;
bool lidar_pushed = false, imu_pushed = false;
std::deque<PointCloudXYZI::Ptr> lidar_buffer;
std::deque<double> time_buffer;
std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;

std::deque<nav_msgs::msg::Odometry> wheel_buffer;

// ======================= 轮速计回调函数实现 =======================
/**
 * @brief 轮速计数据回调函数
 * @param msg_in 轮速计里程计消息指针
 *
 * 功能说明:
 * 1. 接收 ROS2 的 nav_msgs::msg::Odometry 消息
 * 2. 进行时间戳回绕检测（防止 rosbag 回放或系统异常导致的乱序数据）
 * 3. 将有效数据存入 wheel_buffer 等待后续时间同步
 *
 * 数据格式说明:
 * - msg_in->twist.twist.linear: 机器人本体坐标系下的线速度 (v_x, v_y, v_z)
 * - msg_in->twist.twist.angular: 机器人本体坐标系下的角速度 (omega_x, omega_y,
 * omega_z)
 * - 我们主要使用 linear.x 和 linear.y 作为轮速计的速度观测
 */
void wheel_cbk(const nav_msgs::msg::Odometry::ConstSharedPtr& msg_in) {
    // 获取消息时间戳 (单位: 秒)
    double timestamp = omp_get_wtime();

    // 时间戳回绕检测
    // 物理意义: 如果当前时间戳小于上一次记录的时间戳，说明数据出现了乱序
    // 可能原因: rosbag 回放、系统时间重置、网络延迟等
    if (timestamp < last_timestamp_wheel) {
        RCLCPP_WARN(rclcpp::get_logger("li_initialization"),
                    "Wheel odometry timestamp rollback detected! Current: "
                    "%.6f, Last: %.6f",
                    timestamp, last_timestamp_wheel);
        // 不直接返回，而是清空缓冲区后继续（允许从新的时间点开始）
        wheel_buffer.clear();
    }

    // 更新最后时间戳
    last_timestamp_wheel = timestamp;

    // 将数据存入缓冲区
    // 注意: 这里直接拷贝消息内容，避免后续指针失效问题
    wheel_buffer.emplace_back(*msg_in);
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    // mtx_buffer.lock();
    scan_count++;
    double preprocess_start_time = omp_get_wtime();
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {
        RCLCPP_ERROR(rclcpp::get_logger("li_initialization"),
                     "lidar loop back, clear buffer");
        return;
    }

    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();
    // printf("check lidar time %f\n", last_timestamp_lidar);
    // if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 &&
    // !timediff_set_flg && !imu_deque.empty()) {
    //     timediff_set_flg = true;
    //     timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
    //     printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n",
    //     timediff_imu_wrt_lidar);
    // }

    if ((lidar_type == VELO16 || lidar_type == OUST64 ||
         lidar_type == HESAIxt32) &&
        cut_frame_init) {
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        p_pre->process_cut_frame_pcl2(msg, ptr, timestamp_lidar, cut_frame_num,
                                      scan_count);
        while (!ptr.empty() && !timestamp_lidar.empty()) {
            lidar_buffer.push_back(ptr.front());
            ptr.pop_front();
            time_buffer.push_back(timestamp_lidar.front() /
                                  double(1000));  // unit:s
            timestamp_lidar.pop_front();
        }
    } else {
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI(20000, 1));
        p_pre->process(msg, ptr);
        if (con_frame) {
            if (frame_ct == 0) {
                time_con = last_timestamp_lidar;  // msg->header.stamp.toSec();
            }
            if (frame_ct < 10) {
                for (int i = 0; i < ptr->size(); i++) {
                    ptr->points[i].curvature +=
                        (last_timestamp_lidar - time_con) * 1000;
                    ptr_con->push_back(ptr->points[i]);
                }
                frame_ct++;
            } else {
                PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI(10000, 1));
                // std::cout << "ptr div num:" << ptr_div->size() << '\n';
                *ptr_con_i = *ptr_con;
                lidar_buffer.push_back(ptr_con_i);
                double time_con_i = time_con;
                time_buffer.push_back(time_con_i);
                ptr_con->clear();
                frame_ct = 0;
            }
        } else {
            if (!ptr->points.empty()) {
                lidar_buffer.emplace_back(ptr);
                time_buffer.emplace_back(
                    rclcpp::Time(msg->header.stamp).seconds());
            }
        }
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg) {
    // mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {
        RCLCPP_ERROR(rclcpp::get_logger("li_initialization"),
                     "lidar loop back, clear buffer");
        return;
    }

    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();
    // if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 &&
    // !timediff_set_flg && !imu_deque.empty()) {
    //     timediff_set_flg = true;
    //     timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
    //     printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n",
    //     timediff_imu_wrt_lidar);
    // }

    if (cut_frame_init) {
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        p_pre->process_cut_frame_livox(msg, ptr, timestamp_lidar, cut_frame_num,
                                       scan_count);

        while (!ptr.empty() && !timestamp_lidar.empty()) {
            lidar_buffer.push_back(ptr.front());
            ptr.pop_front();
            time_buffer.push_back(timestamp_lidar.front() /
                                  double(1000));  // unit:s
            timestamp_lidar.pop_front();
        }
    } else {
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI(10000, 1));
        p_pre->process(msg, ptr);
        if (con_frame) {
            if (frame_ct == 0) {
                time_con = last_timestamp_lidar;  // msg->header.stamp.toSec();
            }
            if (frame_ct < 10) {
                for (int i = 0; i < ptr->size(); i++) {
                    ptr->points[i].curvature +=
                        (last_timestamp_lidar - time_con) * 1000;
                    ptr_con->push_back(ptr->points[i]);
                }
                frame_ct++;
            } else {
                PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI(10000, 1));
                // std::cout << "ptr div num:" << ptr_div->size() << '\n';
                *ptr_con_i = *ptr_con;
                double time_con_i = time_con;
                lidar_buffer.push_back(ptr_con_i);
                time_buffer.push_back(time_con_i);
                ptr_con->clear();
                frame_ct = 0;
            }
        } else {
            if (!ptr->points.empty()) {
                lidar_buffer.emplace_back(ptr);
                time_buffer.emplace_back(
                    rclcpp::Time(msg->header.stamp).seconds());
            }
        }
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr& msg_in) {
    // mtx_buffer.lock();

    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    // publish_count ++;

    msg->header.stamp =
        get_ros_time(get_time_sec(msg_in->header.stamp) -
                     timediff_imu_wrt_lidar - time_lag_IMU_wtr_lidar);

    double timestamp = get_time_sec(msg->header.stamp);
    // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp,
    // last_timestamp_imu, timestamp);

    if (timestamp < last_timestamp_imu) {
        RCLCPP_ERROR(rclcpp::get_logger("li_initialization"),
                     "imu loop back, clear deque");
        // imu_deque.shrink_to_fit();
        // std::cout << "check time:" << timestamp << ";" << last_timestamp_imu
        // << '\n'; printf("time_diff%f, %f, %f\n", last_timestamp_imu -
        // timestamp, last_timestamp_imu, timestamp);

        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
    }
    imu_deque.emplace_back(msg);
    last_timestamp_imu = timestamp;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup& meas) {
    {
        if (!imu_en) {
            if (!lidar_buffer.empty()) {
                if (!lidar_pushed) {
                    meas.lidar = lidar_buffer.front();
                    meas.lidar_beg_time = time_buffer.front();
                    lose_lid = false;
                    if (meas.lidar->points.empty()) {
                        std::cout << "lose lidar" << '\n';
                        // return false;
                        lose_lid = true;
                    } else {
                        double end_time = meas.lidar->points.back().curvature;
                        for (auto pt : meas.lidar->points) {
                            if (pt.curvature > end_time) {
                                end_time = pt.curvature;
                            }
                        }
                        lidar_end_time =
                            meas.lidar_beg_time + end_time / double(1000);
                        meas.lidar_last_time = lidar_end_time;
                    }
                    lidar_pushed = true;
                }

                time_buffer.pop_front();
                lidar_buffer.pop_front();
                lidar_pushed = false;
                if (!lose_lid) {
                    return true;
                } else {
                    return false;
                }
            }
            return false;
        }

        if (lidar_buffer.empty() || imu_deque.empty()) {
            return false;
        }
        /*** push a lidar scan ***/
        if (!lidar_pushed) {
            lose_lid = false;
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            if (meas.lidar->points.size() < 1) {
                std::cout << "lose lidar" << '\n';
                lose_lid = true;
                // lidar_buffer.pop_front();
                // time_buffer.pop_front();
                // return false;
            } else {
                double end_time = meas.lidar->points.back().curvature;
                for (auto pt : meas.lidar->points) {
                    if (pt.curvature > end_time) {
                        end_time = pt.curvature;
                    }
                }
                lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
                // std::cout << "check time lidar:" << end_time << '\n';
                meas.lidar_last_time = lidar_end_time;
            }
            lidar_pushed = true;
        }

        if (!lose_lid && (last_timestamp_imu < lidar_end_time)) {
            return false;
        }
        if (lose_lid &&
            last_timestamp_imu < meas.lidar_beg_time + lidar_time_inte) {
            return false;
        }

        if (!lose_lid && !imu_pushed) {
            /*** push imu data, and pop from imu buffer ***/
            if (p_imu->imu_need_init_) {
                double imu_time = get_time_sec(imu_deque.front()->header.stamp);
                imu_next = *(imu_deque.front());
                meas.imu.shrink_to_fit();
                while (imu_time < lidar_end_time) {
                    meas.imu.emplace_back(imu_deque.front());
                    imu_last = imu_next;
                    imu_deque.pop_front();
                    if (imu_deque.empty()) break;
                    imu_time = get_time_sec(
                        imu_deque.front()->header.stamp);  // can be changed
                    imu_next = *(imu_deque.front());
                }
            }
            imu_pushed = true;
        }

        if (lose_lid && !imu_pushed) {
            /*** push imu data, and pop from imu buffer ***/
            if (p_imu->imu_need_init_) {
                double imu_time = get_time_sec(imu_deque.front()->header.stamp);
                meas.imu.shrink_to_fit();

                imu_next = *(imu_deque.front());
                while (imu_time < meas.lidar_beg_time + lidar_time_inte) {
                    meas.imu.emplace_back(imu_deque.front());
                    imu_last = imu_next;
                    imu_deque.pop_front();
                    if (imu_deque.empty()) break;
                    imu_time = get_time_sec(
                        imu_deque.front()->header.stamp);  // can be changed
                    imu_next = *(imu_deque.front());
                }
            }
            imu_pushed = true;
        }

        // ========= 轮速计数据同步 (Wheel Odometry Sync) =========
        // 功能: 从全局 wheel_buffer 中提取与当前 LiDAR
        // 帧时间对齐的轮速数据 时间窗口: [meas.lidar_beg_time, lidar_end_time]
        //
        // 同步策略说明:
        // 1. 首先清除 meas.wheel_odom 中的旧数据（来自上一帧的残留）
        // 2. 遍历 wheel_buffer，丢弃所有时间戳早于 lidar_beg_time 的过期数据
        //    - 这些数据已经错过了当前帧的处理窗口，无法用于融合
        // 3. 将时间戳在 [lidar_beg_time, lidar_end_time] 区间内的轮速数据压入
        // meas.wheel_odom
        // 4. 不 pop
        // 区间内的数据，因为下一帧可能仍需要用到（如果时间窗口有重叠）
        //    - 但实际上 LiDAR 帧是连续的，所以区间内数据用完后会自然过期
        //
        // 鲁棒性考虑:
        // - 如果 wheel_buffer 为空或该窗口内没有轮速数据，meas.wheel_odom
        // 会是空的
        // - 这在 cut_frame 模式（短时间窗口如 10ms）下是正常现象
        // - 后端 Estimator 需要处理 wheel_odom 为空的情况（跳过轮速约束）
        if (wheel_en && !wheel_buffer.empty()) {
            // 清空上一帧的残留数据
            meas.wheel_odom.clear();

            // Step 1: 丢弃过期数据（时间戳 < lidar_beg_time）
            // 物理意义:
            // 这些轮速数据对应的时刻早于当前帧，已无法参与当前状态的优化
            while (!wheel_buffer.empty()) {
                double wheel_time =
                    get_time_sec(wheel_buffer.front().header.stamp);
                if (wheel_time < meas.lidar_beg_time) {
                    // 数据已过期，直接丢弃
                    wheel_buffer.pop_front();
                } else {
                    // 遇到第一个未过期的数据，停止丢弃
                    break;
                }
            }

            // Step 2: 提取时间窗口内的轮速数据
            // 遍历 wheel_buffer，将符合条件的数据拷贝到 meas.wheel_odom
            for (const auto& wheel_msg : wheel_buffer) {
                double wheel_time = get_time_sec(wheel_msg.header.stamp);

                // 检查是否在有效时间窗口内
                if (wheel_time >= meas.lidar_beg_time &&
                    wheel_time <= lidar_end_time) {
                    // 数据在窗口内，加入测量组
                    meas.wheel_odom.emplace_back(wheel_msg);
                } else if (wheel_time > lidar_end_time) {
                    // 数据时间已超过当前帧结束时间，后续数据也不会在窗口内
                    // 提前退出循环以提高效率
                    break;
                }
            }

            // 可选: 输出调试信息（正式版本可注释掉）
            // if (!meas.wheel_odom.empty()) {
            //     RCLCPP_DEBUG(rclcpp::get_logger("li_initialization"),
            //         "Synced %zu wheel odom messages in time window [%.3f,
            //         %.3f]", meas.wheel_odom.size(), meas.lidar_beg_time,
            //         lidar_end_time);
            // }
        }

        lidar_buffer.pop_front();
        time_buffer.pop_front();
        lidar_pushed = false;
        imu_pushed = false;
        return true;
    }
}
