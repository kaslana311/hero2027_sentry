//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lidar_imu_data_queue.h"
#include "comm/comm.h"
namespace livox_ros {
void LidarImuDataQueue::Push(ImuData* imu_data, void *p_lidar) {
    ImuData data;
    auto p=static_cast<LidarDevice *>(p_lidar);
    auto cfg = p->livox_config.extrinsic_param;

    data.lidar_type = imu_data->lidar_type;

    double roll = cfg.roll;
    double pitch = cfg.pitch;
    double yaw = cfg.yaw;  // 单位：度

    // 对 IMU 数据进行坐标变换（仅旋转，平移对IMU无意义）
    const double deg2rad = M_PI / 180.0;
    const double cr = std::cos(roll * deg2rad);
    const double sr = std::sin(roll * deg2rad);
    const double cp = std::cos(pitch * deg2rad);
    const double sp = std::sin(pitch * deg2rad);
    const double cy = std::cos(yaw * deg2rad);
    const double sy = std::sin(yaw * deg2rad);

    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    const double r00 = cy * cp;
    const double r01 = cy * sp * sr - sy * cr;
    const double r02 = cy * sp * cr + sy * sr;
    const double r10 = sy * cp;
    const double r11 = sy * sp * sr + cy * cr;
    const double r12 = sy * sp * cr - cy * sr;
    const double r20 = -sp;
    const double r21 = cp * sr;
    const double r22 = cp * cr;

    const double gx = imu_data->gyro_x;
    const double gy = imu_data->gyro_y;
    const double gz = imu_data->gyro_z;

    const double ax = imu_data->acc_x;
    const double ay = imu_data->acc_y;
    const double az = imu_data->acc_z;

    data.handle = imu_data->handle;
    data.time_stamp = imu_data->time_stamp;

    data.gyro_x = r00 * gx + r01 * gy + r02 * gz;
    data.gyro_y = r10 * gx + r11 * gy + r12 * gz;
    data.gyro_z = r20 * gx + r21 * gy + r22 * gz;

    data.acc_x = r00 * ax + r01 * ay + r02 * az;
    data.acc_y = r10 * ax + r11 * ay + r12 * az;
    data.acc_z = r20 * ax + r21 * ay + r22 * az;

    std::lock_guard<std::mutex> lock(mutex_);
    imu_data_queue_.push_back(std::move(data));
}
void LidarImuDataQueue::Push(ImuData* imu_data) {
    ImuData data;
    data.lidar_type = imu_data->lidar_type;
    data.handle = imu_data->handle;
    data.time_stamp = imu_data->time_stamp;

    data.gyro_x = imu_data->gyro_x;
    data.gyro_y = imu_data->gyro_y;
    data.gyro_z = imu_data->gyro_z;

    data.acc_x = imu_data->acc_x;
    data.acc_y = imu_data->acc_y;
    data.acc_z = imu_data->acc_z;

    std::lock_guard<std::mutex> lock(mutex_);
    imu_data_queue_.push_back(std::move(data));
}

bool LidarImuDataQueue::Pop(ImuData& imu_data) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (imu_data_queue_.empty()) {
        return false;
    }
    imu_data = imu_data_queue_.front();
    imu_data_queue_.pop_front();
    return true;
}

bool LidarImuDataQueue::Empty() {
    std::lock_guard<std::mutex> lock(mutex_);
    return imu_data_queue_.empty();
}

void LidarImuDataQueue::Clear() {
    std::list<ImuData> tmp_imu_data_queue;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        imu_data_queue_.swap(tmp_imu_data_queue);
    }
}

}  // namespace livox_ros