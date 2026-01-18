#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

class TFStatisticsNode : public rclcpp::Node {
   private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string target_frame_;
    std::string source_frame_;
    double update_rate_;
    size_t max_samples_;

    std::vector<Eigen::Vector3d> positions_;
    std::vector<Eigen::Vector3d>
        orientations_;  // 存储欧拉角 (roll, pitch, yaw)

   public:
    TFStatisticsNode() : Node("tf_statistics_node") {
        // 声明参数
        this->declare_parameter<std::string>("target_frame", "map");
        this->declare_parameter<std::string>("source_frame", "base_link");
        this->declare_parameter<double>("update_rate", 10.0);
        this->declare_parameter<int>("max_samples", 1000);

        // 获取参数
        target_frame_ = this->get_parameter("target_frame").as_string();
        source_frame_ = this->get_parameter("source_frame").as_string();
        update_rate_ = this->get_parameter("update_rate").as_double();
        max_samples_ = this->get_parameter("max_samples").as_int();

        // 初始化TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建定时器
        auto timer_duration = std::chrono::duration<double>(1.0 / update_rate_);
        timer_ = this->create_wall_timer(
            timer_duration, std::bind(&TFStatisticsNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "TF Statistics Node initialized");
        RCLCPP_INFO(this->get_logger(), "Tracking transform from '%s' to '%s'",
                    source_frame_.c_str(), target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "Update rate: %.1f Hz, Max samples: %zu", update_rate_,
                    max_samples_);
    }

   private:
    void timer_callback() {
        try {
            // 获取最新的TF变换
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                target_frame_, source_frame_, tf2::TimePointZero);

            // 提取位置
            Eigen::Vector3d position;
            position.x() = transform_stamped.transform.translation.x;
            position.y() = transform_stamped.transform.translation.y;
            position.z() = transform_stamped.transform.translation.z;

            // 提取四元数并转换为欧拉角
            tf2::Quaternion quat;
            quat.setX(transform_stamped.transform.rotation.x);
            quat.setY(transform_stamped.transform.rotation.y);
            quat.setZ(transform_stamped.transform.rotation.z);
            quat.setW(transform_stamped.transform.rotation.w);

            Eigen::Vector3d euler_angles;
            tf2::Matrix3x3 mat(quat);
            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);
            euler_angles << roll, pitch, yaw;

            // 添加到数据容器
            positions_.push_back(position);
            orientations_.push_back(euler_angles);

            // 限制数据量
            if (positions_.size() > max_samples_) {
                positions_.erase(positions_.begin());
                orientations_.erase(orientations_.begin());
            }

            // 定期输出统计信息
            if (positions_.size() % 100 == 0) {
                print_stats();
            }

        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF exception: %s", ex.what());
        }
    }

    void calculate_position_stats(Eigen::Vector3d& mean,
                                  Eigen::Vector3d& variance) {
        if (positions_.empty()) {
            mean = Eigen::Vector3d::Zero();
            variance = Eigen::Vector3d::Zero();
            return;
        }

        // 计算均值
        mean.setZero();
        for (const auto& pos : positions_) {
            mean += pos;
        }
        mean /= positions_.size();

        // 计算方差
        variance.setZero();
        for (const auto& pos : positions_) {
            Eigen::Vector3d diff = pos - mean;
            variance += diff.cwiseProduct(diff);
        }
        variance /= positions_.size();
    }

    void calculate_angle_stats(Eigen::Vector3d& mean,
                               Eigen::Vector3d& variance) {
        if (orientations_.empty()) {
            mean = Eigen::Vector3d::Zero();
            variance = Eigen::Vector3d::Zero();
            return;
        }

        // 使用单位圆坐标计算角度均值（处理周期性）
        Eigen::Vector3d sin_sum = Eigen::Vector3d::Zero();
        Eigen::Vector3d cos_sum = Eigen::Vector3d::Zero();

        for (const auto& angles : orientations_) {
            sin_sum += angles.array().sin().matrix();
            cos_sum += angles.array().cos().matrix();
        }

        sin_sum /= orientations_.size();
        cos_sum /= orientations_.size();

        // 计算平均角度
        mean = sin_sum.binaryExpr(
            cos_sum, [](double s, double c) { return std::atan2(s, c); });

        // 计算角度方差（处理周期性）
        variance.setZero();
        for (const auto& angles : orientations_) {
            for (int i = 0; i < 3; ++i) {
                double diff = std::atan2(std::sin(angles[i] - mean[i]),
                                         std::cos(angles[i] - mean[i]));
                variance[i] += diff * diff;
            }
        }
        variance /= orientations_.size();
    }

    void print_stats() {
        if (positions_.empty()) {
            RCLCPP_INFO(this->get_logger(), "No data collected yet");
            return;
        }

        Eigen::Vector3d pos_mean, pos_var;
        Eigen::Vector3d ang_mean, ang_var;

        calculate_position_stats(pos_mean, pos_var);
        calculate_angle_stats(ang_mean, ang_var);

        RCLCPP_INFO(this->get_logger(),
                    "=== TF Statistics (Samples: %zu) ===", positions_.size());

        RCLCPP_INFO(this->get_logger(),
                    "Position - Mean (x, y, z): [%.4f, %.4f, %.4f]",
                    pos_mean.x(), pos_mean.y(), pos_mean.z());
        RCLCPP_INFO(this->get_logger(),
                    "Position - Variance (x, y, z): [%.6f, %.6f, %.6f]",
                    pos_var.x(), pos_var.y(), pos_var.z());
        RCLCPP_INFO(this->get_logger(),
                    "Position - Std Dev (x, y, z): [%.4f, %.4f, %.4f]",
                    std::sqrt(pos_var.x()), std::sqrt(pos_var.y()),
                    std::sqrt(pos_var.z()));

        RCLCPP_INFO(
            this->get_logger(),
            "Orientation - Mean (roll, pitch, yaw) [rad]: [%.4f, %.4f, %.4f]",
            ang_mean.x(), ang_mean.y(), ang_mean.z());
        RCLCPP_INFO(this->get_logger(),
                    "Orientation - Variance (roll, pitch, yaw) [rad²]: [%.6f, "
                    "%.6f, %.6f]",
                    ang_var.x(), ang_var.y(), ang_var.z());
        RCLCPP_INFO(this->get_logger(),
                    "Orientation - Std Dev (roll, pitch, yaw) [rad]: [%.4f, "
                    "%.4f, %.4f]",
                    std::sqrt(ang_var.x()), std::sqrt(ang_var.y()),
                    std::sqrt(ang_var.z()));

        // 转换为角度显示
        Eigen::Vector3d ang_mean_deg = ang_mean * 180.0 / M_PI;
        Eigen::Vector3d ang_var_deg = ang_var * 180.0 / M_PI;
        RCLCPP_INFO(
            this->get_logger(),
            "Orientation - Mean (roll, pitch, yaw) [deg]: [%.4f, %.4f, %.4f]",
            ang_mean_deg.x(), ang_mean_deg.y(), ang_mean_deg.z());
        RCLCPP_INFO(this->get_logger(),
                    "Orientation - Std Dev (roll, pitch, yaw) [deg]: [%.4f, "
                    "%.4f, %.4f]",
                    std::sqrt(ang_var_deg.x()), std::sqrt(ang_var_deg.y()),
                    std::sqrt(ang_var_deg.z()));

        RCLCPP_INFO(this->get_logger(),
                    "=====================================");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFStatisticsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
