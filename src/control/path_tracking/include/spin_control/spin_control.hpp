#include <visualization_msgs/msg/marker.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "math.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_interfaces/msg/robot.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

// #include <Eigen/Dense>
// x 8.08
// y -1.75
struct pose_tf {
    double x, y, z;
    double roll, pitch, yaw;
};
pose_tf point_base_link;

tf2::Vector3 base_link_to_map_translation;
tf2::Matrix3x3 base_link_to_map_rotation;
// 其它车在map中的位置
tf2::Vector3 car_in_base_position;

// 由于是全向轮，没有角速度
class PID {
   private:
    /* data */
   public:
    PID() {
        linear_err = 0.0;
        linear_err_last = 0.0;
        linear_err_l_last = 0.0;
        expect_vel = 0.0;
        actual_vel = 0.0;
    }
    double linear_err;         // 误差
    double linear_err_last;    // 上次误差
    double linear_err_l_last;  // 上上次误差
    double Kp, Ki, Kd;
    double expect_vel;  // 期望速度
    double actual_vel;  // 实际速度
};

class spin_control : public rclcpp::Node {
   private:
    PID pid;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
        pursuit_point_pub;
    rclcpp::Publisher<serial_interfaces::msg::Robot>::SharedPtr robot_pub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
        transform_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr circle_pub;
    rclcpp::TimerBase::SharedPtr timer_tf;
    rclcpp::TimerBase::SharedPtr timer_cmd;
    nav_msgs::msg::Path::SharedPtr path_;
    geometry_msgs::msg::PoseStamped first_pose;
    serial_interfaces::msg::Robot shaobing;
    geometry_msgs::msg::Twist cmd_vel;

    // base_link的姿态
    pose_tf base_link_map;
    // 路径点的姿态
    //  pose_tf point_base_link;
    // map到base_link的平移矩阵
    tf2::Vector3 map_to_base_link_translation;
    // map到base_link的旋转矩阵
    tf2::Matrix3x3 map_to_base_link_rotation;
    // 路径点在base_link中的位置
    tf2::Vector3 path_in_base_position;
    // 用于存储拟合的路径点集合
    std::vector<geometry_msgs::msg::PoseStamped> circle_points;

    // 路径点标号
    int point_i;
    // 一些参数
    double speed_;
    double min_speed_;
    double tolerance_;
    double lookahead_dist_;
    double lookahead_time_;
    double min_lookahead_dist_;
    double max_lookahead_dist_;
    bool use_velocity_scaled_lookahead_dist_;
    double fit_count_;
    double curvature_k_;

    void init();
    void base_link_callback();
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void calculate_cmd_vel();
    double distance(double x1, double y1, double x2, double y2);
    double getLookAheadDistance(double speed);
    geometry_msgs::msg::PoseStamped getLookAheadPoint(
        const double &lookahead_dist,
        const nav_msgs::msg::Path::SharedPtr &path);
    double optimize_speed(const double curvature);

    double fit_circle(std::vector<geometry_msgs::msg::PoseStamped> &points);

   public:
    spin_control() : Node("path_tracking_node") {
        init();
        // 缓存接收到的tf变换。
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        // 监听器负责监听tf变换并将其存储在缓冲区中。
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        // 按时订阅坐标消息
        timer_tf = this->create_wall_timer(
            std::chrono::milliseconds(4),
            std::bind(&spin_control::base_link_callback, this));
        // 订阅全局路径path话题
        path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 1,
            std::bind(&spin_control::path_callback, this,
                      std::placeholders::_1));
        // 发布速度
        cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel_spin", 1);
        timer_cmd = this->create_wall_timer(
            std::chrono::milliseconds(4),
            std::bind(&spin_control::calculate_cmd_vel, this));
        // 发布要追踪的点、
        pursuit_point_pub =
            this->create_publisher<geometry_msgs::msg::PointStamped>(
                "pursuit_point", 1);
        // 发送机器人的相关信息
        robot_pub =
            this->create_publisher<serial_interfaces::msg::Robot>("robot", 1);
        transform_pub =
            this->create_publisher<geometry_msgs::msg::PointStamped>(
                "base_link_tf", 1);
        // 拟合圆可视化
        circle_pub = this->create_publisher<visualization_msgs::msg::Marker>(
            "circle_marker", 10);
    }

    // static double get_yaw();
};
