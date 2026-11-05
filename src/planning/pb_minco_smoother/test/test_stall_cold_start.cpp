// Real BT detector + real MINCO plugin, isolated ROS domain, synthetic motion.
// No chassis command topic, Gazebo, or ground-truth velocity is used.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <rcl/time.h>

#include "pb_minco_smoother/minco_smoother.hpp"
#include "pb_nav2_plugins/bt_nodes/is_stuck_condition.hpp"

namespace {
using Poly = interfaces::msg::MincoTrajectory;

class StallColdStart : public ::testing::Test {
 protected:
    static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
    static void TearDownTestSuite() {rclcpp::shutdown();}

    void setup(bool enabled = true, bool detector = true) {
        rclcpp::NodeOptions options;
        options.parameter_overrides({{"use_sim_time", true}});
        node_ = std::make_shared<nav2_util::LifecycleNode>("smoother", "/stall_cold_test", options);
        driver_ = std::make_shared<rclcpp::Node>("driver", "/stall_cold_test", options);
        setTime(100.0);
        node_->declare_parameter("minco_smoother.odom_topic", "/stall_cold_test/odom");
        if (enabled) {
            node_->declare_parameter("minco_smoother.stuck_state_topic", "/stall_cold_test/stalled");
        }
        executor_.add_node(node_->get_node_base_interface());
        executor_.add_node(driver_);
        auto costs = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
            node_, "/stall_cold_test/unused_smoother_costmap");
        auto map = std::make_shared<nav2_msgs::msg::Costmap>();
        map->header.frame_id = "map";
        map->metadata.size_x = 100; map->metadata.size_y = 100;
        map->metadata.resolution = 0.1;
        map->metadata.origin.position.x = -5.0; map->metadata.origin.position.y = -5.0;
        map->metadata.origin.orientation.w = 1.0;
        map->data.resize(10000, 0);
        costs->costmapCallback(map);
        tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        smoother_ = std::make_unique<pb_minco::MincoSmoother>();
        smoother_->configure(node_, "minco_smoother", tf_, costs, nullptr);
        smoother_->activate();
        cmd_pub_ = driver_->create_publisher<geometry_msgs::msg::Twist>("command_probe", 10);
        odom_pub_ = driver_->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
        cost_pub_ = driver_->create_publisher<nav2_msgs::msg::Costmap>("detector_costmap", rclcpp::SensorDataQoS());
        signal_pub_ = driver_->create_publisher<std_msgs::msg::Bool>("stalled", 10);
        signal_sub_ = driver_->create_subscription<std_msgs::msg::Bool>("stalled", 10,
            [this](std_msgs::msg::Bool::ConstSharedPtr msg) {signals_.push_back(msg->data);});
        poly_sub_ = driver_->create_subscription<Poly>("smoother/minco_polynomial_trajectory", 10,
            [this](Poly::ConstSharedPtr msg) {poly_ = *msg; ++poly_count_;});
        if (detector) {
            factory_ = std::make_unique<BT::BehaviorTreeFactory>();
            factory_->registerNodeType<pb_nav2_plugins::IsStuckCondition>("IsStuckCondition");
            auto blackboard = BT::Blackboard::create();
            blackboard->set("node", driver_);
            blackboard->set("tf_buffer", tf_);
            const std::string signal_port = enabled ? " motion_stall_topic='stalled'" : "";
            tree_ = factory_->createTreeFromText(
                "<root main_tree_to_execute='MainTree'><BehaviorTree ID='MainTree'>"
                "<IsStuckCondition cmd_vel_topic='command_probe' odom_topic='odom' "
                "costmap_topic='detector_costmap'" + signal_port + "/>"
                "</BehaviorTree></root>", blackboard);
        }
        drain(30);
    }

    void TearDown() override {
        tree_.nodes.clear();  // Stop/join the detector's dedicated executor first.
        factory_.reset();
        if (smoother_) {smoother_->deactivate(); smoother_->cleanup(); smoother_.reset();}
        if (node_) {executor_.remove_node(node_->get_node_base_interface());}
        if (driver_) {executor_.remove_node(driver_);}
    }

    void setTime(double time) {
        for (const auto &clock : {node_->get_clock(), driver_->get_clock()}) {
            ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(),
                static_cast<int64_t>(time * 1e9)), RCL_RET_OK);
        }
    }

    void drain(int count = 5) {
        for (int i = 0; i < count; ++i) {
            executor_.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    void feed(double time, double x, double command = 0.5) {
        setTime(time);
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = command;
        cmd_pub_->publish(cmd);
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = "map"; odom.header.stamp = node_->now();
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x; odom.pose.pose.orientation.w = 1.0;
        // Deliberately leave measured twist zero, even when changing position.
        // Normal MINCO boundary must still inherit its own projected velocity.
        odom_pub_->publish(odom);
        drain();
    }

    Poly smooth(double x) {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map"; path.header.stamp = node_->now();
        for (int i = 0; i <= 80; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = x + (4.0 - x) * i / 80;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        auto previous = poly_count_;
        EXPECT_TRUE(smoother_->smooth(path, rclcpp::Duration::from_seconds(3.0)));
        drain();
        EXPECT_GT(poly_count_, previous);
        EXPECT_GE(poly_.coefficients.size(), 12u);
        return poly_;
    }

    double headSpeed(const Poly &poly) {
        return std::hypot(poly.coefficients.at(4), poly.coefficients.at(10));
    }
    double headAcceleration(const Poly &poly) {
        return 2.0 * std::hypot(poly.coefficients.at(3), poly.coefficients.at(9));
    }
    void emit(bool value) {
        std_msgs::msg::Bool signal; signal.data = value;
        signal_pub_->publish(signal); drain();
    }

    nav2_util::LifecycleNode::SharedPtr node_;
    rclcpp::Node::SharedPtr driver_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::unique_ptr<pb_minco::MincoSmoother> smoother_;
    std::unique_ptr<BT::BehaviorTreeFactory> factory_;
    BT::Tree tree_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr cost_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr signal_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr signal_sub_;
    rclcpp::Subscription<Poly>::SharedPtr poly_sub_;
    Poly poly_;
    unsigned poly_count_{0};
    std::vector<bool> signals_;
};

TEST_F(StallColdStart, ConfirmedStallColdUntilRealProgressWithoutTickingTheBt) {
    setup();
    feed(100.0, 0.0); smooth(0.0);
    feed(100.25, 0.04);
    EXPECT_GT(headSpeed(smooth(0.04)), 1e-4);
    // Never tick the BT: the smoother must not wait for controller retries.
    for (int i = 1; i <= 17; ++i) {feed(100.25 + i * 0.1, 0.04);}
    ASSERT_FALSE(signals_.empty()); EXPECT_TRUE(signals_.back());
    const auto cold = smooth(0.04);
    EXPECT_NEAR(headSpeed(cold), 0.0, 1e-9);
    EXPECT_NEAR(headAcceleration(cold), 0.0, 1e-9);
    EXPECT_NEAR(cold.coefficients.at(5), 0.04, 1e-9);
    feed(102.05, 0.04, 0.0);
    EXPECT_TRUE(signals_.back());
    EXPECT_NEAR(headSpeed(smooth(0.04)), 0.0, 1e-9);
    feed(102.15, 0.08);
    EXPECT_FALSE(signals_.back());
    EXPECT_GT(headSpeed(smooth(0.08)), 1e-4);
    // An unconsumed legacy latch must not disable detection of a second stall.
    for (int i = 1; i <= 17; ++i) {feed(102.15 + i * 0.1, 0.08);}
    EXPECT_TRUE(signals_.back());
    EXPECT_NEAR(headSpeed(smooth(0.08)), 0.0, 1e-9);
}

TEST_F(StallColdStart, DisabledIgnoresTheOptionalSignalAndKeepsProjectedVelocity) {
    setup(false, false);
    feed(100.0, 0.0); smooth(0.0);
    feed(100.25, 0.04); emit(true);
    EXPECT_GT(headSpeed(smooth(0.04)), 1e-4);
}

TEST_F(StallColdStart, BriefConfirmedEventInvalidatesPreStallCacheEvenAfterRelease) {
    setup(true, false);
    feed(100.0, 0.0); smooth(0.0);
    feed(100.25, 0.04);
    EXPECT_GT(headSpeed(smooth(0.04)), 1e-4);
    emit(true); emit(false);
    EXPECT_NEAR(headSpeed(smooth(0.04)), 0.0, 1e-9);
    EXPECT_NEAR(headAcceleration(poly_), 0.0, 1e-9);
}

TEST_F(StallColdStart, CostmapOverlapAloneDoesNotDisableWarmStart) {
    setup();
    feed(100.0, 0.0); smooth(0.0);
    auto cost = std::make_shared<nav2_msgs::msg::Costmap>();
    cost->header.frame_id = "map";
    cost->metadata.size_x = 20; cost->metadata.size_y = 20;
    cost->metadata.resolution = 0.1;
    cost->metadata.origin.position.x = -1.; cost->metadata.origin.position.y = -1.;
    cost->data.resize(400, 254);
    cost_pub_->publish(*cost);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map"; transform.child_frame_id = "base_link";
    transform.transform.rotation.w = 1.0;
    tf_->setTransform(transform, "synthetic_test", true);
    bool cost_triggered = false;
    for (int i = 1; i <= 8; ++i) {
        feed(100.0 + i * 0.21, i * 0.04);
        cost_triggered |= tree_.tickRoot() == BT::NodeStatus::SUCCESS;
    }
    EXPECT_TRUE(cost_triggered);
    ASSERT_FALSE(signals_.empty()); EXPECT_FALSE(signals_.back());
    // Refresh the previous trajectory, then check a normal moving warm start.
    smooth(0.32);
    feed(101.88, 0.36);
    EXPECT_GT(headSpeed(smooth(0.36)), 1e-4);
}

}  // namespace
