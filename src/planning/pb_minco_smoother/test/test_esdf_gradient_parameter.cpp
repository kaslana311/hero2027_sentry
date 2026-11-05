// Exercises the real smoother and its ROS parameter service, without a robot,
// odometry feed, hardware commands, or the navigation behavior tree.
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <nav2_msgs/msg/costmap.hpp>
#include <rclcpp/parameter_client.hpp>

#include "pb_minco_smoother/minco_smoother.hpp"

namespace {

class EsdfGradientParameter : public ::testing::Test {
 protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override {
        node_ = std::make_shared<nav2_util::LifecycleNode>(
            "smoother_probe", "/esdf_gradient_test");
        client_node_ = std::make_shared<rclcpp::Node>(
            "parameter_client", "/esdf_gradient_test");
        executor_.add_node(node_->get_node_base_interface());
        executor_.add_node(client_node_);
        client_ = std::make_shared<rclcpp::AsyncParametersClient>(
            client_node_, "/esdf_gradient_test/smoother_probe");
        ASSERT_TRUE(client_->wait_for_service(std::chrono::seconds(3)));
    }

    void configure(double stage1_weight, double stage2_weight, bool obstacle) {
        node_->declare_parameter("minco_smoother.stage1_weight_obstacle", stage1_weight);
        node_->declare_parameter("minco_smoother.weight_obstacle", stage2_weight);
        auto tf = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        auto costs = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
            node_, "/esdf_gradient_test/input_costmap");
        auto map = std::make_shared<nav2_msgs::msg::Costmap>();
        map->header.frame_id = "map";
        map->metadata.size_x = 120;
        map->metadata.size_y = 120;
        map->metadata.resolution = 0.05;
        map->metadata.origin.position.x = -3.0;
        map->metadata.origin.position.y = -3.0;
        map->metadata.origin.orientation.w = 1.0;
        map->data.resize(120 * 120, 0);
        if (obstacle) {
            for (int j = 0; j < 120; ++j) {
                for (int i = 0; i < 120; ++i) {
                    double x = -3.0 + (i + 0.5) * 0.05;
                    double y = -3.0 + (j + 0.5) * 0.05;
                    if (std::abs(0.8 * x + 0.6 * y) < 0.10 &&
                        std::hypot(x, y) < 1.2) {
                        map->data[j * 120 + i] = 254;
                    }
                }
            }
        }
        costs->costmapCallback(map);
        smoother_ = std::make_unique<pb_minco::MincoSmoother>();
        smoother_->configure(node_, "minco_smoother", tf, costs, nullptr);
        smoother_->activate();
    }

    void TearDown() override {
        if (smoother_) {
            smoother_->deactivate();
            smoother_->cleanup();
            smoother_.reset();
        }
        client_.reset();
        executor_.remove_node(node_->get_node_base_interface());
        executor_.remove_node(client_node_);
    }

    rcl_interfaces::msg::SetParametersResult set(const rclcpp::Parameter &parameter) {
        auto future = client_->set_parameters({parameter});
        if (executor_.spin_until_future_complete(future, std::chrono::seconds(3)) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            ADD_FAILURE() << "Parameter service did not reply within three seconds";
            rcl_interfaces::msg::SetParametersResult failed;
            failed.successful = false;
            failed.reason = "Parameter service timed out";
            return failed;
        }
        return future.get().at(0);
    }

    nav_msgs::msg::Path smooth() {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = node_->now();
        for (int i = 0; i <= 40; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = -1.0 + i * 0.05;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        // No odometry is published: all runs use the same original cold-start
        // boundary, so the A/B comparison cannot inherit a different trajectory.
        EXPECT_TRUE(smoother_->smooth(path, rclcpp::Duration::from_seconds(3.0)));
        EXPECT_GT(path.poses.size(), 2u);
        for (const auto &pose : path.poses) {
            EXPECT_TRUE(std::isfinite(pose.pose.position.x));
            EXPECT_TRUE(std::isfinite(pose.pose.position.y));
        }
        return path;
    }

    double difference(const nav_msgs::msg::Path &a, const nav_msgs::msg::Path &b) {
        double delta = std::abs(double(a.poses.size()) - double(b.poses.size()));
        for (std::size_t i = 0; i < std::min(a.poses.size(), b.poses.size()); ++i) {
            delta = std::max(delta, std::hypot(
                a.poses[i].pose.position.x - b.poses[i].pose.position.x,
                a.poses[i].pose.position.y - b.poses[i].pose.position.y));
        }
        return delta;
    }

    void checkToggle(double first, double second) {
        configure(first, second, true);
        EXPECT_TRUE(node_->get_parameter("minco_smoother.project_esdf_gradient").as_bool());
        const auto legacy = smooth();
        ASSERT_TRUE(set({"minco_smoother.project_esdf_gradient", false}).successful);
        EXPECT_FALSE(node_->get_parameter("minco_smoother.project_esdf_gradient").as_bool());
        const auto full = smooth();
        EXPECT_GT(difference(legacy, full), 1e-6);
        ASSERT_TRUE(set({"minco_smoother.project_esdf_gradient", true}).successful);
        EXPECT_DOUBLE_EQ(difference(legacy, smooth()), 0.0);
    }

    nav2_util::LifecycleNode::SharedPtr node_;
    rclcpp::Node::SharedPtr client_node_;
    std::shared_ptr<rclcpp::AsyncParametersClient> client_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::unique_ptr<pb_minco::MincoSmoother> smoother_;
};

TEST_F(EsdfGradientParameter, BothStagesRespondAndRestore) { checkToggle(50.0, 100.0); }
TEST_F(EsdfGradientParameter, FirstStageReceivesTheSwitch) { checkToggle(50.0, 0.0); }
TEST_F(EsdfGradientParameter, SecondStageReceivesTheSwitch) { checkToggle(0.0, 100.0); }

TEST_F(EsdfGradientParameter, FreeSpaceIsIdentical) {
    configure(50.0, 100.0, false);
    const auto legacy = smooth();
    ASSERT_TRUE(set({"minco_smoother.project_esdf_gradient", false}).successful);
    EXPECT_DOUBLE_EQ(difference(legacy, smooth()), 0.0);
}

TEST_F(EsdfGradientParameter, ParameterTypeRemainsBoolean) {
    configure(50.0, 100.0, false);
    EXPECT_FALSE(set({"minco_smoother.project_esdf_gradient", 1.0}).successful);
    EXPECT_TRUE(node_->get_parameter("minco_smoother.project_esdf_gradient").as_bool());
}

}  // namespace
