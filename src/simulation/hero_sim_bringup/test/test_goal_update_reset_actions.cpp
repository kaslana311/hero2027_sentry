// Real Nav2 BtActionNode clients, isolated fake action servers, no Gazebo.
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <vector>
#include <unistd.h>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using Compute = nav2_msgs::action::ComputePathToPose;
using Smooth = nav2_msgs::action::SmoothPath;
using Follow = nav2_msgs::action::FollowPath;
using Status = BT::NodeStatus;
using Pose = geometry_msgs::msg::PoseStamped;
void check(bool b, const char * text) {if (!b) {throw std::runtime_error(text);}}
Pose goal(int x) {Pose p; p.header.frame_id = "map"; p.pose.position.x = x; p.pose.orientation.w = 1.; return p;}

struct Servers
{
  std::atomic<bool> stop{false}, error{false};
  std::atomic<bool> delay_plan_cancel{false};
  std::atomic<int> plan_gate{1}, smooth_gate{1}, reached{0};
  std::atomic<int> plans_a{0}, plans_b{0}, smooth_a{0}, smooth_b{0};
  std::atomic<int> follows_a{0}, follows_b{0}, old_result_sent{0}, cancellations{0};
  rclcpp::Node::SharedPtr node;
  rclcpp_action::Server<Compute>::SharedPtr compute;
  rclcpp_action::Server<Smooth>::SharedPtr smooth;
  rclcpp_action::Server<Follow>::SharedPtr follow;
  std::mutex mutex;
  std::vector<std::thread> workers;

  template<class Action, class Execute>
  typename rclcpp_action::Server<Action>::SharedPtr make(const char * name, Execute execute)
  {
    using Handle = rclcpp_action::ServerGoalHandle<Action>;
    return rclcpp_action::create_server<Action>(node, name,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename Action::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<Handle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this, execute](std::shared_ptr<Handle> handle) {
        std::lock_guard<std::mutex> lock(mutex);
        workers.emplace_back([this, execute, handle]() {
            try {execute(handle);} catch (const std::exception & e) {
              error = true; std::cerr << "fake server: " << e.what() << '\n';
            }
          });
      });
  }
  template<class Action>
  bool wait(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> & h,
    const std::atomic<int> & gate, int id)
  {
    while (!stop) {
      const bool hold_cancel = std::is_same<Action, Compute>::value && delay_plan_cancel && gate < id;
      if (h->is_canceling() && !hold_cancel) {
        h->canceled(std::make_shared<typename Action::Result>()); ++cancellations; return false;
      }
      if (gate >= id) {return true;}
      std::this_thread::sleep_for(2ms);
    }
    return false;
  }
  explicit Servers(const std::string & ns)
  {
    node = std::make_shared<rclcpp::Node>("fake_servers", ns);
    compute = make<Compute>("compute_path_to_pose", [this](auto h) {
        int id = static_cast<int>(h->get_goal()->goal.pose.position.x);
        ++(id == 1 ? plans_a : plans_b);
        if (!wait<Compute>(h, plan_gate, id)) {return;}
        auto result = std::make_shared<Compute::Result>();
        result->path.header = h->get_goal()->goal.header;
        result->path.poses = {goal(0), h->get_goal()->goal}; h->succeed(result);
      });
    smooth = make<Smooth>("smooth_path", [this](auto h) {
        int id = static_cast<int>(h->get_goal()->path.poses.back().pose.position.x);
        ++(id == 1 ? smooth_a : smooth_b);
        if (!wait<Smooth>(h, smooth_gate, id)) {return;}
        auto result = std::make_shared<Smooth::Result>();
        result->path = h->get_goal()->path; result->was_completed = true; h->succeed(result);
      });
    follow = make<Follow>("follow_path", [this](auto h) {
        int id = static_cast<int>(h->get_goal()->path.poses.back().pose.position.x);
        ++(id == 1 ? follows_a : follows_b);
        if (!wait<Follow>(h, reached, id)) {return;}
        h->succeed(std::make_shared<Follow::Result>());
        if (id == 1) {++old_result_sent;}
      });
  }
  void join()
  {
    stop = true;
    std::lock_guard<std::mutex> lock(mutex);
    for (auto & worker : workers) {if (worker.joinable()) {worker.join();}}
  }
};

class Clear : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("service_name")};}
  Status tick() override {return Status::SUCCESS;}
};

int main(int argc, char ** argv)
{
  check(argc == 5, "usage: test PLUGIN XML NAV2_LIB_DIR CASE");
  rclcpp::init(argc, argv);
  const std::string ns = "/codex_goal_reset_" + std::to_string(getpid());
  Servers servers(ns);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(servers.node);
  std::thread spin([&]() {executor.spin();});
  int result = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>("bt_client", ns);
    BT::BehaviorTreeFactory factory;
    for (const auto * name : {"pipeline_sequence", "recovery_node", "rate_controller",
      "compute_path_to_pose_action", "smooth_path_action", "follow_path_action"})
    {
      factory.registerFromPlugin(std::string(argv[3]) + "/libnav2_" + name + "_bt_node.so");
    }
    factory.registerFromPlugin(argv[1]); factory.registerNodeType<Clear>("ClearEntireCostmap");
    auto bb = BT::Blackboard::create();
    bb->set("node", node); bb->set("goal", goal(1)); bb->set("path", nav_msgs::msg::Path{});
    bb->set("server_timeout", std::chrono::milliseconds(100));
    bb->set("bt_loop_duration", std::chrono::milliseconds(10));
    bb->set("wait_for_service_timeout", std::chrono::milliseconds(2000));
    bb->set("number_recoveries", 0);
    std::ifstream input(argv[2]); std::stringstream xml; xml << input.rdbuf();
    auto tree = factory.createTreeFromText(xml.str(), bb);
    Status status = Status::IDLE;
    auto until = [&](auto predicate, bool allow_success = false) {
        auto start = std::chrono::steady_clock::now();
        while (!predicate()) {
          status = tree.tickRoot();
          check(!servers.error, "fake action server threw");
          check(status != Status::FAILURE, "unexpected BT failure");
          check(allow_success || status != Status::SUCCESS, "old result completed a new goal");
          check(std::chrono::steady_clock::now()-start < 5s, "async test timed out");
          std::this_thread::sleep_for(10ms);
        }
      };
    until([&]() {return servers.follows_a.load() > 0;});
    const std::string scenario = argv[4];
    if (scenario == "planner_running" || scenario == "delayed_cancel") {
      servers.delay_plan_cancel = scenario == "delayed_cancel";
      servers.plan_gate = 0; until([&]() {return servers.plans_a.load() > 1;});
      // Consume the acknowledgement so halt exercises the existing active
      // goal cancellation/result wait, not just the unacknowledged-goal case.
      status = tree.tickRoot();
    }
    if (scenario == "smoother_running") {
      servers.smooth_gate = 0; until([&]() {return servers.smooth_a.load() > 1;});
    }
    // Queue old success without ticking its BtActionNode result callback.
    servers.reached = 1;
    if (scenario == "old_result_queued" || scenario == "legacy_race") {
      auto start = std::chrono::steady_clock::now();
      while (!servers.old_result_sent) {
        check(std::chrono::steady_clock::now()-start < 1s, "old result not sent");
        std::this_thread::sleep_for(2ms);
      }
      std::this_thread::sleep_for(10ms);
    }
    bb->set("goal", goal(2)); servers.plan_gate = scenario == "smoother_running" ? 2 : 1;
    if (scenario == "delayed_cancel") {servers.plan_gate = 0;}
    servers.smooth_gate = 1;
    if (scenario == "legacy_race") {
      until([&]() {return status == Status::SUCCESS;}, true);
      check(servers.follows_b == 0, "legacy race did not finish on old follow");
    } else {
      until([&]() {return servers.plans_b.load() > 0;});
      auto hold = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now()-hold < 150ms) {
        status = tree.tickRoot();
        check(status == Status::RUNNING && servers.follows_b == 0,
          "pending new planning/smoothing inherited old success");
        std::this_thread::sleep_for(10ms);
      }
      servers.plan_gate = servers.smooth_gate = 2;
      until([&]() {return servers.follows_b.load() > 0;});
      check(status == Status::RUNNING, "B completed before arrival");
      servers.reached = 2;
      until([&]() {return status == Status::SUCCESS;}, true);
    }
    tree.haltTree();
    std::cout << "PASS async " << scenario << " plan_B=" << servers.plans_b
              << " follow_B=" << servers.follows_b << " cancel_ack=" << servers.cancellations << '\n';
  } catch (const std::exception & e) {
    std::cerr << "FAIL async " << argv[4] << ": " << e.what() << '\n'; result = 1;
  }
  servers.join(); executor.cancel(); spin.join(); rclcpp::shutdown(); return result;
}
