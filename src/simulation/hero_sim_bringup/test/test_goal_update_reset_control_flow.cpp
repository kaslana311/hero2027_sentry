// Real installed Nav2 control plugins; deterministic action leaves; no ROS I/O.
#include <chrono>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

using Pose = geometry_msgs::msg::PoseStamped;
using Path = nav_msgs::msg::Path;
using Status = BT::NodeStatus;
struct State
{
  bool plan_ready{true}, smooth_ready{true}, plan_failure{false};
  int plan_calls{0}, smooth_calls{0}, follow_calls{0}, clear_calls{0};
  int plan_halts{0}, smooth_halts{0}, follow_halts{0};
  Pose planned, followed;
  std::set<double> reached;
} s;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}
Pose destination(double x)
{
  Pose p; p.header.frame_id = "map"; p.pose.position.x = x; p.pose.orientation.w = 1.;
  return p;
}
class Compute : public BT::ActionNodeBase
{
public:
  using BT::ActionNodeBase::ActionNodeBase;
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<Pose>("goal"), BT::OutputPort<Path>("path"),
      BT::InputPort<std::string>("planner_id")};
  }
  Status tick() override
  {
    if (status() == Status::IDLE) {getInput("goal", goal_); ++s.plan_calls;}
    if (s.plan_failure) {return Status::FAILURE;}
    if (!s.plan_ready) {return Status::RUNNING;}
    s.planned = goal_;
    Path path; path.header = goal_.header; path.poses.push_back(destination(0));
    path.poses.push_back(goal_); setOutput("path", path);
    return Status::SUCCESS;
  }
  void halt() override {++s.plan_halts; setStatus(Status::IDLE);}
private:
  Pose goal_;
};
class Smooth : public BT::ActionNodeBase
{
public:
  using BT::ActionNodeBase::ActionNodeBase;
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<Path>("unsmoothed_path"), BT::OutputPort<Path>("smoothed_path"),
      BT::InputPort<std::string>("smoother_id")};
  }
  Status tick() override
  {
    if (status() == Status::IDLE) {getInput("unsmoothed_path", path_); ++s.smooth_calls;}
    if (!s.smooth_ready) {return Status::RUNNING;}
    setOutput("smoothed_path", path_); return Status::SUCCESS;
  }
  void halt() override {++s.smooth_halts; setStatus(Status::IDLE);}
private:
  Path path_;
};
class Follow : public BT::ActionNodeBase
{
public:
  using BT::ActionNodeBase::ActionNodeBase;
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<Path>("path"), BT::InputPort<std::string>("controller_id")};
  }
  Status tick() override
  {
    ++s.follow_calls; Path path; getInput("path", path);
    require(!path.poses.empty(), "FollowPath received an empty path");
    s.followed = path.poses.back();
    return s.reached.count(s.followed.pose.position.x) ? Status::SUCCESS : Status::RUNNING;
  }
  void halt() override {++s.follow_halts; setStatus(Status::IDLE);}
};
class Clear : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("service_name")};}
  Status tick() override {++s.clear_calls; return Status::SUCCESS;}
};

int main(int argc, char ** argv)
{
  require(argc == 5, "usage: test PLUGIN XML NAV2_LIBRARY_DIR CASE");
  BT::BehaviorTreeFactory factory;
  for (const auto * name : {"pipeline_sequence", "recovery_node", "rate_controller"}) {
    factory.registerFromPlugin(std::string(argv[3]) + "/libnav2_" + name + "_bt_node.so");
  }
  factory.registerFromPlugin(argv[1]);
  factory.registerNodeType<Compute>("ComputePathToPose");
  factory.registerNodeType<Smooth>("SmoothPath");
  factory.registerNodeType<Follow>("FollowPath");
  factory.registerNodeType<Clear>("ClearEntireCostmap");
  std::ifstream input(argv[2]); std::stringstream xml; xml << input.rdbuf();
  auto bb = BT::Blackboard::create(); bb->set("goal", destination(1)); bb->set("path", Path{});
  auto tree = factory.createTreeFromText(xml.str(), bb);
  const std::string scenario = argv[4];
  auto tick = [&]() {return tree.tickRoot();};
  auto setGoal = [&](double x) {bb->set("goal", destination(x));};
  auto waitRate = []() {std::this_thread::sleep_for(std::chrono::milliseconds(350));};
  auto finish = [&](double x) {
      s.plan_ready = s.smooth_ready = true;
      require(tick() == Status::RUNNING, "new destination completed before it was reached");
      require(s.followed.pose.position.x == x, "old destination still being followed");
      s.reached.insert(x);
      require(tick() == Status::SUCCESS, "legitimate new-goal completion failed");
    };

  if (scenario == "normal_arrival") {
    s.reached.insert(1);
    require(tick() == Status::SUCCESS, "ordinary arrival regressed");
  } else {
    require(tick() == Status::RUNNING && s.followed.pose.position.x == 1, "baseline not running");
    if (scenario == "old_safe_ready") {
      setGoal(2); s.reached.insert(1); waitRate();
      require(tick() == Status::RUNNING && s.followed.pose.position.x == 2,
        "unprotected normal preemption should work when the new path is ready first");
      s.reached.insert(2); require(tick() == Status::SUCCESS, "normal preemption cannot finish");
    } else if (scenario == "old_race" || scenario == "old_race_inflight") {
      setGoal(2); s.plan_ready = false; s.reached.insert(1);
      if (scenario == "old_race_inflight") {waitRate();}
      require(tick() == Status::SUCCESS, "unprotected counterfactual no longer reproduces");
      require(s.followed.pose.position.x == 1, "counterfactual did not follow the old goal");
    } else if (scenario == "unchanged_goal_replanning") {
      s.plan_ready = false; waitRate();
      const int before = s.follow_calls;
      for (int i = 0; i < 4; ++i) {require(tick() == Status::RUNNING, "replanning failed");}
      require(s.follow_calls == before + 4 && s.follow_halts == 0, "same goal no longer pipelines");
      s.plan_ready = true; require(tick() == Status::RUNNING, "same-goal replanning failed");
    } else if (scenario == "stamp_only") {
      auto p = destination(1); p.header.stamp.sec = 42; bb->set("goal", p);
      const int before = s.plan_calls;
      require(tick() == Status::RUNNING && s.follow_halts == 0, "restamp stopped old follow");
      require(s.plan_calls == before, "restamp bypassed the original rate controller");
    } else if (scenario == "cancel_restart") {
      tree.haltTree(); setGoal(2); s.plan_ready = false; s.reached.insert(1);
      const int before = s.follow_calls;
      require(tick() == Status::RUNNING && s.follow_calls == before, "cancel leaked old follow");
      finish(2);
    } else if (scenario == "new_goal_after_success") {
      s.reached.insert(1); require(tick() == Status::SUCCESS, "first goal failed");
      tree.haltTree(); setGoal(2); s.plan_ready = false;
      require(tick() == Status::RUNNING, "old success ended the next navigation"); finish(2);
    } else {
      if (scenario == "planner_running") {
        s.plan_ready = false; waitRate(); require(tick() == Status::RUNNING, "planner not running");
      }
      if (scenario == "smoother_running") {
        s.smooth_ready = false; waitRate(); require(tick() == Status::RUNNING, "smoother not running");
      }
      const int old_follow_calls = s.follow_calls;
      const int old_plan_calls = s.plan_calls;
      setGoal(2); s.reached.insert(1); s.plan_ready = false;
      if (scenario == "yaw_change" || scenario == "frame_change") {
        auto p = destination(1);
        if (scenario == "yaw_change") {p.pose.orientation.z = .5; p.pose.orientation.w = .86602540378;}
        else {p.header.frame_id = "other_map";}
        bb->set("goal", p);
      }
      require(tick() == Status::RUNNING, "new goal inherited old follow success");
      require(s.follow_calls == old_follow_calls, "old follow was reticked during new planning");
      require(s.plan_calls == old_plan_calls + 1, "new goal did not restart planning immediately");
      require(s.follow_halts == 1, "old follow was not halted exactly once");
      if (scenario == "planner_running") {require(s.plan_halts >= 1, "old planner not halted");}
      if (scenario == "smoother_running") {require(s.smooth_halts >= 1, "old smoother not halted");}
      if (scenario == "many_goals") {
        setGoal(3); s.reached.insert(2);
        require(tick() == Status::RUNNING && s.follow_calls == old_follow_calls, "B leaked into C");
        finish(3);
      } else if (scenario == "planning_failure") {
        s.plan_failure = true; Status out = Status::RUNNING;
        for (int i = 0; i < 5 && out == Status::RUNNING; ++i) {out = tick();}
        require(out == Status::FAILURE && s.follow_calls == old_follow_calls, "failure masked by old success");
      } else if (scenario == "yaw_change" || scenario == "frame_change") {
        // The destination changed even though X did not. Ignore the old reached
        // flag here; this case tests cancellation/reset, not a fake goal checker.
        s.reached.clear(); s.plan_ready = s.smooth_ready = true;
        require(tick() == Status::RUNNING, "changed orientation/frame failed");
      } else {
        finish(2);
      }
    }
  }
  tree.haltTree();
  std::cout << "PASS " << scenario << " plans=" << s.plan_calls << " follows=" << s.follow_calls
            << " follow_halts=" << s.follow_halts << "\n";
}
