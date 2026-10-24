// Copyright 2026 HERO contributors
// SPDX-License-Identifier: Apache-2.0
#include "behaviortree_cpp_v3/bt_factory.h"
#include "hero_sim_bringup/reset_on_goal_update.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<hero_sim_bringup::ResetOnGoalUpdate>("ResetOnGoalUpdate");
}
