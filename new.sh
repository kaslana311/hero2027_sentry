#!/bin/bash
#gnome-terminal -- bash -i -c "cd /home/dji/hero2026_-sentry && ./startup_system_try.sh; exec bash"
# 1. 核心：先进入原脚本所在的目录
# 这样原脚本里的所有相对路径就都能找到了
cd /home/dji/hero2026_-sentry/bash
  plugin: "nav2_mppi_controller::MPPIController"
      # param 
      publish_critics_stats: True
      adjustThre: 14.0 # slopeDegree larger than this value will trigger the adjust heading behavior
      minAngleDiff: 365.0 # if the angle difference is larger than this value, the robot will continue to adjust heading
      adjustHeadingSpeed: 0.2 # angular speed to adjust heading
      # 
      time_steps: 24 # prediction horizon = time_steps x model_dt 
      model_dt: 0.05 # =1/controller_frequency
      batch_size: 1000 # Count of randomly sampled candidate trajectories from current optimal control sequence in a given iteration. 1000 @ 50 Hz or 2000 @ 30 Hz seems to produce good results.
      vx_std: 0.78
      vy_std: 0.78
      wz_std: 0.1
      vx_max: 2.0 # prediction horizon x vx_max = 3.0 m -->local costmap dim should larger than 3.0 x 2 = 6m
      vx_min: -2.0 # Maximum reverse velocity (m/s).
      vy_max: 2.0
      wz_max: 0.1
      iteration_count: 1 # Recommended to remain as 1 and instead prefer larger batch sizes.
      prune_distance: 4.0 # proportion to your maximum velocity and prediction horizon.
      transform_tolerance: 0.1 # Time tolerance for data transformations with TF (s).
      temperature: 0.3 # 0 mean use control with best cost, huge value will lead to just taking mean of all trajectories without cost consideration.
      gamma: 0.07 # A trade-off between smoothness (high) and low energy (low). This is a complex parameter that likely won’t need to be changed from the default.
      motion_model: "Omni"
      visualize: true  # set to true to visualize the trajectories, but it will slow down the computation by a lot
      # publish_critics_stats: true
      reset_period: 1.0 # Required time of inactivity to reset optimizer (only in Humble
      regenerate_noises: false
      retry_attempt_limit: 3 # Number of attempts to find feasible trajectory on failure for soft-resets before reporting total failure.
      TrajectoryVisualizer:
        trajectory_step: 5
        time_step: 3
      # AckermannConstraints:
      #   min_turning_r: 0.2
      critics: ["ObstaclesCritic", "GoalCritic", "PathAlignCritic", "PathFollowCritic", "TwirlingCritic"]
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 0.7 # Minimal distance (m) between robot and goal above which goal distance cost considered. It is wise to start with this as being the same as your prediction horizon to have a clean hand-off with the path follower critic.
        # i.e. When this critic is add to the cost function
      PreferForwardCritic: # moving in the forward direction, rather than reversing.
        enabled: false
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 0.7 # Distance (m) between robot and goal to stop considering preferring forward and allow goal critics to take over.
      ObstaclesCritic: # 如果容易撞到低矮的障碍物，检查地形分析是否正确然后再调整这个参数
        enabled: true
        cost_power: 1
        repulsion_weight: 0.2 #! Higher radii should correspond to reduced repulsion_weight due to the penalty formation (e.g. inflation_radius - min_dist_to_obstacle). 
        # This is separated from the critical term to allow for fine tuning of obstacle behaviors with path alignment for dynamic scenes without impacting actions which may directly lead to near-collisions. This is applied within the inflation_radius distance from obstacles.
        critical_weight: 20.0
        consider_footprint: false # turn on this can increase computation cost
        collision_cost: 40.0
        collision_margin_distance: 0.05 # 0.05 # Margin distance (m) from collision to apply severe penalty, similar to footprint inflation. Between 0.05-0.2 is reasonable. Note that it will highly influence the controller not to enter spaces more confined than this, so ensure this parameter is set lower than the narrowest you expect the robot to need to traverse through.
        near_goal_distance: 0.1 # if the robot is within this distance of the goal, it will not be repelled by obstacles.
        inflation_radius: 0.4 # Radius to inflate costmap around lethal obstacles. This should be the same as for your inflation layer (Humble only)
        cost_scaling_factor: 10.0 # Exponential decay factor across inflation radius. This should be the same as for your inflation layer (Humble only)
        inflation_layer_name: ["local_inflation_layer","global_inflation_layer"] # 
      PathAlignCritic:
        enabled: true
        cost_power: 1
        cost_weight: 25.0
        max_path_occupancy_ratio: 0.001
        trajectory_point_step: 3 #?
        threshold_to_consider: 0.7 # 0.5 # Distance (m) between robot and goal to stop considering path alignment and allow goal critics to take over. So just keep it same as threshold_to_consider in GoalCritic.
        offset_from_furthest: 2 #  A selection of this value to be ~30% of the maximum velocity distance projected is good 
        # (e.g. if a planner produces points every 2.5cm, 60 can fit on the 1.5m local costmap radius. If the max speed is 0.5m/s with a 3s prediction time, 
        # then 20 points represents 33% of the maximum speed projected over the prediction horizon onto the path). When in doubt, 
        # prediction_horizon_s * max_speed / path_resolution / 3.0 is a good baseline. = 3 * 0.8 / 0.1 /3.0
        use_path_orientations: false
      PathFollowCritic: #? incentivizes making progress along the path, what is the diff between this and PathAlignCritic?
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        offset_from_furthest: 10 # Number of path points after furthest one any trajectory achieves to drive path tracking relative to.
        threshold_to_consider: 0.7 # That is to say, path following will be considered until the robot is within this distance of the goal, at which point the goal critic will take over.
      PathAngleCritic: # This critic penalizes trajectories at a high relative angle to the path. This helps the robot make sharp turns when necessary due to large accumulated angular errors.
        enabled: false
        cost_power: 1 
        cost_weight: 2.0
        offset_from_furthest: 4
        threshold_to_consider: 0.5
        max_angle_to_furthest: 1.0
        mode: 0
      TwirlingCritic:
        enabled: true
        twirling_cost_power: 1
        twirling_cost_weight: 10.0
# 2. 执行原脚本
# 使用 ./ 确保是在当前目录下执行
./startup_system_try.sh


