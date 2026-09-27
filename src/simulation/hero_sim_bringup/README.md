# HERO perception simulation bringup

This package is a simulation-only entry point. It does not start real Livox,
serial, Odin, or Zenoh drivers and does not change the real-robot launch path.

The first stage intentionally bypasses Point-LIO. Gazebo ground-truth odometry
provides the `map -> odom -> base_footprint` transform, while the simulated
point cloud is exposed as `cloud_registered`. This isolates `dog_map`, the
custom Nav2 costmap layers, inflation, planning, and control.

## Build

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select hero_sim_bringup
source install/setup.bash
```

## Run

Start Gazebo first from the simulator repository. In a second terminal:

```bash
source /opt/ros/humble/setup.bash
source "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))/install/setup.bash"
ros2 launch hero_sim_bringup perception_truth_sim.launch.py costmap_mode:=combined
```

The default remains the RMUC 2025 static map. For the RMUC 2026 Gazebo world
and dual-lidar image, use the matching map explicitly:

```bash
ros2 launch hero_sim_bringup perception_truth_sim.launch.py \
  field:=rmuc_2026 \
  lidar_mode:=dual \
  costmap_mode:=combined
```

The RMUC 2026 map uses 0.05 m cells and is registered to the simulator's
current `(-10, 0, yaw=0)` spawn while the truth bridge zeroes the first
odometry sample. See `maps/rmuc_2026_SOURCE.md` before changing either the
spawn or `zero_at_first_odometry`.

Available modes:

- `combined`: static map + `fast_layer` + inflation
- `static_only`: static map + inflation
- `dynamic_only`: `fast_layer` + inflation

For a costmap-only inspection that cannot move the robot, add
`relay_cmd_vel:=false`. RViz can be disabled with `use_rviz:=false`.

## Expected interfaces

Under `/red_standard_robot1`:

- common input: `chassis_odometry_gt`
- single-lidar input: `livox/lidar`
- dual-lidar inputs: `livox/lidar_left`, `livox/lidar_right`
- generated: `odometry/filtered`, `cloud_registered`, TF
- perception: `rog_map/inf_occ`
- Nav2: `local_costmap/costmap`, `global_costmap/costmap`
- control relay: `cmd_vel_nav2_result` -> `cmd_vel`

The dual-lidar simulation currently transforms both standard `PointCloud2`
streams into `chassis` and merges them inside `sim_ground_truth_bridge`. It
does not yet synthesize Livox `CustomMsg` or exercise the real `lidar_merge`
and Point-LIO timing path.

## Raised-edge stall recovery

The arena is a detailed 3-D collision mesh while Nav2 plans in a 2-D
costmap. A low mesh edge can therefore stop the physical chassis even when
the planned path and costmap cell are valid. The simulation behavior tree now
distinguishes that case from a costmap overlap:

- `IsStuckCondition` latches a stall when `cmd_vel_nav2_result` requests at
  least 0.08 m/s for 1.5 s but `odometry/filtered` moves less than 0.03 m;
- the simulator progress checker allows slow narrow-gap motion, but starts
  recovery after 5 s without 0.10 m of progress;
- recovery first performs one local clear, then requests a conservative
  0.30 m reverse at 0.15 m/s;
- if the ESDF says the robot is already in free space, `BackUpFreeSpace` now
  executes Nav2's collision-checked reverse instead of reporting success
  without motion.

Expected diagnostic messages are:

```text
IsStuckCondition: commanded-motion stall confirmed (...)
BackUpFreeSpace: costmap reports free space despite a recovery request; using collision-checked reverse fallback (...)
```

These changes improve recovery; they do not prove that the simulated ramp
geometry matches the real chassis clearance. Do not simplify the world
collision mesh until the robot collision dimensions and real-field/CAD
geometry have been compared.

## Costmap boundary

The current `combined` mode intentionally keeps the established baseline:

```text
local costmap : static_layer + fast_layer + inflation_layer
global costmap: static_layer + fast_layer + inflation_layer
```

The planned experiment is global static planning plus a rolling local costmap
that contains only `fast_layer` and inflation. That split is not enabled yet.
Before enabling it, verify where the local dynamic obstacle costs constrain
the MINCO/MPC trajectory: `HeroMpcController` currently receives a costmap
pointer but does not query it in its control calculation.

## Repository and experiment notes

The simulator image, dual-lidar overlays, project status, and rollback guide
are maintained in:

```text
/home/apocalypse_linux_00/build_sim_docker
```

Read `PROJECT_PROGRESS.md` before changing the experiment architecture. Keep
simulation-only changes on the `sim-perception-integration` branch and avoid
mixing real-robot driver changes into the same commit.
