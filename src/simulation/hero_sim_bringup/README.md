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
