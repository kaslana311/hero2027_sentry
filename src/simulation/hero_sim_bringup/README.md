# HERO perception simulation bringup

This package is a simulation-only entry point. It does not start real Livox,
serial, Odin, or Zenoh drivers and does not change the real-robot launch path.

The first stage intentionally bypasses Point-LIO. Gazebo ground-truth odometry
provides the planar `map -> odom -> base_footprint` transform. A separate
stamped 6-D chassis pose registers each simulated scan into `map` before it
is exposed as `cloud_registered`. This isolates `dog_map`, the
custom Nav2 costmap layers, inflation, planning, and control.

## Build

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select hero_sim_bringup
source install/setup.bash
```

## Run

Start Gazebo first from the simulator repository. Use the updated startup
scripts: they build the separate 6-D truth topic into the local derivative
image. The published official base image is not modified. In a second terminal:

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

- navigation input: `chassis_odometry_gt` (unchanged planar pose)
- cloud pose input: `chassis_odometry_3d_gt` (full position, roll/pitch/yaw)
- single-lidar input: `livox/lidar`
- dual-lidar inputs: `livox/lidar_left`, `livox/lidar_right`
- generated: `odometry/filtered`, `cloud_registered`, TF
- perception: `rog_map/inf_occ`
- Nav2: `local_costmap/costmap`, `global_costmap/costmap`
- control relay: `cmd_vel_nav2_result` -> `cmd_vel`

The dual-lidar simulation transforms each standard `PointCloud2` stream into
`map` at its own acquisition stamp, then merges them inside `sim_ground_truth_bridge`. It
does not yet synthesize Livox `CustomMsg` or exercise the real `lidar_merge`
and Point-LIO timing path.

## Acquisition-time cloud registration (2026-09-30)

The old bridge forwarded single-lidar scans in the sensor frame and merged
dual scans in `chassis`. DOG map then used the latest TF, not the scan's TF.
Moreover, the upstream `MecanumDrive2` publishes only X/Y/yaw on its original
odometry topic. Timing alone cannot recover the missing slope attitude.

The simulator now publishes `chassis_odometry_3d_gt` independently. The bridge
buffers those poses and interpolates at each scan's acquisition stamp. Single
lidar mounts come from robot TF; dual mounts keep the existing calibrated
parameters. `lidar_sensor_pose` accounts for the macro's 0.03 m ray-origin
offset above each MID360 link. Output XYZ is in `map`; acquisition stamps,
intensity and other fields are preserved. A merged scan uses the newer stamp,
but each constituent was already registered at its own stamp.

This does not change navigation's planar odometry, motor/PID control, DOG map,
or the real-robot Point-LIO path. It is frame-level registration, not Livox
per-point motion compensation. `cloud_target_frame` still denotes the chassis
mounting reference; it no longer denotes the published cloud frame.

Missing acquisition-time pose/TF waits in a bounded, nonblocking queue
(`cloud_queue_size=20`, `cloud_tf_wait_sec=0.5` wall-clock seconds). Expired or
zero-stamped scans are dropped; there is no latest-pose fallback. If clouds
stop and the bridge reports missing 3-D truth, rebuild/restart the simulator
through its updated script. Do not reuse an old running container.

Regression verification (no Gazebo, RViz or motor commands):

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select hero_sim_bringup --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select hero_sim_bringup
colcon test-result --test-result-base build/hero_sim_bringup
```

Eight black-box tests cover delayed scans after a tilt, delayed poses, dual
acquisition stamps, pose interpolation, missing/zero stamps, map origin, and
queue overflow. They pass on the host. Image rebuilding and repeated physical
slope/tunnel navigation still require field validation; synthetic tests are
not a claim of successful full simulation runs.

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
