# HERO perception simulation bringup

This package is a simulation-only entry point. It does not start real Livox,
serial, Odin, or Zenoh drivers and does not change the real-robot launch path.

2026-10-11: algorithm-only branch `codex/perception-navigation-iteration` adds
an opt-in high-column DOG output filter. Existing startup commands leave it off;
`filter_high_columns:=true high_column_min_z:=0.30` enables an absolute-map-Z
experiment, not a validated universal clearance rule. The simulator is unchanged.
The five-item roadmap, limitations, commands and acceptance checklist are in
[PERCEPTION_NAVIGATION_ITERATION.md](./PERCEPTION_NAVIGATION_ITERATION.md).

The first stage intentionally bypasses Point-LIO. Gazebo ground-truth odometry
provides the planar `map -> odom -> base_footprint` transform. A separate
stamped 6-D chassis pose registers each simulated scan into `map` before it
is exposed as `cloud_registered`. This isolates `dog_map`, the
custom Nav2 costmap layers, inflation, planning, and control.

On 2026-10-01 the user confirmed smooth RMUC 2026 tunnel navigation with
the default tunnel profile, dual lidar and combined costmaps. The paired
`sim/truth-tunnel-baseline-v1` tag preserves that truth-based baseline;
`codex/point-lio-sim` now includes an isolated Point-LIO shadow entry point.
Navigation in that cloud-only baseline uses truth; Point-LIO has not taken
over its navigation. The new full-navigation experiment is documented in
[POINT_LIO_NAVIGATION.md](./POINT_LIO_NAVIGATION.md). On 2026-10-08 the user confirmed good clouds, negligible observed
drift and normal tunnel traversal for the cloud-only stage. The paired
`sim/point-lio-cloud-baseline-v1` tag archives that qualitative feedback, not
full LIO navigation takeover or quantitative route acceptance. See
[POINT_LIO.md](./POINT_LIO.md) and [SIMULATION_BASELINE.md](../../../SIMULATION_BASELINE.md).

## Registered-cloud deployment (2026-10-02)

After the filter/gravity fixes the user reports markedly reduced drift, not
quantified full-route acceptance. `perception_point_lio_sim.launch.py` starts
navigation and Point-LIO together in one host terminal. DOG map consumes
`pointlio/cloud_registered_map`; navigation pose remains ground truth. A single,
explicit truth-seeded initial transform aligns the local LIO world with map;
later drift is retained, with no automatic truth-cloud fallback. Do not also run
the old truth or standalone shadow algorithm launches. See
[POINT_LIO_CLOUD.md](./POINT_LIO_CLOUD.md) for commands, checks and rollback.
Docker, planner/controller settings and the original truth entry are unchanged.
A subsequent runtime check found that the new adapter's best-effort output did
not match DOG map's reliable subscription. `25336a1` fixes only that output QoS;
42 isolated regressions now include actual DOG map receipt and nonempty occupancy.
The adapter's publishing counter alone is not downstream delivery evidence.
The stage is now archived on `codex/point-lio-sim` in both repositories. This
archive changes documentation only, leaves truth-baseline tags/stable branches
untouched and does not require recompilation. All startup commands are collected
in `/home/apocalypse_linux_00/build_sim_docker/docs/STARTUP.md`.

## Build

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
./build_hero2026_safe.sh --jobs 4
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Use the existing safe-build script for this workspace's punctuation-heavy path
and dependency handling; retain the original build/install directories. For
an installed-artifact audit without compiling, run
`./build_hero2026_safe.sh --verify-only`. It does not build changed sources.
Documentation-only commits or initial branch creation need no rebuild.

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

The truth-based dual-lidar simulation transforms each standard `PointCloud2` stream into
`map` at its own acquisition stamp, then merges them inside `sim_ground_truth_bridge`. It
does not yet synthesize Livox `CustomMsg` or exercise the real `lidar_merge`
and Point-LIO timing path. The new shadow entry separately adapts raw scans
to `CustomMsg` snapshots; it does not replace or change this truth launch.

## Point-LIO shadow (2026-10-01)

Keep Gazebo and the truth navigation launch above. In a third clean terminal:

```bash
source /opt/ros/humble/setup.bash
source "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))/install/setup.bash"
ros2 launch hero_sim_bringup point_lio_shadow.launch.py use_rviz:=true
```

The input adapter and estimator consume only raw dual lidar and the right IMU;
they never consume truth odometry or send motor commands. A separate observer
reads truth only for display/evaluation, without feedback. Outputs are isolated under
`/red_standard_robot1/pointlio`, including `odometry`, `cloud_registered` and
private `tf`. The original DOG map and navigation still consume truth data.
The optional second RViz now shows the existing static map in the fixed `map`
frame: yellow origin, green truth chassis, blue estimated chassis and their trails.
Camera tools and the Views panel enable zoom/pan; clouds are optional, off by default.
Alignment is frozen at the first time-matched pair, including the IMU-to-chassis
offset; subsequent drift is retained. Initial overlap is by construction, not a
localization accuracy result. Only private TF is used for this display alignment.
After updating, restart both host algorithm launches and re-source the install;
the truth launch adds pose-only `odometry/truth_3d` diagnostics at its existing origin.
Leave the robot still for IMU initialization before moving it. Restart this
shadow launch after a simulator time reset. No new Docker image is needed.

The simulator uses rendered whole-frame snapshots, not Livox scan timing:
only exactly equal acquisition stamps are merged, with zero per-point offset.
The initial SI IMU and output-frame adaptations were simulation-only settings.
After stationary divergence was observed, `1c14dd3` corrected the shared Point-LIO
batch measurement-noise weighting on this experimental branch; `94495a3` added
read-only scan diagnostics, and `e447239` aligned only the snapshot YAML gravity
with the measured 9.8 m/s². Real-robot YAML, stable branches, truth navigation,
DOG map, Nav2/MINCO/MPC and Docker images remain unchanged. The shared filter
formula change is not a simulation-only parameter. Restart the shadow launch
after re-sourcing to load it; actual slope/tunnel motion acceptance is pending.
Full contract, limitations, tests and pending runtime checks are in
[POINT_LIO.md](./POINT_LIO.md).

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
source install/setup.bash
export BRIDGE_EXECUTABLE="$(ros2 pkg prefix hero_sim_bringup)/lib/hero_sim_bringup/sim_ground_truth_bridge"
export POINT_LIO_INPUT_EXECUTABLE="$(ros2 pkg prefix hero_sim_bringup)/lib/hero_sim_bringup/sim_point_lio_input_bridge"
export POINT_LIO_EXECUTABLE="$(ros2 pkg prefix point_lio)/lib/point_lio/pointlio_mapping"
export EVALUATION_EXECUTABLE="$(ros2 pkg prefix hero_sim_bringup)/lib/hero_sim_bringup/sim_point_lio_evaluation"
export POINT_LIO_CLOUD_EXECUTABLE="$(ros2 pkg prefix hero_sim_bringup)/lib/hero_sim_bringup/sim_point_lio_cloud_bridge"
export DOG_MAP_EXECUTABLE="$(ros2 pkg prefix dog_map)/lib/dog_map/dog_map_node_lidars"
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=174 python3 -m pytest src/simulation/hero_sim_bringup/test -q
```

Eight black-box tests cover delayed scans after a tilt, delayed poses, dual
acquisition stamps, pose interpolation, missing/zero stamps, map origin, and
queue overflow. Together with three BT contract tests, all 11 pass on the host.
Seven new shadow tests (including real Point-LIO initialization on synthetic
planes) also passed initially: 18 total. Comparison tests now additionally cover
the unchanged map origin, fixed one-time alignment, chassis extrinsics, acquisition-time
interpolation, missing truth, time reset and visualization tools (see POINT_LIO.md).
The 2026-10-01 truth runtime report is recorded
separately from these synthetic tests; shadow motion validation is still pending;
per-run action results, error statistics and image ID have not been collected.

## Raised-edge stall recovery

The arena is a detailed 3-D collision mesh while Nav2 plans in a 2-D
costmap. A low mesh edge can therefore stop the physical chassis even when
the planned path and costmap cell are valid. The simulation behavior tree now
distinguishes that case from a costmap overlap:

- `IsStuckCondition` latches a stall when `cmd_vel_nav2_result` requests at
  least 0.08 m/s for 1.5 s but `odometry/filtered` moves less than 0.03 m;
- the simulator progress checker allows slow narrow-gap motion, but starts
  recovery after 5 s without 0.10 m of progress;
- recovery allows up to 10 local-clear retries for FollowPath, then requests a conservative
  0.30 m reverse at 0.15 m/s;
- if the ESDF says the robot is already in free space, `BackUpFreeSpace` now
  executes Nav2's collision-checked reverse instead of reporting success
without motion.

On 2026-09-30 the simulation tree's overall retry count was restored from 6
to the previous 150, and FollowPath retries from 1 to 10. These were the
overly aggressive parts of `fd84a3b`; physical stall detection, progress
checking and collision-checked reverse remain. The old `Wait="0.2"` is also
corrected to `Wait="1"`: Humble's integer-seconds input had truncated 0.2 to
zero, so clearing/replanning could exhaust retries without waiting for a
transient obstacle to disappear. This is a forward correction, not a revert
of the whole recovery implementation. Three BT regression checks accompany
the eight cloud-registration tests.

One second is a genuine minimum wait, not an asserted DOG-map expiration
time: occupancy decays over updates/log odds and fresh observations can
extend it. Restored retries also mean persistent failures take longer to
abort; users can still cancel the navigation goal. Clearing is not a
substitute for correct cloud registration or a guarantee of passability.

Expected diagnostic messages are:

```text
IsStuckCondition: commanded-motion stall confirmed (...)
BackUpFreeSpace: costmap reports free space despite a recovery request; using collision-checked reverse fallback (...)
```

These changes do not prove that simulated ramp contacts match the real chassis.
The separately reviewed 2026 world cleanup removes decorative fragments only,
not structural ramps, walls or tunnel roofs. It is not a manifold repair or
a general permission to simplify field geometry.

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
the stable truth baseline on `sim-perception-integration`; develop the next
Point-LIO stage on `codex/point-lio-sim` in both repositories. Preserve this
truth launch and avoid mixing real-robot driver changes into the same commit.
The combined costmap remains unchanged during that stage; the costmap split
above is a separate future experiment. Baseline preparation introduced no code;
the subsequent shadow commit introduces host-side code but no new Docker image.
## 2026-10-08：完整 Point-LIO 接管实验入口（C2）

新增 `navigation_point_lio_sim.launch.py`，导航定位与 DOG map 点云共同来自
Point-LIO。分支 `codex/point-lio-navigation`，从 C1 已验收标签新建；
旧入口保留，不修改规划/控制参数或镜像。C2 尚待用户联合实跑，不能将 C1
反馈直接当作 C2 验收。完整命令、初始化和切回方法见
[POINT_LIO_NAVIGATION.md](./POINT_LIO_NAVIGATION.md)。
