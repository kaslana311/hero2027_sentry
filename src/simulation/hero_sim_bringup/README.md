# HERO perception simulation bringup

This package is a simulation-only entry point. It does not start real Livox,
serial, Odin, or Zenoh drivers and does not change the real-robot launch path.

2026-10-20 follow-up: at the user's request, all simulation navigation entries
now default to `dog_resolution_xy:=0.05`. Only DOG's XY occupancy grid changes;
DOG Z remains 0.03 m, and PGM/Nav2 geometry, filtering rules and thresholds stay
unchanged. `dog_resolution_xy:=0.0` restores the old PGM-coupled behavior after
an algorithm restart. This is a startup-only setting, not a dynamic parameter.
See [DOG_XY_RESOLUTION.md](./DOG_XY_RESOLUTION.md) for scope, build and ablation.

2026-10-20: algorithm branch `codex/split-static-planning` adds opt-in
`costmap_mode:=split`. Both normal Nav2 maps and MINCO's map input remain
identical to `dynamic_only`. Smac alone uses an independent original-PGM plus
inflation map through `hero_static_prior_planner`; there is no DOG subscription
on that prior. At that commit the other three modes kept their pre-split
parameter snapshots; the later DOG XY setting above is a separate change.
Build, exact topic routing, commands, limitations and acceptance are documented
in [SPLIT_MODE.md](./SPLIT_MODE.md). Gazebo, Point-LIO, DOG filtering, MINCO/MPC
algorithms, existing rates and Docker images are unchanged. This is not a
collision-safety or full-route acceptance claim.

2026-10-14: the main `hero_sim.rviz` enables `Smoothed trajectory (MPC reference)`
and `Registered point cloud` by default in all three costmap modes. The former
subscribes to `FollowPath/reference_path`, the MPC horizon sampled from MINCO,
not the debug-only discrete `minco_trajectory` Path. It does not require
`publish_debug_visualizations:=true`; it appears during valid MPC execution.
The latter uses the actual perception input: `pointlio/cloud_registered_map`
in C1/C2 and, via an RViz-only remap, `cloud_registered` in the truth entry.
Both topics are robot-relative; Best Effort cloud subscription accepts either
publisher reliability. Close the old RViz without saving its old configuration
over the template, then reopen normally. There is no new launch argument,
algorithm change, Docker rebuild or C++ compilation requirement.

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
- `dynamic_only`: boundary-only static layer + `fast_layer` + inflation

### RMUC 2027 field (2026-10-16)

On branch `codex/rmuc-2027-field`, all three simulation navigation entries also
accept `field:=rmuc_2027`. Existing defaults and the 2025/2026 assets are unchanged.
Use the paired simulator's `scripts/start_sim_rmuc_2027_dual_lidar.sh`; the old
2026 script still selects the old field. Do not mix a 2027 world with a 2026 map.

The installed 1120x642, 2.5 cm PGM is now the user's manual revision of the redrawn
multilevel support-surface map, applied on 2026-10-16 without resampling or changing
coordinates. The paired simulator retains the generated baseline for provenance;
do not regenerate it over the user's source edits. The central highland remains
driveable and the central mechanism remains occupied. It replaces the incorrect
5 cm projection. See `maps/rmuc_2027_SOURCE.md` and `rmuc_2027_projection.json`
for provenance, mapping assumptions and runtime acceptance limits.
The PGM now has a 20 cm inward occupied rim (8 cells); dynamic_only retains
its existing 35 cm generated rim. The paired simulator also fixes one reviewed
1 mm welded ground strip without flattening the field. That mesh repair needs
the usual 2027 simulator script to rebuild/restart the image, then an algorithm
restart after settling. The original source STL and 2026 assets are retained.
The 2026-like open-area spawn is now (4.6,6.4), yaw=0; map origin is (-4.6,-6.4).
The field Z is realigned so this local floor remains near zero. The spawn update
requires both sides to restart; this later map-only deployment needs only an
algorithm restart when the simulator is already at the current spawn. No new
Docker image or launch argument is needed. At that import, DOG XY resolution also
followed the new 2.5 cm YAML (about 4x XY cells). The 2026-10-20 follow-up above
decouples it and defaults DOG XY back to 5 cm; range and filter thresholds stay unchanged.
Planner/controller/TF implementations and the old 2026 field are unchanged.
For initial tests use the existing C2 launch with `field:=rmuc_2027`,
`lidar_mode:=dual`, `costmap_mode:=dynamic_only`; its global window is 28x17 m,
and the inherited 35 cm boundary/PGM-reference visualization remain enabled.
Navigation pose/cloud are still Point-LIO, without ongoing truth correction.

The new floor has ~1.5 degree grade. DOG's absolute high-column Z threshold is
unchanged and is not relative to the local floor; revalidate the old threshold.
No simulator/robot-control, planning, MINCO, MPC, Point-LIO or DOG tuning is part
of this field import. It is not a claim of successful 2027 tunnel traversal.

### Full-field dynamic-only map with an arena rim (2026-10-14)

`dynamic_only` now uses a fixed global map covering the selected PGM's geometry
(origin and resolution), with a free interior and a 0.35 m inward occupied rim.
On 2026-10-15 the rim increased from 0.20 m by another 0.15 m on each side;
map geometry and inflation settings are unchanged. Restart the algorithm to regenerate it.
Original PGM interior occupancy values are NOT merged; FastLayer still supplies
the detected interior obstacles. Humble requires
integer-metre width/height parameters, so each extent is rounded up: RMUC 2026
uses 30 x 17 m at 0.05 m/cell, origin (-4.575, -6.4006561), covering the original
29.15 x 16.05 m PGM. The extra positive-X/Y padding is occupied, not traversable.
The rim follows the original PGM bounds, not the padded 30 x 17 m perimeter. Goals
outside the old +/-6 m window can now be planned if they are within this fixed
map and a feasible path exists. Unobserved/unmarked cells are assumed free,
not proven safe by lidar; DOG still processes only +/-10 m and retains its old
filtering/decay/publication behavior. Its historical nonempty-only publication
and FastLayer cache are unchanged, not a new obstacle-memory system.

Only `dynamic_only` changes: both costmaps load the standard Nav2 StaticLayer
named `boundary_layer` before FastLayer and InflationLayer. The existing
map_server serves a generated boundary-only PGM on the navigation `/map` topic.
Since 2026-10-15, with `use_rviz:=true`, an independent map_server and lifecycle
manager under `<robot>/visualization` publish the original PGM on
`visualization/static_map`. Only the main RViz's existing `Map` background is
remapped there, showing black walls/white free space for visual reference.
`Global Costmap`/`Local Costmap` still show the actual navigation costs, including
the 0.35 m rim. Original PGM walls do NOT enter planning or smoothing in this mode.
Untick `Map` to hide the reference; no extra terminal or launch argument is needed.
The reference nodes are disabled with `use_rviz:=false`, and their lifecycle is
separate from the navigation map. Other costmap modes keep their existing nodes
and RViz subscriptions; the Point-LIO comparison RViz is unchanged.
The DOG map still reads the original YAML for geometry. `combined`/`static_only`
keep their original map and complete configuration. Local geometry remains
5 x 5 m; DOG range/filters, ESDF/MINCO/MPC source and weights, point-cloud
bridges, BT and Docker images are unchanged. No custom costmap plugin is added;
the two new nodes above are visualization-only. Existing inflation and ESDF
also see the rim as a hard obstacle, not the reference PGM's interior walls.
This is a navigation guard, not a Gazebo collision fence or a guarantee against
falling under manual velocity commands, localization drift or control errors.

Rebuild with the safe script above and restart the same algorithm command with
`costmap_mode:=dynamic_only`. Do not start an additional algorithm instance.
Configuration-isolation regressions are in `test/test_costmap_modes.py`;
synthetic tests are not full Gazebo/real-robot acceptance.

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
