# Point-LIO 仿真旁路

日期：2026-10-01。分支：`codex/point-lio-sim`。这是第一步旁路适配，
**不是 Point-LIO 导航接管**。原真值过洞基线保留在 `sim-perception-integration`
及标签 `sim/truth-tunnel-baseline-v1`，仿真仓库也有同名分支和基线标签。

## 数据流与改动范围

```text
Gazebo 原始 lidar_left + lidar_right + imu_right
  → sim_point_lio_input_bridge（固定外参合并 / IMU 向量旋转）
  → input/fused_cloud + input/imu
  → 队内 pointlio_mapping（自主估计、点云配准）
  → pointlio/odometry + pointlio/cloud_registered + 私有 TF/RViz

原 truth bridge → DOG map → costmap → Nav2/MINCO/MPC 继续照常运行
```

不读取真值里程计，不使用仿真绝对 orientation，不加载先验 PCD，不启用 small_gicp，
不启动真车雷达/串口，不发速度指令。DOG map、地图、规划、控制、恢复树和原导航
频率均未改动。没有新增镜像或容器。

Point-LIO 发布端新增参数，默认保持真车原输出行为；未修改估计/匹配/滤波计算：

| 参数 | 默认（真车兼容） | 仿真旁路 |
|---|---|---|
| `odometry.output_imu_pose` | false，保留旧写死的车体外参转换 | true，直接输出 KF IMU 位姿 |
| `odometry.odom_frame_id` | odom | pointlio_odom |
| `odometry.body_frame_id` | base_link | pointlio_imu |
| `publish.body_frame_id` | lidar_link | pointlio_imu |

Point-LIO 状态对应 IMU 位姿，不能将旧真车车体转换套在新虚拟输入上。后续接管
需要显式 IMU→底盘转换、初始地图对齐及 TF 所有权划分，本阶段不做隐含近似。

## 原始输入契约

- 左/右安装 pose 与当前仿真 XMacro 一致；包含 gpu_lidar 局部 z=0.03 m 的射线原点。
- 统一输入原点在右侧 IMU 测点，坐标轴与 chassis 平行。保持测点不变，旋转 gyro、
  acceleration 和协方差，不伪造平移后需要的杆臂补偿。orientation 标记为不可用。
- 仅合并采样时间完全相等的左右扫描，队列每侧最多 5 帧；不采用最新真值/TF
  补偿，也不将异时扫描强行修改为相同时间。实跑仍需验证时间同步。
- 原始点云为 FLOAT32 xyz/intensity 的小端快照，非法布局/过大帧拒绝，非有限 XYZ 丢弃。
- 输出 `CustomMsg.header.stamp` 与 ns 单位 timebase 保留采样时刻；每点 offset_time=0。
  tag=0x10、line=0 仅为输入兼容，不代表真实 MID360 扫描模式。
- Gazebo gpu_lidar 每次 Render 后用一个 `_now` 发布整帧，无 Livox 逐点采样时间，
  依据 [Gazebo Sensors 6 源码](https://github.com/gazebosim/gz-sensors/blob/ign-sensors6/src/GpuLidarSensor.cc)。
  因此目前只能验证快照式 LIO，不能宣称复现真实扫描去畸变。
- IMU 使用 m/s²：仿真配置 `mapping.acc_norm=9.81`。源码饱和检测比较归一化前的
  原始加速度，因此 `mapping.satu_acc=49.05` 对应真车 5 g，不能照搬 5.0。
- 保留现有前端噪声、匹配、滤波参数。输入 20 Hz 雷达、200 Hz IMU 是模型设置，
  不是保证接收频率。旁路里程计按帧发布，关闭无限增长的 path 和 PCD 保存。
- 当前真车 `lidar_merge` 接收 CustomMsg，且 CMake 未定义 `TWO_LIDARS`；本阶段
  使用独立仿真快照合并，不修改也不声称验证真车合并器。

## 编译、启动与检查

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
./build_hero2026_safe.sh --jobs 4
source /opt/ros/humble/setup.bash
source install/setup.bash
```

继续使用原 build/install；不要改用临时手写的 colcon 命令。当前实现已完整构建。
先启动原 RMUC 2026 双雷达 tunnel 仿真和真值导航（参数仍为
`field:=rmuc_2026 lidar_mode:=dual costmap_mode:=combined`），第三个干净终端：

```bash
source /opt/ros/humble/setup.bash
source "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))/install/setup.bash"
ros2 launch hero_sim_bringup point_lio_shadow.launch.py use_rviz:=true
```

可不启动真值导航，只启动 Gazebo 和旁路做静止检查。默认 `use_rviz:=false`。
Gazebo 播放后先静止初始化，确认以下输出，再用原导航低速走平地/坡面/隧道：

```bash
ros2 topic hz /red_standard_robot1/pointlio/input/fused_cloud --wall-time
ros2 topic hz /red_standard_robot1/pointlio/input/imu --wall-time
ros2 topic hz /red_standard_robot1/pointlio/cloud_registered --wall-time
ros2 topic echo /red_standard_robot1/pointlio/odometry nav_msgs/msg/Odometry --once
```

hz 持续运行，按 Ctrl+C 退出。原 RViz/DOG map 仍使用真值点云；新 RViz 用
`pointlio_odom` 和私有 `/red_standard_robot1/pointlio/tf`，不能仅改名叠到 Nav2 map。
必须在评估时按同一时间戳、虚拟 IMU→底盘外参和初始对齐比较误差。
仿真重置时间后重启旁路，不沿用原有缓存或估计器地图。关闭第三个终端即可关闭实验。

回归测试（隔离 domain，不操作真实仿真/底盘）：

```bash
export BRIDGE_EXECUTABLE="$(ros2 pkg prefix hero_sim_bringup)/lib/hero_sim_bringup/sim_ground_truth_bridge"
export POINT_LIO_INPUT_EXECUTABLE="$(ros2 pkg prefix hero_sim_bringup)/lib/hero_sim_bringup/sim_point_lio_input_bridge"
export POINT_LIO_EXECUTABLE="$(ros2 pkg prefix point_lio)/lib/point_lio/pointlio_mapping"
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=174 python3 -m pytest src/simulation/hero_sim_bringup/test -q
```

## 已测与未测

安全脚本 38 包构建成功；安装审核 42 前缀、115 ELF，断链/缺库/临时 RPATH 均为 0。
18 测试通过：8 个真值桥、3 个恢复契约、7 个旁路输入/前端测试。新测试覆盖固定
安装变换、同步边界、IMU 单位/旋转、非法输入、无真值订阅和实际 Point-LIO 初始化；
真实可执行程序处理合成平面后能发布有限、近原点的位姿和配准点云。

尚未测：真实 Gazebo 消息同步、实际重力测量、运动响应、静止漂移、坡面残差、
洞内退化、时间延迟和定量误差；未启动 Gazebo/Nav2 或移动机器人验证此旁路。
通过以上联合测试后再实现独立定位/点云接管入口，不隐藏真值回退，不同时调感知参数。
