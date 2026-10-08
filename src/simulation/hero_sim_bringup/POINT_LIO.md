# Point-LIO 仿真旁路

2026-10-08 更新：用户已确认当前 C1 点云接管场景的点云/定位对照/过洞表现正常，
归档为配对标签 `sim/point-lio-cloud-baseline-v1`。导航仍是真值，C2 尚未实施。
当前推荐两终端入口见 [POINT_LIO_CLOUD.md](./POINT_LIO_CLOUD.md)；下文三终端
旁路和待复测文字保留原日期及用途，不代表当前推荐流程或尚未获得用户反馈。

2026-10-02 更新：用户已反馈上一轮修正后几乎不发散；尚无全路线统计。
本篇三终端操作仍适用于**只读旁路**。另有两终端的点云接管入口，见
[POINT_LIO_CLOUD.md](./POINT_LIO_CLOUD.md)：DOG map 用 LIO 云，导航仍用真值；
显式给定一次初始位置，不实时校正，不与本篇旁路同时启动。
此前待复测描述保留为历史记录，不代表尚未收到用户的新反馈。

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

输入适配器与 Point-LIO 不读取真值里程计，不使用仿真绝对 orientation；独立显示观察器
只为对照而读取真值，不反馈给估计器或导航。不加载先验 PCD，不启用 small_gicp，
不启动真车雷达/串口，不发速度指令。DOG map、地图、规划、控制、恢复树和原导航
频率均未改动。没有新增镜像或容器。

初版旁路仅给 Point-LIO 发布端新增参数，默认保持真车原输出行为；初版未修改
估计/匹配/滤波计算。2026-10-01 后续另有共享滤波公式修正，见本文末尾，不将其
误称为“只改仿真 YAML”。发布参数仍为：

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
- IMU 使用 m/s²：实测静止模长 9.8，仿真配置的 `mapping.acc_norm`、`gravity`
  和 `gravity_init` 因而统一为 9.8。源码饱和检测比较归一化前的原始加速度，
  因此 `mapping.satu_acc=49.0` 对应 5 g，不能照搬 5.0。真车 YAML 不改。
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

只启动 Gazebo 和旁路仍能做前端静止检查，但地图/真值对照需要原真值导航入口。
默认 `use_rviz:=false`。本次新增真值诊断发布者，更新后需重启第二、三个终端的
算法进程并重新 source；Gazebo 启动命令、镜像和容器不变。
Gazebo 播放后先静止初始化，确认以下输出，再用原导航低速走平地/坡面/隧道：

```bash
ros2 topic hz /red_standard_robot1/pointlio/input/fused_cloud --wall-time
ros2 topic hz /red_standard_robot1/pointlio/input/imu --wall-time
ros2 topic hz /red_standard_robot1/pointlio/cloud_registered --wall-time
ros2 topic echo /red_standard_robot1/pointlio/odometry nav_msgs/msg/Odometry --once
```

hz 持续运行，按 Ctrl+C 退出。原 RViz/DOG map 仍使用真值点云；新 RViz 现在固定在
`map`，通过下面的一次性对齐和私有 TF 显示两套底盘位姿，非简单 frame 改名。
仿真重置时间后重启旁路，不沿用原有缓存或估计器地图。关闭第三个终端即可关闭实验。

## 对照画面、原点与对齐（2026-10-01）

第二个窗口只用于定位对照，不用于发导航目标。默认显示灰黑静态场地图（非 costmap）、
1 m 网格、原点、定位标记和轨迹；点云默认关闭，避免遮挡，勾选 `Point-LIO scan`
可看已对齐的配准点云，高度颜色不是障碍分类。工具栏恢复 Move Camera/Select/Focus
Camera，右侧恢复 Views；滚轮缩放，左键旋转，Shift+左键平移。默认 Orbit 固定在
map 而非跟随 IMU，Distance=45、Focal Point=(10,1.5,0)，先看到整个 2026 地图，
也可在 Views 中直接修改距离和焦点。

| 画面标记 | 代表什么 |
|---|---|
| 黄色球与 `MAP = ODOM ORIGIN (0,0,0)` | 固定 map/odom 原点，不随机器人运动 |
| 绿色大环、长箭头、`TRUTH chassis` | 同一采样时刻的三维真值底盘位置/朝向 |
| 蓝色小环、短箭头、`POINT-LIO chassis` | Point-LIO IMU 位姿经固定外参转换后的估计底盘位置/朝向 |
| 绿/蓝轨迹 | 两者最近最多 1000 个样本，最多 10 Hz；初始重合是正常的 |

环的半径不同只是避免重叠看不清，不是 footprint、碰撞体或误差置信圆。文字位置有
显示偏移，定位位置是环的中心和箭头起点；文字同时列出 map 坐标。定位标记短时
过期，收到新数据才刷新；停更不能被误看成当前位置持续更新。

当前真值入口发布固定 identity 的 map→odom；第一帧**二维**底盘真值定义原点的
X/Y/yaw，Z=0。它不是 PGM 左下角（YAML 的 origin），也不是 Gazebo 世界原点或
地图中心。机器人初始底盘中心可能有离地高度，因此标记 Z 不必等于 0。
Point-LIO 的 `pointlio_odom` 是独立初始化局部系，原点对应右 IMU 初始化状态。

真值桥新增仅用于位姿诊断的 `/red_standard_robot1/odometry/truth_3d`，沿用桥已确定的
map 原点，保留三维姿态和源时间戳；观察器晚启动也不重新定义原点。该消息只用于
pose 对照，twist 不作为 map-frame 速度接口使用。原二维 `odometry/filtered`、
点云配准和导航 TF 不变。

观察器缓存真值，在 Point-LIO 时间戳处做平移插值与四元数 slerp；不拿最新姿态替代。
首次匹配 t0 时冻结显示变换：

```text
T_map_lio = T_map_chassis_truth(t0) × T_chassis_imu × inverse(T_lio_imu(t0))
T_map_chassis_est(t) = T_map_lio × T_lio_imu(t) × inverse(T_chassis_imu)
T_chassis_imu = translation(0, -0.217, 0.121) m，旋转为 identity
```

因此 t0 误差为零是人为初始化对齐，**不证明绝对定位精度或自动地图重定位成功**。
后续绝不实时追随真值重对齐，漂移会保留。真值插值帧间隔超过 0.15 s、不足或缺失，
最多等待 1 s（墙钟）后丢弃该比较；时间大幅倒退停止对照，须重启旁路。
map→pointlio_odom 只发布在私有 `pointlio/tf_static`；不污染原导航 TF。

观察器输出都在 `/red_standard_robot1/pointlio/evaluation/` 下：
`truth_pose`、`pointlio_pose`（底盘 PoseStamped）、`truth_path`、`pointlio_path`、
`markers`、`error`（JSON，米/度，含采样与对齐时间）。例如：

```bash
ros2 topic echo /red_standard_robot1/pointlio/evaluation/error std_msgs/msg/String --once
```

估计器 odometry 本身仍是 IMU 位姿、配准点云仍在 pointlio_odom；只改变显示变换，
未更改估计计算、输入、导航/感知/控制参数，也没有把真值喂给 Point-LIO。

回归测试（隔离 domain，不操作真实仿真/底盘）：

```bash
export BRIDGE_EXECUTABLE="$(ros2 pkg prefix hero_sim_bringup)/lib/hero_sim_bringup/sim_ground_truth_bridge"
export POINT_LIO_INPUT_EXECUTABLE="$(ros2 pkg prefix hero_sim_bringup)/lib/hero_sim_bringup/sim_point_lio_input_bridge"
export POINT_LIO_EXECUTABLE="$(ros2 pkg prefix point_lio)/lib/point_lio/pointlio_mapping"
export EVALUATION_EXECUTABLE="$(ros2 pkg prefix hero_sim_bringup)/lib/hero_sim_bringup/sim_point_lio_evaluation"
export POINT_LIO_CLOUD_EXECUTABLE="$(ros2 pkg prefix hero_sim_bringup)/lib/hero_sim_bringup/sim_point_lio_cloud_bridge"
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=174 python3 -m pytest src/simulation/hero_sim_bringup/test -q
```

## 已测与未测

初版旁路安全脚本 38 包构建成功；安装审核 42 前缀、115 ELF，断链/缺库/临时 RPATH
均为 0，18 测试通过。本次新增观察器的构建/测试结果在下面另记，不覆盖初版记录。
初版输入/前端测试覆盖固定
安装变换、同步边界、IMU 单位/旋转、非法输入、无真值订阅和实际 Point-LIO 初始化；
真实可执行程序处理合成平面后能发布有限、近原点的位姿和配准点云。

本次对照窗口：同一安全脚本最终完整 38 包构建成功，42 前缀/116 ELF，断链、缺库、
临时 RPATH 均为 0。26 项测试通过（9 真值桥 + 3 BT + 7 输入/前端 + 7 对照），
新增覆盖 map 原点/三维姿态、一致采样时间、IMU→底盘外参、一次对齐后保留漂移、
延迟真值插值、缺失/非法 frame、时间重置、无导航发布和相机工具配置。
以 domain 177 合成位姿/静态地图短暂打开 RViz，检查标记、轨迹和工具/Views，
并调整文字偏移减少重叠；不连接实际 Gazebo、不发控制指令、测试进程已关闭。
这个显示检查不是实测 Point-LIO 运动或导航验收，也不改变全局图形/GPU 设置。

上述初版测试之后，用户已实跑并报告平地/坡面漂移、静止后持续下沉。修正前的
只读采样确认静止真值几乎不动，但估计位姿持续发散；实际 IMU 模长为 9.8。
这不是单纯 RViz 显示原点不一致，详见下面的后续排查。修正后现场运动仍待复测。
通过以上联合测试后再实现独立定位/点云接管入口，不隐藏真值回退，不同时调感知参数。

## 静止发散排查与最小修正（2026-10-01）

大白话：传感器报“车没动”，滤波器却越算越觉得自己在动。已经确认一个批量
激光更新公式错误，以及一个仿真重力参数不一致；它们是要先消除的实际缺陷，
但尚不能证明修正后所有隧道漂移都已消失。暂不通过增加点数、伪造逐点时间、
真值反馈或自动重新对齐掩盖问题。

| 算法提交（均在 `codex/point-lio-sim`） | 目的 | 时间（UTC+8） |
|---|---|---|
| `1c14dd3` | 修正批量更新的测量噪声权重，增加原生 EKF 回归 | 2026-10-01 21:42:26 |
| `94495a3` | 默认关闭的只读扫描诊断，仿真开启 | 2026-10-01 22:01:28 |
| `e447239` | 仅仿真重力统一为 9.8，增加运动/倾斜/静止回归 | 2026-10-01 22:01:50 |

共享 `esekfom.hpp` 的少点更新把 `M_Noise` 当方差 R，但大量同时间点更新曾用
`R * HᵀH`，正确应为 `HᵀH / R`；增益中相应使用 1/R。仿真 `R=0.005`，旧
信息权重为 0.005 而应为 200，相差 40000 倍；这不是说位姿误差固定放大 40000 倍。
Gazebo 快照的 offset 全为零，常整帧走这一批量分支。旧公式在本地原始源码和
[上游实现](https://github.com/hku-mars/Point-LIO/blob/point-lio-with-grid-map/include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp#L199-L215)
均存在，不是旁路显示提交引入。

这次确实修改了真车与仿真共用的滤波头文件，但只在实验分支上。真车 YAML、
稳定分支、基线标签、真值导航、DOG map、Nav2/MINCO/MPC 和 Docker 均不改。
关闭第三个终端仍是原真值导航；只有切换源码时才需按安全脚本重编。

新的 `[LIO_DIAG]` 日志最多每仿真秒一条：

- `points`：预处理后的扫描点数；`matched`：本帧实际参与更新的平面残差数。
- `max_batch`/`batch_updates`：单次最大更新行数/大量点更新次数，不是接收频率。
- `no_match`：没有有效匹配的更新次数；`dt`/`negative_dt`：状态预测步长范围/负步长次数。
- `pos`/`speed`：滤波内部位置/速度；`imu_acc_norm`：当前处理 IMU 加速度模长。

同时间点的零步长是合法的；负步长、长期无匹配、停车后内部速度或误差持续
增大才值得进一步追查。话题 hz 和这组日志是不同证据，hz 正常不保证匹配有效。
默认 `runtime_lio_diagnostics=false`，仿真 YAML 开启；日志不修改滤波状态、
不重置、不给导航发速度，也不读真值。

验证记录：

- 原生 GTest 直接调用生产 EKF 头文件，覆盖 1/10/29/30/31/100/2000 残差，
  批量与逐点同时间更新等价、无匹配不校正、协方差对称/正定。旧版 9 项中 5 项
  失败；修正版 9 项全过，实际 CMake/CTest 测试目标也构建并通过。
- 安全脚本 `./build_hero2026_safe.sh --jobs 4` 全部 38 包成功，42 安装前缀、
  116 ELF，断链/缺库/临时 RPATH 均为零。没有换生产编译方式。
- 27 项隔离 ROS 回归通过（9 真值桥、3 BT、8 输入/前端、7 对照）；新增运动测试
  在三平面中平移 0.4 m、升高 0.03 m/倾斜 0.12 rad 后停车，要求末段位置误差
  <0.08 m、角误差 <2°、静止段漂移 <0.02 m。修正版通过；旧二进制在这个理想
  合成运动测试中也通过，不能把它说成现场故障复现或修复充分证明。
- 合成运行日志可见有效批量匹配、`negative_dt=0`，末段内部速度接近零。
- 修正后做过 30 秒墙钟独立命名空间传感器读取，但适配后的 IMU、三维真值和
  估计里程计消息计数都为零，故没有完成现场静止验证。只关闭该探测启动的两个
  进程，不发运动指令。

复测无需重新构建镜像，也不用改启动入口。重新 source 安装环境、重启第三个
终端的 `point_lio_shadow.launch.py` 即可加载新二进制/参数。先静止 10 秒，再走
平地并停 10 秒，再上小坡并停 10 秒，最后过洞并停车；在原导航 RViz 发目标，
对照窗口只看绿蓝轨迹/误差。每次 Gazebo 重置后重启旁路。

若仍发散，保留从初始化到发散的 `[LIO_DIAG]` 与同时间 `evaluation/error`。
下一步先看实际匹配是否丢失、预测步长/时间对齐是否异常，然后再检验洞内几何
退化和点云分布；目前不接管导航、不调 DOG map、不合并稳定分支或打验收标签。
