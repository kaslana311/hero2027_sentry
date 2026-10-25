# Point-LIO 配准点云接入 DOG map

日期：2026-10-02。两仓库分支：`codex/point-lio-sim`。
**本轮只接管感知点云，导航定位仍是真值。** 原真值入口和基线标签保留。

## 2026-10-08 阶段归档

用户明确反馈：点云没有大问题、定位几乎无漂移、过洞比较正常，这一阶段结束。
因此 C1 点云接管已获用户定性确认；没有提供全路线误差、轮数或 action 统计，
不把反馈写成定量成功率。运行配置按本对话最近的 2026 / tunnel / dual / combined
命令记录，本次未另提供变更参数或新日志/录像。

两个仓库保持 `codex/point-lio-sim`，在本次归档提交上创建同名 annotated tag
`sim/point-lio-cloud-baseline-v1`。算法归档前 HEAD `e4d48fe`，最后运行修复
`25336a1`；原真值标签 `sim/truth-tunnel-baseline-v1` 和稳定分支不移动，
不合并到真车 master 或推送远程。本次仅修改 Markdown，不改代码/参数、不重新编译。

目前 DOG map 使用 Point-LIO 配准云，导航 TF/里程计仍为真值；对照窗口的 Point-LIO
定位没有接管导航。C2 完整定位接管尚未实施，也不代表真实逐点去畸变已经复现。
三个入口：当前 `perception_point_lio_sim.launch.py`；对照
`perception_truth_sim.launch.py`；可选只读 `point_lio_shadow.launch.py`。
当前入口已包含估计器，不应再启动独立旁路。命令仍按下文执行，无需额外终端。

本次安全脚本 `--verify-only`：42 前缀/117 ELF、断链/缺库/临时 RPATH 均为 0；
仿真 14 项测试通过。42 算法回归是 2026-10-02 修复后的历史结果，本次没有重跑。
完整配对记录与入口总览位于仿真仓库：
`experiments/2026-10-08_point-lio-cloud-baseline.md`、`docs/STARTUP.md`。
下文“待测/不新增标签”保留其 2026-10-02 实现及修复时的历史范围，不代表尚未收到反馈。

## 本轮边界

用户反馈上一轮滤波权重/重力修正后 Point-LIO 几乎不再发散，附图单时刻
XY 误差约 0.072 m、姿态误差约 0.42°。这是改善反馈，不是全路线误差统计。
现在只换一个变量：DOG map 点云换成 Point-LIO 配准云；导航里程计/TF、规划、
MINCO/MPC、恢复树、频率、combined 插件列表和感知过滤参数都不变。没有新镜像。

```text
双雷达 + 右 IMU → 输入适配 → Point-LIO → pointlio/cloud_registered（pointlio_odom）
                                                    ↓ 固定一次初始对齐
                                  pointlio/cloud_registered_map（map）→ DOG map
Gazebo 真值 → 原导航里程计/TF（本轮继续保留）             → costmap → 原规划/控制
            → truth_3d → 观察器首次同步对齐；之后只评估漂移
```

这不是完整 LIO 导航接管；漂移云与真值底盘的失配仍会影响周围过滤，正是要测的变量。
下一步通过本轮后才接管定位，不同时修改感知或导航参数。

## 两个终端启动

更新代码用 `./build_hero2026_safe.sh --jobs 4`，原 build/install 不分开。
先停止旧的 `perception_truth_sim.launch.py` 和独立 `point_lio_shadow.launch.py`，
不要重复启动估计器、DOG map、导航 TF 或控制发布者。

```bash
# 终端一：仿真入口不变，已有同配置容器不必重建
cd /home/apocalypse_linux_00/build_sim_docker
./scripts/start_sim_rmuc_2026_dual_lidar.sh
```

```bash
# 终端二：导航、Point-LIO 和点云适配一起启动
source /opt/ros/humble/setup.bash
source "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))/install/setup.bash"
ros2 launch hero_sim_bringup perception_point_lio_sim.launch.py \
  field:=rmuc_2026 lidar_mode:=dual costmap_mode:=combined \
  relay_cmd_vel:=true publish_debug_visualizations:=false \
  point_lio_rviz:=true
```

`point_lio_rviz` 默认 false，true 只是打开额外对照窗口；主导航 RViz 默认开启。
不用另开第三个终端跑旁路。新入口只支持双雷达，旧入口仍支持单/双雷达。
`relay_cmd_vel:=false` 可先做不移动的感知检查。不要混入 pb_ws_nav 环境。
播放后先静止初始化约 10 秒，确认输出刷新后再发导航目标。

空闲终端可做一次只读检查（不是常驻第三个算法进程）：

```bash
ros2 param get /red_standard_robot1/dog_map_node cloud_topic
ros2 topic echo /red_standard_robot1/pointlio/cloud_deployment/status std_msgs/msg/String --once
ros2 topic info /red_standard_robot1/pointlio/cloud_registered_map --verbose
```

DOG map 期望订阅 `pointlio/cloud_registered_map`；状态 `publishing`，published
持续增加、wall_age_sec 较小，**只证明适配节点正在发布，不证明 DOG map 已收到**。
还必须检查发布端与 DOG map 订阅端均为 `RELIABLE`，以及下游实际输出。
主 RViz 的 DOG map 障碍显示来自 LIO 云；
对照窗口绿色是真值定位、蓝色是 LIO 定位。勾选 Point-LIO scan 看配准云，
颜色表示高度而非障碍。原 `/red_standard_robot1/cloud_registered` 真值云保留
供对照，但 DOG map 不订阅它，没有自动回退。

```bash
# DOG map 输出保持 BEST_EFFORT，检查时使用兼容的接收策略
timeout 10s ros2 topic echo /red_standard_robot1/rog_map/ground \
  sensor_msgs/msg/PointCloud2 --field header --qos-reliability best_effort --once
timeout 10s ros2 topic echo /red_standard_robot1/rog_map/inf_occ \
  sensor_msgs/msg/PointCloud2 --field width --qos-reliability best_effort --once
```

`rog_map/ground` 在当前实现中是 DOG map 接收、转换后的点云，不是仅含地面点；
收到它证明输入回调已执行。`inf_occ` 是占据分析结果，非空输出才证明障碍生成链路
已通；没有有效障碍时节点不会发布空结果，因此单次超时不能单独判定 QoS 故障。
若下游未更新，先取消导航，避免只依赖静态地图继续移动。

## 2026-10-02 实跑发现的 QoS 断链及修正

11:24 的实跑日志出现 `RELIABILITY_QOS_POLICY`：新适配器此前输出 BEST_EFFORT，
但 `dog_map_node_lidars` 要求 RELIABLE，ROS 2 因而不投递。Point-LIO 和适配器
可以正常发布，DOG map 却收不到任何云；此前的 40 项回归没有覆盖这个实际接收端。
这是新增接口的遗漏，不是 Point-LIO 发散或 DOG map 过滤阈值问题；不能把此前
这次运行的导航效果记成“LIO 动态障碍闭环已验收”。

修复提交 `25336a1ce976b09f4f66437a410dd3a96de34274`，
2026-10-02T15:11:21+08:00，仅将点云适配器输出改成 RELIABLE / KeepLast(10)，
与原真值桥输出契约一致。输入仍为 SensorDataQoS，DOG map 的订阅、BEST_EFFORT
输出、感知阈值、导航/TF、规划/控制和 Docker 都不变。增加 Reliable 接收测试、
实际生产 DOG map 接收并输出非空占据点云的测试，以及相应测试依赖。

两个新增测试在修复前安装产物上均失败，DOG map 报出与实跑相同的 QoS 警告；
安全脚本重编后全部 **42 项算法测试、14 项仿真测试通过**。生产 DOG map 测试
沿用队内感知参数，使用隔离 ROS 域/合成墙面和小幅自由地图，不运行 Gazebo/Nav2，
不发布运动指令；它证明数据接收/占据输出，不替代完整过洞验收。
安全构建 38 包、42 前缀/117 ELF，断链/缺库/临时 RPATH 均为 0。

修复后停止并重启整个算法入口，启动命令和参数不变；已有仿真容器可以继续用，
无需重建镜像。完整证据见仿真仓库 `experiments/2026-10-02_point-lio-dog-map-qos.md`。

## 一次对齐，不是动态真值校正

LIO 输出已经在固定局部世界系，不是雷达/底盘系，不能再套当前底盘 TF。
观察器首次在相同采样时刻 t0 取得两套位姿，冻结：

```text
T_map_lio = T_map_chassis_truth(t0) × T_chassis_imu × inverse(T_lio_imu(t0))
p_map(t) = T_map_lio × p_lio(t)
T_chassis_imu = translation(0, -0.167, 0.121) m，旋转 identity（2026-10-25 内移后的安装）
```

新入口显式开启 `publish_cloud_alignment`，通过持久化的
`pointlio/evaluation/cloud_alignment` 只发布一次 TransformStamped。
旧旁路默认关闭该输出，仍是显示观察器。适配节点不订阅真值、里程计或移动 TF，
不将对齐反馈给估计器，之后不追随真值重新对齐。初始位置由真值给定是实验前提，
不是自动重定位；不加载先验 PCD，不启用 small_gicp，静态地图不参与 LIO 前端配准。

输出保持扫描时间戳、点数、intensity/curvature、字段布局与行填充，XYZ 做刚体
变换；若有完整 normal_x/y/z，则仅旋转法线。支持实际 PCL FLOAT32 小端布局，
错误 frame/stamp/layout 拒绝。NaN 保持缺失，溢出结果整帧拒绝。单帧上限 16 MiB，
默认缓存 5 帧、等对齐最多 1 秒墙钟；过期或早于对齐的帧不回放。
没有 TimePointZero 或阻塞 TF 查询。DOG map 既有 z 平移和周围过滤保持原样。

## 失效状态与快速切回

- `waiting_for_alignment`：还没有同步初始化位姿。
- `waiting_for_cloud`：已对齐，没有有效后续云。
- `stale`：超过 1 秒墙钟无输出；暂停仿真也会出现。
- `restart_required`：云时间/仿真时钟倒退超过 0.5 秒，或有效对齐改变。

失效时本节点停止云输出，不偷偷换成真值云、不重新对齐。
**停止云输出不等于停止 Nav2 或车辆**，静态层可能继续规划；应取消目标/停止
算法入口。Gazebo 重置后重启整个新入口，不能只重启某个 LIO 节点沿用旧对齐。

停止新算法入口，重新 source 同一安装环境后，以下命令回到原理想真值场景：

```bash
ros2 launch hero_sim_bringup perception_truth_sim.launch.py \
  field:=rmuc_2026 lidar_mode:=dual costmap_mode:=combined \
  relay_cmd_vel:=true publish_debug_visualizations:=false
```

同版本装有两个入口，因此不需要切分支、重编译或重建镜像。若要精确恢复历史
标签源码，则成对切换源码并安全重编。不要同时运行两个入口，回退不删除实验提交。

## 验证与待测（2026-10-02 历史记录）

实现提交 `f0ea42d77611e4528749f9c0bb9eb8b099a3937c`，2026-10-02 11:00:08 UTC+8；
权限整理另在 `34fd662` 提交，无消息/包定义文本变化。
安全脚本完整 38 包构建成功，42 前缀/117 ELF，断链/缺库/临时 RPATH 均为 0。
首次 40 项隔离 ROS 测试和 14 项仿真仓库测试通过，新安装入口参数解析通过；
它们当时只覆盖到点云适配器，遗漏 DOG map 实际 Reliable 订阅。
上述 QoS 修复及两项新增回归后，当前算法回归总数为 42。
“联合回归”是让实际 Point-LIO 与新适配节点连起来检查新旧功能，不是 Git 回退。

回归覆盖真实 Point-LIO 输出接入、XYZ/法线变换、字段/时间保留、缺失/过期对齐、
一次对齐后保留漂移、非法布局/NaN、时间重置、无真值/电机接口，以及生成的
导航配置除 DOG 点云来源之外完全一致。构建与测试实际结果见仿真侧
`experiments/2026-10-02_point-lio-cloud-deployment.md`；不把自动化回归当过洞验收。

保持同场地、速度、目标，依次静止、平地、坡面、过洞、停车，比较真值入口与新
LIO 点云入口，记录误差、误障碍和导航 action 结果。没有替用户运行 Gazebo/Nav2、
没有发送运动指令；本轮不合并稳定分支，不新建“已验收”标签。
