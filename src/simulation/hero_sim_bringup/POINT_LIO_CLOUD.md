# Point-LIO 配准点云接入 DOG map

日期：2026-10-02。两仓库分支：`codex/point-lio-sim`。
**本轮只接管感知点云，导航定位仍是真值。** 原真值入口和基线标签保留。

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
持续增加、wall_age_sec 较小。主 RViz 的 DOG map 障碍显示来自 LIO 云；
对照窗口绿色是真值定位、蓝色是 LIO 定位。勾选 Point-LIO scan 看配准云，
颜色表示高度而非障碍。原 `/red_standard_robot1/cloud_registered` 真值云保留
供对照，但 DOG map 不订阅它，没有自动回退。

## 一次对齐，不是动态真值校正

LIO 输出已经在固定局部世界系，不是雷达/底盘系，不能再套当前底盘 TF。
观察器首次在相同采样时刻 t0 取得两套位姿，冻结：

```text
T_map_lio = T_map_chassis_truth(t0) × T_chassis_imu × inverse(T_lio_imu(t0))
p_map(t) = T_map_lio × p_lio(t)
T_chassis_imu = translation(0, -0.217, 0.121) m，旋转 identity
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

## 验证与待测

实现提交 `f0ea42d77611e4528749f9c0bb9eb8b099a3937c`，2026-10-02 11:00:08 UTC+8；
权限整理另在 `34fd662` 提交，无消息/包定义文本变化。
安全脚本完整 38 包构建成功，42 前缀/117 ELF，断链/缺库/临时 RPATH 均为 0。
40 项隔离 ROS 测试和 14 项仿真仓库测试通过，新安装入口参数解析通过。
“联合回归”是让实际 Point-LIO 与新适配节点连起来检查新旧功能，不是 Git 回退。

回归覆盖真实 Point-LIO 输出接入、XYZ/法线变换、字段/时间保留、缺失/过期对齐、
一次对齐后保留漂移、非法布局/NaN、时间重置、无真值/电机接口，以及生成的
导航配置除 DOG 点云来源之外完全一致。构建与测试实际结果见仿真侧
`experiments/2026-10-02_point-lio-cloud-deployment.md`；不把自动化回归当过洞验收。

保持同场地、速度、目标，依次静止、平地、坡面、过洞、停车，比较真值入口与新
LIO 点云入口，记录误差、误障碍和导航 action 结果。没有替用户运行 Gazebo/Nav2、
没有发送运动指令；本轮不合并稳定分支，不新建“已验收”标签。
