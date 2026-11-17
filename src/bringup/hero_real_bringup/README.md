# 真传感器分支：Point-LIO 导航 + 东北大学后端旁路

2026-11-17，从 `codex/point-lio-backend` 的 `b723fdb` 建立
`codex/real-sensor-navigation`。本包不重写规划、平滑、控制或后端算法。
旧仿真入口不删除，默认仍是仿真；此包提供真实 Livox 输入和通信启动组合。

## 保留和替换的内容

- 保留四种 `costmap_mode`、split 静态 A* / 动态 MINCO、A* 起步恢复、
  局部路线修正、确认卡住后冷启动、高处柱过滤、ESDF 梯度投影热调开关。
  真/仿真共用原参数生成器和已修复的导航行为树，不复制两套算法配置。
- 不修改 MINCO/MPC 的优化函数、权重、速度上限或热启动判据。
  不新增轮速/真值速度闭环；底盘速度闭环仍由单片机负责。
- 真输入不启动 `sim_ground_truth_bridge`、仿真点云/IMU 拼造器或真值评估器。
  直接读取 Livox CustomMsg 的真实点时间戳，IMU 单位维持原驱动的 g、rad/s。
- 东大后端仍计算且使用**私有 TF**，不自动接管导航。
  没有真实三维先验包时计算纯里程计因子图，不能声称能纠正全局漂移。
  不自动把仿真 CAD 先验当作实测先验。PGM 不是三维定位先验。

## 上赛季实车接口参考与局限

只读检查主机 `dji-NucBox-K8-Plus` 的 `/home/dji/hero2026_-sentry`，
参考 HEAD `5d5e69d` 及其现有未提交配置，旧仓库保留不动。

| 接口 | 本分支使用方式 |
|---|---|
| MID360 193、153 | `config/livox_dual.json`，原驱动 CustomMsg、多话题、20 Hz |
| 双雷达相对外参 | `config/hardware.yaml`，从旧 `lidar_merge/config/cfg.yaml` 读取 |
| 主 IMU | `/livox/imu_192_168_1_153`，不是仿真的 SI 虚拟 IMU |
| 原融合点云 | `/livox/fused_cloud`；新增显式双雷达可执行程序，旧单路程序不改变 |
| 底盘串口 | `/dev/chassis`，460800；沿用 `hero_serial` 的收发协议、CRC 和单位换算 |
| 视觉电脑 | 原 `zenoh-bridge-ros2dds`，端点 `tcp/192.168.123.20:7447` |
| 出站话题 | `/carrot_pose`、`/MaskID`、`/gimbal_decision` |
| 入站话题 | `/tracker/autoaim` |

外参沿用旧实车 Point-LIO 实际输出所用矩阵：
`T_base_imu.xyz=[0.062208,-0.207112,0]`，RPY=`[0,0.614120,-pi/2]`。
这和旧 `fusion_launch.py` 中另一套 pitch 数字并不完全相同；采用旧里程计
实际应用的矩阵，避免叠加两次。**它不是对今年新机械布局的现场标定结论**。
雷达重新安装后必须修改硬件配置，并重新核对 LiDAR→IMU 和双雷达外参。
旧融合配置写 `blind: 0.15`，节点实际读取 `blind_distance`，实际生效为默认
0.25；新配置显式保持该实际值，没有悄悄改变盲区。

真 Point-LIO 使用实车参考的降采样和噪声配置，`acc_norm=1`、`satu_acc=5`、
`vel_cov=20`，不用仿真的 9.8/49/200。保留前端既有 IMU 缓冲、回调排空和越界修复。
输出选择 IMU 位姿，由唯一外参适配器转到车体；扫描结束位姿与点云同时间戳，
让后端 1 ms 同步容差真正适用，不把 IMU 频率位姿与相邻扫描强行拼在一起。

## TF 与速度

主树：`map -> odom -> base_footprint -> base_link -> pointlio_imu`。

`T_odom_base=T_odom_imu * inverse(T_base_imu)`。
第一帧根据**用户配置的初始车体 XYZ/yaw**确定 `map->odom`，而不是读取出生真值。
之后保持局部里程计连续；`map_odom_offset_*` 只修改全局对齐。
注册点云通过主树的采集时刻 `map->odom` 转到 map，跟导航一起移动。
`publish_map_to_odom=false` 时必须有其他唯一发布者；后端旁路不是该发布者。
不要同时启动旧 Point-LIO、旧重定位/TF 管理器或第二个 map→odom 发布器。

`cmd_vel_nav2_result -> real_point_lio_navigation_bridge -> cmd_vel -> hero_serial`。
MPC 已经将速度转换到 base_link，串口只做 m/s→mm/s；不再启动会重复旋转的
fake_vel_transform。保留原输入新鲜度/命令超时保护，不新增导航速度控制策略。
`base_link_in_map_odom` 为从主 TF 得到的前端车体位姿，不是地面真值。
后端窗口显示前后端同时间位置差，**没有真值时不能将它称为绝对定位误差**。

## 启动

使用干净终端，source **新目录**，不要先 source 旧实车安装空间：

```bash
source /opt/ros/humble/setup.bash
source /home/dji/hero2026_sentry_real/install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch hero_real_bringup navigation_point_lio_backend_shadow_real.launch.py \
  field:=rmuc_2027 lidar_mode:=dual costmap_mode:=split \
  filter_high_columns:=false high_column_min_z:=0.20 \
  split_local_refinement:=true stuck_cold_start:=true \
  start_lidar_driver:=true start_lidar_merge:=true \
  relay_cmd_vel:=false start_serial:=false
```

以上不打开串口，但会启动传感器驱动。旧驱动仍运行时改为
`start_lidar_driver:=false start_lidar_merge:=false`，复用现有融合输入。
没有后端的对照入口是 `navigation_point_lio_real.launch.py`。
原入口也保留：`hero_sim_bringup navigation_point_lio_backend_shadow_sim.launch.py`
增加 `data_source:=real namespace:=""` 即选择同一真实入口，其余已有参数仍有效。

真正驱动前，确认机器人可安全运动、雷达时钟/外参/地图/串口无误，且旧程序已退出。
随后才按需打开 `relay_cmd_vel:=true start_serial:=true start_zenoh:=true
start_vision_bridge:=true`。**原串口节点自身就可能发送小陀螺/姿态指令**，
所以仅 `relay_cmd_vel=false` 不等同于硬件断使能；本包默认不打开串口。
不自动启动比赛决策；`start_decision=true` 才启动原决策，要求根命名空间。
该开关加载**新安装空间**中的原决策 XML，不读取旧工作区的绝对路径。
这仍是原赛季决策，不是 2027 规则适配。原裁判系统/雷达目标使用
`global_red_map`，而新导航使用 map；部署比赛决策前，必须按实际红蓝方和
PGM 原点标定 `map -> global_red_map`，以及 outpost/rune 等目标坐标。
本入口不猜测这些比赛场地变换，也不启动旧重定位程序来重复发布 map→odom。
未标定时不要启用比赛决策；手动发送 map 坐标导航目标不依赖这些附加坐标系。

主要新增参数：

| 参数 | 含义 |
|---|---|
| `hardware_profile` | 新硬件 YAML；空值用上赛季参考，不代表今年已标定 |
| `point_lio_config` / `livox_config` | 指定真实前端配置/驱动 JSON |
| `initial_x/y/z/yaw` | 启动时车体在 map 中的近似位置和航向（m、rad），默认零 |
| `map` | 可指定真实场地二维 YAML；空值仍选 field 对应现有 PGM |
| `prior_map` | 实测三维先验包目录；空值只运行里程计因子图 |
| `color_map` | 视觉语义区域图；默认灰度 PGM 无红色豁免区域 |
| `start_lidar_driver/merge` | 两者默认 true，可使用已运行的外部传感器输入 |
| `start_serial/zenoh/vision_bridge/decision` | 默认 false，各自显式开启 |
| `relay_cmd_vel` | 默认 false，独立于串口开关 |

原参数 `namespace`、`field`、`lidar_mode`、四种 `costmap_mode`、`split_local_refinement`、
`stuck_cold_start`、`filter_high_columns`、`high_column_min_z`、`dog_resolution_xy`、
`publish_debug_visualizations`、各 RViz 开关和 `map_odom_offset_*` 都保留。
实车入口默认根命名空间，以兼容原决策和外部通信；仿真入口默认命名空间不变。
`minco_smoother.project_esdf_gradient`、ESDF 安全距离/权重仍可用原 ros2 param 命令调整。

`high_column_min_z` 仍按 map 的绝对 Z 判断，不会自动按各区域地面重新计算。
`split_local_refinement` 的六隧道区域仍基于现有 rmuc_2027/map 几何；换场地或地图
原点后必须重新适配，不能因真实传感器接入成功就假定这些区域坐标也已正确。

## 验证和部署边界

契约测试对比真/仿真参数，除时钟和输入适配之外导航代价及优化配置相同。
笔记本隔离构建了六个改动包；237 项真/仿真配置与既有模式测试、32 项
规划恢复/局部修正回归、4 项真实适配可执行程序测试、2 项完整入口测试通过。
运行测试在独立 ROS 域注入合成里程计和点云，不触碰串口；完整入口验证
Nav2 生命周期激活和后端输出，不代表传感器硬件已验证。
物理雷达视野、2027 新外参和真正过洞仍需真车安全验收。

新仓库：`https://github.com/kaslana311/hero2027_sentry.git`；新分支：
`codex/real-sensor-navigation`。旧 github/gitee 远端保持原值，不向它们推送。
哨兵部署目录为 `/home/dji/hero2026_sentry_real`，不要对旧目录执行 pull/reset。
若哨兵没有公网路由，可通过临时 SSH SOCKS 转发获取 GitHub 源码/依赖，
或用已推送提交的 Git bundle fetch 保留提交身份；不复制笔记本二进制。
在哨兵本机执行 `./build_hero2026_safe.sh --jobs 8`，无 ninja 时自动使用 make。
后端 GTSAM、控制器 acados 等仍在新工作区隔离构建，不覆盖系统安装。

首次上车前：确认两个雷达 IP 和主机接收地址（参考 JSON 为 192.168.1.50），
确认 `/dev/chassis` 设备及权限，测量今年外参，设置实际初始位姿和二维地图。
需要全局纠偏时另行采集真实三维先验包并传 `prior_map:=...`；默认空值只做
里程计因子图对照，不能用前后端差值证明满足 30 cm 绝对误差要求。
