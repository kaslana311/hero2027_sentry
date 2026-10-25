# C2：Point-LIO 位姿和配准云共同接入导航

2026-10-08，两仓库 `codex/point-lio-navigation`，从配对标签
`sim/point-lio-cloud-baseline-v1` 新建。旧 C1/真值/旁路入口保留；原标签不移动。
C2 是新实验，尚未由用户实跑验收，不宣称过洞成功或真车定位完全复现。

实现提交 `1bbf80c53122f7d13918dc8abdd76848a4a2c97b`，
2026-10-08T16:54:58+08:00，`feat(sim): add isolated Point-LIO navigation takeover`。
算法分支起点 `1530e45eb1a1449598b56a9abc6bf522d32a261a`；仿真起点
`32cd4f4cec776200983628d551be0270d88bace5`。两边同名旧标签不移动。

安全脚本 `./build_hero2026_safe.sh --jobs 4` 完成38包构建、42安装前缀/118 ELF；
断链、缺库、临时RPATH均0。13项新增测试单独通过；全55项算法适配回归
42.39秒通过，包含旧42项和实际点云桥/生产DOG map接收测试。仿真14项1.32秒
通过。已安装新入口 `--show-args` 解析成功，关键launch与源码相同。
隔离域183、随机命名空间，仅合成测试指令，不运行Gazebo/Nav2，不发布实际
机器人的运动指令。首轮编译发现Humble时钟now接口的const限制，已修正后重编。
这些是接口/回归证据，不替代完整实跑，更不等于真实逐点扫描去畸变验收。

## 启动

仅运行以下一套算法入口。先停止旧算法、不要重复启动独立 Point-LIO 旁路。
仿真仍是2026双雷达/tunnel，两个终端即可，没有新镜像。

```bash
# 终端一，已有相同仿真可跳过
cd /home/apocalypse_linux_00/build_sim_docker
SIM_ROBOT_PROFILE=tunnel ./scripts/start_sim_rmuc_2026_dual_lidar.sh
```

```bash
# 终端二，干净环境
source /opt/ros/humble/setup.bash
source "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))/install/setup.bash"
ros2 launch hero_sim_bringup navigation_point_lio_sim.launch.py \
  field:=rmuc_2026 lidar_mode:=dual costmap_mode:=combined \
  relay_cmd_vel:=true publish_debug_visualizations:=false point_lio_rviz:=true
```

先静止初始化约10秒，再短距离平地运动、往返、小坡和过洞。主导航位姿现在是
LIO；对照窗口绿色真值、蓝色 LIO、黄色 map/odom 原点。

## 接口契约

- `pointlio/odometry`（pointlio_odom→pointlio_imu）经过新增导航适配，输出
  `odometry/filtered`（odom→base_link），只此一个导航里程计发布者。
- `T_map_chassis = T_map_lio × T_lio_imu × inverse(T_chassis_imu)`；固定安装
  偏移 `(0, -0.167, 0.121) m`（2026-10-25 两侧雷达各内移 5 cm 后）。定位与点云使用同一个持久化初始对齐。
- `map→odom` 恒等；`odom→base_footprint` XY/yaw；`base_footprint→base_link`
  动态保留高度、roll/pitch。采样时间戳不被现在时间替换。
- 真值桥改发 `odometry/truth_planar`，TF 隔离到 `truth/tf[_static]`，关闭
  其速度转发。`odometry/truth_3d`、真值云仅用于对照和首次对齐。
- Point-LIO 快照没有输出速度，新增适配用连续底盘位姿的采样时间差计算速度，
  将速度表达在 base_link，先扣除 IMU 安装偏移，避免转向时出现假平移。
  这仍是差分区间速度，存在噪声，未额外宣称协方差或电机反馈质量。
- 新桥不订阅真值，也不在漂移/断流时回退真值。首次起点仍由真值初始化，
  不属于无需起点的自主重定位。真实逐点扫描去畸变仍未复现。
- DOG map、Nav2/MINCO/MPC/行为树/规划频率等均不改；C1/C2生成的导航 YAML
  全文相同。仅公共 launch 工厂增加默认关闭的诊断隔离选项。

适配状态：`pointlio/navigation/status`。`ready` 需有效冻结对齐、连续两帧位姿
（间隔≤0.5秒）、非空配准云，采样时间和墙钟年龄≤1秒。控制指令0.5秒未更新
则停止活动指令；断流/暂停时停止，恢复不重放旧命令。时间重置或对齐改变会
锁定 `restart_required`，必须重启整个算法入口。健康状态不是 DOG map 接收
或导航成功的证明；原点云输出仍是 Reliable，DOG map分类输出另行确认。
`relay_cmd_vel:=false` 只禁止这个桥转发，不是硬件急停。

## 对照与版本

停止 C2 后，把上述入口换成 `perception_point_lio_sim.launch.py`，参数不变，
回到 C1（LIO 云 + 真值导航）。换成 `perception_truth_sim.launch.py` 并移除
`point_lio_rviz` 参数，回到原纯真值。都在同一install，不需要重建容器。
原精确版本保留在 `codex/point-lio-sim` 和配对标签；Git不恢复二进制。
构建仍用工作区根目录 `./build_hero2026_safe.sh --jobs 4`。

仿真仓库详细流程和配对记录：`docs/POINT_LIO_NAVIGATION.md`、
`experiments/2026-10-08_point-lio-navigation.md`。版本标签是固定提交名，
不是提交说明，也不是会随新提交前进的分支。

## 2026-10-25：目标已接收、控制器有速度，但最终底盘速度为零

现场使用 `navigation_point_lio_backend_shadow_sim.launch.py`，2027/dual/split。
这里的后端是旁路，导航仍读取原 Point-LIO，不读取 z_backend 的错误重定位结果。

被动采样 7 秒确认：`cmd_vel_controller` 342 条、`cmd_vel_nav2_result` 122 条，
全部非零；最终 `cmd_vel` 122 条，全部为零。规划和平滑继续成功，FollowPath 活跃，
控制器随后因车不移动报告 `Failed to make progress`，进入恢复重试。
这不是 A 星服务卡死、目标未接收或底盘没有驱动力。

直接停车原因是 C2 导航桥的 `stale_cloud` 安全门控。
例如位姿 stamp=528.75 s，最后有效云仍为 511.35 s；之后位姿到 584.50 s，
云仍停在 511.35 s。点云桥持续发布状态，dropped=0、queued=0，且无 reset/layout
错误；Point-LIO 的 body 云和位姿继续更新，原始 registered 云及地图系云却收不到。

实际原始云发布者是 RELIABLE，而两段关键输入原来都是 BEST_EFFORT：

    Point-LIO registered [RELIABLE] -> 点云桥 [原 BEST_EFFORT]
    地图系云             [RELIABLE] -> 导航健康门控 [原 BEST_EFFORT]

这种组合在 ROS 2 是兼容的，**不是** 2026-10-02 的 QoS 不兼容故障，不能混淆。
但是 BEST_EFFORT 接收不请求丢失样本的重传，不能据发布端 RELIABLE 就宣称接收可靠。
现场新建可靠订阅测试期间两路云恢复、状态变成 ready；测试退出后短时观察继续正常。
这一先后关系支持加强可靠接收，但不足以证明 Fast DDS 内部具体停更机制，也不是
重演了原长时故障。主进程调试附加被系统 ptrace 策略拒绝，未拿到内部线程堆栈。

本次仅把上述两个云输入改为 RELIABLE / KeepLast(5)，保留有限队列。
原 Point-LIO 发布策略、位姿估计、一次对齐、消息时间戳、DOG/代价地图、
A 星/MINCO/MPC、行为树、速度增益和所有超时门限不改。
断云仍会停车，不缓存或重放旧速度、不发假空云、不回退真值。
已有测试的模拟发布器同步改为 Point-LIO 实际使用的 RELIABLE 契约。

验证包含两个实际接收端 QoS 断言、240 KB/帧的大点云流以及既有过期云/暂停/恢复
不重放旧命令的隔离 ROS 测试。它们证明接口修正及保护保留，不代表长时实跑已验收。
证据在本任务 visualizations 的 `nav_stall_20261025_2221/`。

安装仍使用工作区根目录 `./build_hero2026_safe.sh --jobs 12`，必须先关闭算法；
安装后重启完整算法入口再重新发目标。Gazebo 可以保留，不需要新镜像。
若之后还出现 stale_cloud，应再次检查两段实际云输出，不应通过放宽/取消门控继续走。

本轮实际验证结果：临时独立编译程序 17+14=31 项通过；安全安装后，使用
`install/hero_sim_bringup/lib/hero_sim_bringup/` 下的正式程序重新运行同样两组，
仍为 17+14=31 项通过。正式点云组 16.49 秒，导航桥组 9.40 秒，使用隔离
ROS 域 180/183；没有向正在运行的 Gazebo 或底盘发目标、速度或服务请求。
这里的“大点云测试”不注入网络丢包，也没有覆盖原现场长时 DDS 停更条件。

`./build_hero2026_safe.sh --jobs 12` 成功：42 个包构建步骤、47 个安装前缀，
检查 143 个 ELF；断链、缺失共享库、临时 RPATH 均为 0。完整构建日志、
两次测试日志/XML 和现场数据存放在：
`/home/apocalypse_linux_00/.codex/visualizations/2026/09/24/01a0d39c-2651-74c3-88fa-9206c221d0f1/nav_stall_20261025_2221/`。

可靠接收不是“永不丢数据”的保证：传输拥塞仍可能增加排队/重传延迟，可靠
发布也可能受到慢接收端的背压。保留深度 5 和原有采样年龄检查，延迟过大时仍
停车；不通过扩大缓存或放宽超时掩盖问题。原现场尚需长时复测，不宣称彻底根治。
后端旁路的对称场地错误重定位和 SIGSEGV 未在这次通信修正中解决，须分别处理。
