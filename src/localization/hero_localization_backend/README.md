# Point-LIO + Z-Localization：旁路阶段

日期：2026-10-25。此包**没有接管导航**，不是“漂移已经修好”的验收结论。
原 navigation_point_lio_sim.launch.py 链路、Point-LIO 实现及参数、PGM、DOG、
A 星、MINCO、MPC、速度及电机链保持不变。

## 版本与边界

- 算法起点：codex/split-static-planning / 33cb2af。
- 新分支：codex/point-lio-backend；仿真仓库不修改。
- 后端来源 https://github.com/mozihe/z-localization ，
  固定 76e126328b670adc92981d9b4900cc059a6f2343；保留上游 GPL-2.0-only 许可证。
- GTSAM 4.2 固定 4f66a491ffc83cf092d0d818b11dc35135521612；
  安全脚本在工作区 install/gtsam 安装，不用 sudo，不改 /usr/local。
- 上游算法源文件原样保留；本包负责启动、显示和地图准备。
- legacy 兼容输入不具备完整 ZLIO 会话和前端重置协议。
  Point-LIO 若内部已严重发散，仍需停车后重启整套算法，不能声称已自动重置修复。

## 数据链和坐标

输入只有 /red_standard_robot1/pointlio/odometry 和
/red_standard_robot1/pointlio/cloud_registered，均为扫描结束时间。
前者是 odom -> pointlio_imu，后者坐标为 odom；
2026-10-31 C2 统一主 TF 后只修改此输入坐标名，原生数值不变；后端仍是私有 TF
旁路评估，未自动接管导航。主链路与交接说明见 hero_sim_bringup 的
`UNIFIED_LOCALIZATION_TF.md`。
input_cloud_in_local_frame=false 让后端按**同一帧位姿**还原传感器系点云。
没有使用真值配准后的 cloud_registered_map，没有订阅真值来修正后端。
同步容差 1 ms，避免把相邻 50 ms 的两次扫描配错。

虚拟 IMU 的轴与底盘平行，底盘到 IMU 是 [0,-0.167,+0.121] m。
上游 body.offset 定义相反，故设置 [0,+0.167,-0.121] m。
这不是重新修改雷达安装位置，也不复用真车安装外参。

后端节点是 /red_standard_robot1/z_backend/backend。
后端 TF **仅**发布到 /red_standard_robot1/z_backend/tf，
不会竞争当前导航的 map->odom 或一次性对齐。
全局修正点云发布到 /red_standard_robot1/z_backend/cloud_registered_map。
后端禁用 UWB、运行时回环和旧 robot_navigation 输出，保留先验匹配、因子图和恢复能力。

startup_hint 为标定出生位置 [0,0,0.076] m，不是实时真值，也不会在漂移后重复锚定。
它仍须通过先验地图几何匹配才算初始化成功。
请先在默认 2027 出生位置启动算法，再行驶；中途才启动旁路需重新给合适初值。
CAD 的 map XY 使用名义出生点 (4.6,6.4)，Z 使用仿真场地地面标定；
旧真值观察器取启动时首次平面里程计为原点，沉降的毫米/厘米偏差需与漂移区分。

## 编译和三维地图

先正常关闭算法；不要在运行中替换动态库。

    cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
    ./build_hero2026_safe.sh --jobs 12
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run hero_localization_backend prepare_rmuc_2027_prior.py \
      --sim-repo /home/apocalypse_linux_00/build_sim_docker \
      --output-dir "$PWD/localization_maps/rmuc_2027_sim_v1"

三维先验与手工 PGM 分开。工具仅接受已审查修复模型
rmuc_2027_tunnel_lips_repaired.stl 的固定 SHA，
应用相同 SDF 平移及名义出生 XY，保留斜面、立面、洞顶和高地。
全场表面采样为 5 cm；关键帧云为 10 cm，模拟扫描半径 12 m，
根据垂直交点选择支撑高度，再使用角度分桶近似遮挡。
SC 描述子的范围、密度和高度与后端配置一致。
这些是**CAD 合成的定位关键帧**，不是实际雷达建图结果；
后续真车应使用实测三维地图，重复对称场地的全局召回还须实测。
生成不会读写 PGM，也拒绝覆盖已有输出目录。
大点云文件留在 localization_maps/（不纳入源码提交），生成清单保留源 SHA、坐标及每个输出 SHA。

## 仍用两个终端

仿真终端维持 2027 入口：

    cd /home/apocalypse_linux_00/build_sim_docker
    SIM_ROBOT_PROFILE=tunnel ./scripts/start_sim_rmuc_2027_dual_lidar.sh

算法终端将旧算法命令替换为下面这一条；**不要与旧算法入口同时启动**：

    cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch hero_sim_bringup navigation_point_lio_backend_shadow_sim.launch.py \
      field:=rmuc_2027 lidar_mode:=dual costmap_mode:=split \
      relay_cmd_vel:=true publish_debug_visualizations:=false \
      point_lio_rviz:=false backend_rviz:=true \
      filter_high_columns:=false high_column_min_z:=0.20 \
      prior_map:="$PWD/localization_maps/rmuc_2027_sim_v1"

原三种模式和 split 均仍由旧入口处理；新包装只附加旁路节点。
后端窗口：灰色为三维先验，绿色为仿真真值轨迹（只作观察），
蓝色为原前端轨迹，青色为后端可信轨迹。
原点仍在名义出生点 XY；右键/滚轮缩放、中键平移。
后端成功不代表当前导航已获纠偏，导航此阶段仍按旧链运行。

如果仅附加到已经运行的算法，可单独启动
ros2 launch hero_localization_backend point_lio_backend_shadow.launch.py prior_map:=...
但这是可选第三终端，不是常规启动必需。

## 观察与后续验收

    ros2 topic echo /red_standard_robot1/z_backend/backend/localization_state
    ros2 topic echo /red_standard_robot1/z_backend/comparison/error

比较器只读真值与后端位姿，同时间插值，不用最新位姿冒充同刻数据。
输出 xy_m/xyz_m；轨迹有界 2000 点，不写隐藏 JSONL。
INITIALIZING/SUSPECT/LOST 不累计为可信轨迹；错误状态会在 RViz 橙色提示。

待实测事项：默认出生能否稳定 NORMAL；正常运动误差/耗时；
三次历史漂移能否纠偏；走廊退化和对称场景能否拒绝错误匹配；
前端真正发散时能否可靠停车并在受控重启后恢复。
旧录包的雷达位置可能是内移 5 cm **之前**，重放必须按录包外参，不可直接套当前配置。
完成这些后才能接管 map->odom，并同时处理点云、速度连续性和导航可信状态门控。
此阶段未做这些接管改动，更没有用仿真真值持续纠偏。

另一个尚未接管的接口细节：当前 Point-LIO 的 Odometry 主要填位姿，twist 未完整填充。
不能把上游依赖速度输入的所有异常检测分支视为已经适配。此阶段验证的是点云几何约束
与位姿因子图；后续需要完善局部速度和前端重置接口，再做导航接管。

## 2026-10-25 实际验证记录

- 两次安全脚本运行均成功，使用 --jobs 12；47 个安装目录、143 个 ELF，
  缺失动态库、断链和临时 RPATH 均为 0。GTSAM 仅安装在本工作区。
- 旁路接口/地图格式及原代价地图模式回归：205 项通过，耗时约 5.85 秒。
  首次测试被沙箱的默认 ROS 日志目录只读限制阻止；指定临时日志目录重跑，
  没有为此修改业务逻辑。
- 使用实际 backend_node、实际三维先验，在独立 ROS 域 207 注入合成扫描，
  先静止初始化，再逐渐增加 30 cm 的 X 方向里程计偏差。
  最终真值车体位置 [0.2,0,0.076] m，后端输出约
  [0.224997,-0.001146,0.075112] m，XY 误差 0.025023 m；
  292 条全局位姿输出，状态 INITIALIZING -> NORMAL，日志确认先验因子接受及图更新。
- 后端实际订阅只有时钟、参数事件和 Point-LIO 的 odometry/cloud_registered；
  没有真值订阅。公共 /tf、导航 odometry/filtered 和 cmd_vel 消息均为 0。
- 合成测试不是实际雷达视野/动力学/过洞测试，不代表真车达到 2.5 cm 精度，
  也不代表对称场景全局重定位和 LOST 自动恢复已验收。
- 三维地图：528241 个表面点、108 个关键帧、约 30 MiB。
  map.pcd SHA256：
  d431bfb7deab1d8cc93c9f9b47b5cdb1fd7ea5780a6a08f40e9afdeafdf3c7de
- 地图生成清单 SHA256：
  edaa71fd98bb4d5be62fdca4acf59c78c9744b0d8f7bf1f71c3d6cea5158c27c
- 原手工 PGM SHA256 未变：
  650be768ca3eda4655856be5c407707ab6b553eed8f5c34ed9381427ec86e1ba
- 测试证据保存于本任务 visualizations 目录的 z_backend_20261025/，
  包含 contracts.xml、runtime_metrics.json、runtime_backend.log。
- 没有启动实际 Gazebo 测试，没有合并 main，没有创建“过洞通过”里程碑标签。

## 2026-11-05 窗口与对称恢复修复

已用真实录包和最小用例分别复现窗口 SIGSEGV 与对称半场误恢复，修复与能力边界见
[本次修复记录](BACKEND_RECOVERY_FIX_20261105.md)。
SUSPECT 保留有位姿连续性约束的就近自动恢复；当前无 UWB 的 Point-LIO 适配在真正
LOST 后不再自动全场猜位置。后端仍未接管导航，恢复期间冻结输出仍明确不可信。
`backend_debug:=true` 现使用 `output=both`，不再依赖用户从滚动终端手动寻找调用栈。

以下 2026-10-25 内容保留为历史记录：当时仅修复显示和增加调试入口，未根治崩溃；
其中“screen 输出会自动进入 launch.log”的判断已被本次实际验证纠正。

## 2026-10-25 现场退出排查与显示修复

### 已确认的故障链路

本次现场算法启动目录为
`~/.ros/log/2026-10-25-21-08-45-525648-Legion-R9000P-ADR10-88087/`。
其 `launch.log` 记录 backend_node PID 88128 于 21:10:51 以 -11 退出，即 SIGSEGV。
这不是正常的定位降级，更不是已经确认的“匹配错了几米”。

后端最后一个轨迹点的仿真时刻约 58.65 s，位置为
[6.995205, 8.659209, 0.119297] m；现场取样时仿真已到 164.35 s。
此时真值约 [0.316040, 8.191104, 0.145324] m，原 Point-LIO 同刻 XY 误差约
0.0863 m。把已经停止的旧后端位置和现在的真值比较，就会看见约 6.7 m 的差距，
但两点并不是同一个时刻。旧比较器仍显示缓存的 NORMAL 和约 1 mm 的历史误差，
而 RViz 的原生 PoseWithCovariance 箭头也不会随发布进程退出自动消失。

最后一条匹配日志是一次 NIS 冲突、进入 OBS_DEGRADED；它发生在退出前约两秒。
没有崩溃调用栈，不能据此认定该冲突、固定窗口、GTSAM 或某一个函数就是段错误根因。
系统未保留本次 core，无法事后还原任意线程的现场内存。

### 本次实际修改及边界

1. 比较器同时检查位姿的采集时间和墙钟接收时间。采集落后 2 仿真秒或
   5 墙钟秒没有新位姿时显示 `STALE - NO FRESH BACKEND POSE`，不再把缓存的
   NORMAL 当作当前健康状态；暂停仿真也可能触发，提示明确保留这个区别。
2. 看门狗使用独立稳态时钟，即使 `/clock` 停了也能告警。
   误差始终标明 HISTORICAL 及原采集时间，不把历史小误差重标为当前误差。
3. 当前车位改用比较器控制的青色箭头，不可信或过期即发 DELETE；
   原生“最后一次位姿及协方差”显示保留但默认关闭，供手动诊断使用。
   青色历史轨迹不删除；后端扫描显示 2 秒后过期。
4. 增加可选启动参数 `backend_debug:=true`，只让旁路后端运行在 GDB 下。
   SIGSEGV 时所有线程调用栈进入该次 `launch.log`；保留非零退出状态，
   没有自动重启、没有吞掉异常、没有切换到真值或假定位。默认 false。

没有修改上游估计器、Point-LIO、噪声/匹配/固定窗口参数、导航、PGM、模型或 TF 归属。
修复的是已经有证据的失效显示问题；**原 SIGSEGV 根因仍未定位、未修复验收**。

### 验证与可复现性

- 13 项包装接口与显示健康判断测试通过，包括：缓存 NORMAL 过期、仿真时间停止、
  LOST 不可信和调试器缺失/错误退出合同。
- 独立 C++ 图测试：40 个固定随机种子、每组 500 个节点，转弯/起伏/噪声/绝对校正/
  降秩观测/30 秒窗口淘汰；20,000 个里程计节点通过，未复现 SIGSEGV。
- 完整移动 CAD 场景测试运行到约 66 仿真秒，后端仍输出 NORMAL，无段错误；
  但观察端末尾接收滞后导致“最新输出时刻”断言失败，故不把整项测试记为通过。
- 临时 AddressSanitizer + UndefinedBehaviorSanitizer 编译版，使用旧录包的 683 组
  雷达/里程计重建输入，并在前面增加 30 秒静止段，运行至 74.1 仿真秒，
  经历 NORMAL/OBS_DEGRADED/SUSPECT/重新匹配，没有非法内存报告或段错误。
  没有公共 TF、导航位姿或速度消息泄漏。使用旧包的 0.217 m 外参，未修改现行外参。
  **这不是原事故的精确回放**：旧包缺少配准点云，重建没有复原 Point-LIO 的逐点去畸变。
- GDB 开关另用一个故意 SIGSEGV 的小程序验证：得到 SIGSEGV 和线程调用栈，
  退出码非零。这个小程序测试不冒充真实后端事故复现。
- 已安装的 `backend_debug:=true` 入口通过 15 秒独立 ROS 合成输入测试，
  逐渐注入 0.30 m 原始里程计偏差，末尾同刻 XY 误差约 0.0276 m；
  公共 TF、导航位姿、速度消息为 0。它仍只是隔离接口测试，不是 Gazebo 验收。
- 加上既有代价地图和 split 模式，合计 207 项测试通过；真实 ROS 冻结时钟测试
  也确认：停止 /clock 和新位姿后 5 秒左右报警，当前箭头被删除。
- 本次安全脚本 --jobs 12 编译成功，47 个安装目录、143 个 ELF，
  缺失库、断链、临时 RPATH 均为 0；原 PGM SHA256 未变。

现场证据在本任务 visualizations 目录的 `z_backend_live_failure_20261025/`；
后续排查证据在相邻 `z_backend_fix_20261025/`。临时诊断源、ASAN 构建不进入生产安装。

### 下次运行捕获真实堆栈（仍为两个终端）

安全编译需先关闭算法，使用 `./build_hero2026_safe.sh --jobs 12`。
仿真入口维持不变，算法用下面一条替换旧启动命令：

    cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch hero_sim_bringup navigation_point_lio_backend_shadow_sim.launch.py \
      field:=rmuc_2027 lidar_mode:=dual costmap_mode:=split \
      relay_cmd_vel:=true publish_debug_visualizations:=false \
      point_lio_rviz:=false backend_rviz:=true backend_debug:=true \
      filter_high_columns:=false high_column_min_z:=0.20 \
      prior_map:="$PWD/localization_maps/rmuc_2027_sim_v1"

若再次出现 STALE，请保留该次终端和 `~/.ros/log`，查看是否有
`received signal SIGSEGV` 及后续 `#0` 等调用帧。仅 STALE 也可能是暂停或输入中断，
不能单凭它断言进程崩溃。调试运行增加进程和少量调试开销，不作为实时性能验收。
没有合并 main，没有创建“后端已修好”的里程碑标签。
