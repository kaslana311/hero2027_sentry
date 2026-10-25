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
前者是 pointlio_odom -> pointlio_imu，后者坐标为 pointlio_odom；
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
