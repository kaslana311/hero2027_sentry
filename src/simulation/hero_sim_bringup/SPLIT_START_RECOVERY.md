# Split：静态先验内的起点恢复

日期：2026-10-21（UTC+08:00）。算法分支 `codex/split-static-planning`，
父提交 `2440452`。不修改另外三种代价地图模式，也不修改仿真模型或镜像。

2026-10-24 补充修复：本轮父提交 `ec084cb`。除了起点已占据，也覆盖“起点合法，
但原 Smac 规划失败且局部缺少起步余量”的情况；原规划成功时原样返回。
现场采样、触发条件和验证记录见下方同日章节。没有新增参数或修改规划权重。

## 问题证据

2026-10-20 23:32启动的split运行中，23:34:07开始连续出现
`Starting point in lethal space`；23:34:08控制器被行为树取消并停车。
23:36:08新导航目标实际已被接收，从(10.01,-4.07)到(8.29,-4.06)，随后仍因起点
碰撞而失败。退出前记录112次起点碰撞错误，不是新目标收不到。

原恢复动作清理动态global_costmap，而Smac读取独立静态先验；清理不会改变先验
中的无效起点。外层150次重试、局部动态图困住判断和等待组合造成长时间原地重试。
本轮不改变行为树/重试次数，也不据此认定Point-LIO一定漂移。

## 修复边界

仅扩展 `hero_static_prior_planner/StaticPriorPlanner`：

1. 起点在先验图中合法时，先原样调用原Smac Hybrid A*；成功则原样返回。
   失败且起点仍有原接回点判定要求的起步余量时，也保持原失败结果，不扩大恢复范围。
   未知通行策略保持原设置。
2. 先验碰撞检查拒绝起点，或原规划失败且起点缺少上述局部余量时，搜索默认
   0.60 m内的先验合法栅格中心。该余量检查是局部受限的筛选，不是完整Dubin可达性证明。
   优先最短脱离距离，等距时偏向目标（忽略浮点地图带来的亚微米误差）。
   接回点不能是未知或硬碰撞，并保留与原转弯半径/格子大小匹配的起步余量，
   避免Dubin模型面对紧贴的墙、没有有效第一步。
3. 从真实起点到接回点的整条短直线，按两张图最细分辨率的半格采样，使用Nav2
   同类的圆形/footprint和航向离散碰撞检查。动态已占据、未知、越界或地图未就绪均拒绝。
   该短段允许麦轮底盘保持航向侧移，不宣称适用于非全向汽车底盘。
4. 接回点到目标只执行一次恢复Smac搜索，仍全部使用不变的静态先验。
   已占据起点仍只调用一次Smac；合法但受限的起点最多先做一次原搜索、失败后再做
   一次恢复搜索，不循环尝试多个全局搜索，不修改原单次预算、搜索模型或代价参数。
5. 搜索后重查短连接的最新动态代价，再拼成
   **真实起点 → 短连接 → 原静态A星路线 → 目标**，没有起点瞬移或缺失连接段。

PGM和两个costmap均不写入、不清墙、不复制成另一张可修改先验图；也没有空点云、
旧路线缓存、全场动态A星回退、额外ROS节点/线程或速度发布器。
正常成功起点的搜索行为不变，MINCO仍只用动态global_costmap。
不改DOG/Point-LIO/TF、ESDF/MINCO/MPC公式或权重、频率、出生点和传感器。

`plan`是适配器的完整输出（恢复时包含短连接）。Smac自己的
`unsmoothed_plan`调试话题仍只表示其接回点之后的搜索段，不能把它当作完整恢复路线。

## 参数和原入口

同一个split启动命令即可启用，无新launch参数；重启**算法**加载新插件。
若仿真还在运行，无需为了此项修复重启Gazebo或重建Docker镜像。
仍正常退出并重启整套算法，Point-LIO会随原入口重新初始化，无需改变其配置。

在planner_server上，以下两个参数只由split插件声明，每次规划读取，可动态修改：

| 参数 | 默认 | 含义 |
|---|---|---|
| `GridBased.start_recovery_enabled` | true | false恢复原来的起点碰撞失败行为 |
| `GridBased.start_recovery_radius` | 0.60 | 单位米，允许范围(0,2]，非法值拒绝设置 |

```bash
ros2 param get /red_standard_robot1/planner_server GridBased.start_recovery_enabled
ros2 param get /red_standard_robot1/planner_server GridBased.start_recovery_radius
ros2 param set /red_standard_robot1/planner_server GridBased.start_recovery_enabled false
ros2 param set /red_standard_robot1/planner_server GridBased.start_recovery_enabled true
```

成功恢复会打印 `SPLIT start recovery: ... dynamic-checked connector; static prior unchanged`。
没有有效短连接时明确失败，不能把“接回点投影成功”当作能安全离开的证明。

## 验证

采用现有安全脚本，不在带空格/括号的源码目录手写colcon命令：

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
./build_hero2026_safe.sh --jobs 4
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=196 ROS_LOG_DIR=/tmp/hero_split_start_recovery_logs \
  PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q \
  src/simulation/hero_sim_bringup/test/test_costmap_modes.py \
  src/simulation/hero_sim_bringup/test/test_split_mode.py \
  src/simulation/hero_sim_bringup/test/test_recovery_contract.py \
  src/simulation/hero_sim_bringup/test/test_start_connector.py \
  src/simulation/hero_sim_bringup/test/test_split_planner_runtime.py \
  -o cache_dir=/tmp/hero_split_start_recovery_pytest_cache
```

测试使用隔离域和唯一命名空间，不启动Gazebo/真实硬件，不发送机器人底盘速度。
原生C++辅助算法检查最近出口、目标变更、半径、动态占据/未知、薄墙、
改走另一侧及非法输入。真实Nav2集成检查占据先验起点的连续多目标、
动态占据拒绝、开关/半径设置、PGM代价字节不变、正常路线不变及命名空间TF。
另外三种模式继续使用分支前完整参数摘要对照，不放宽其比较条件。

最终验证结果：安全脚本完整编译成功，43个安装前缀、119个ELF审计通过，
断链/缺少共享库/临时RPATH均为0。上面五个限定测试文件共207项全部通过
（13.85秒），包括原生C++的10个案例和两种TF命名空间下的真实Nav2集成。
未运行Gazebo或底盘运动验收，不将这207项称为整仓库/全部感知节点的测试结果。

开发中真实集成曾发现：仅选择最近合法格会因float32地图几何的亚微米误差
选错等距出口，且贴硬碰撞带的接回点可能让Dubin无有效第一步。现修正等距
排序并增加上述起步余量；补充float32回归后，连续左右目标恢复、动态占据
拒绝/恢复、原路线一致和两次节点生命周期均通过。没有通过放宽原A星参数
或改测试的正常路线期望来绕过问题。

## 限制和实际验收

### 2026-10-24：合法起点无法起步的补充修复

用户本次运行于split，目标约(-2.66,-5.05)。现场8秒只读采样中，定位点约
(4.787,-4.480)，动态global/local代价均为0，独立static_prior代价为223；
前方约5 cm出现253硬碰撞。Point-LIO桥ready但command_active=false，没有新的
底盘速度。规划日志持续no valid path found，控制器在15:28:24（UTC+08:00）
记录Goal was canceled后停车。上述时间来自日志Unix时间1792826904.75。

用保存的float32代价图、64朝向格、5 cm最小转弯半径和DUBIN投影离线复算，
起点及第一步总计只有4个可展开状态，后续均进入253区；二维起终点仍连通。
旧恢复只看起点是否碰撞，223合法，因而跳过恢复。这不是触地保护或MPC限速故障，
也不把约16 cm的LIO/真值差异认定为唯一诱因。

本轮仅把已有接回点余量检查复用到“原规划失败”的合法起点；先尝试原规划，
成功时不会因为贴墙而强行改路。失败后余量不足才进入原有有界恢复，动态碰撞、
未知、半径、接回连续性和规划后重查仍保留。没有空点云清图、自动倒车速度发布、
原路线缓存、循环全局回退或PGM改写。也不改另外三种模式、DOG、Point-LIO、
MINCO/MPC、BT重试数及仿真镜像。新增回归要求原模型关闭恢复时失败、开启后连续
多目标成功，同时真正动态占据的同一合法起点仍失败、PGM代价字节保持不变。

现场原始数据及独立重放脚本已保存在本机诊断目录：
`/home/apocalypse_linux_00/.codex/visualizations/2026/09/24/01a0d39c-2651-74c3-88fa-9206c221d0f1/split_start_departure_20261024/`。
其中`snapshot.json`保留现场地图和姿态；`test_frozen_start.py`在隔离域196运行
真实Nav2规划器，不启动仿真/控制器、不发送底盘速度。原恢复关闭时状态6失败；
开启本轮修复后状态4成功，经约5 cm连接接回先验，生成342个路径点，最大相邻
间距约3.83 cm，静态地图字节不变。`replay_result.json`和`replay_planner.log`
保留结果。该对照1项通过（4.91秒），不是整车行驶验收。

验证命令沿用上方安全编译和隔离域测试。现场程序由用户正常关闭后才编译部署。
本轮安全脚本`./build_hero2026_safe.sh --jobs 4`完整编译/安装成功：43个安装前缀、
119个ELF检查通过，损坏链接/缺失动态库/临时RPATH均为0。五个限定回归文件
共209项通过（14.54秒），包括两种TF命名空间下的真实Nav2集成；冻结现场对照
另1项通过。PGM源码和安装文件SHA-256仍为
`c53fc95b7ad4154078cc216f5c2d4a567d67a13e2847b63adb2d9261a4e88e45`。

新增合成案例使用float32栅格中心，并把目标放在原`analytic_expansion_max_length`
之外，确保确实复现DUBIN离散起步受阻。调试曾发现近目标仍可由Smac解析连接成功；
这是应保留的正常行为，不能把它误判为必须恢复的失败。本轮只校正新增测试样例，
未改变解析连接长度、转弯半径、目标容差或任何搜索参数。

实际新一轮Gazebo导航仍须用户用原启动命令复测，不把隔离测试当作实车验收。

- 这是小范围先验冲突恢复，不是定位纠偏，不会修正PGM或Point-LIO的整体错位。
- 动态地图空闲只表示当前代价地图没有标为障碍；dynamic_only原本的未观测空闲
  策略未变，不能据此证明现实中没有漏检墙面或危险地形。
- 选择一条最近的可通行短连接，然后做一次Smac搜索；若接回点与目标在静态图中
  不连通、冲突深于半径、动态图也认为起点碰撞，仍明确失败，不扩展为盲目清墙。
- 下游MINCO仍是原软避障优化器；本次验证的是规划路径短连接，不等于给最终
  优化曲线增加了硬碰撞保证或让所有全局不可达场景都能脱困。
- 150次重试和等待没有变。真正无可行连接时仍可能重试，不把安全拒绝改成假成功。

用户实际验收：保持原split/感知参数，经过之前触发起点碰撞的位置后连续发送
不同目标；观察恢复日志、完整plan和车辆是否确实离开冲突区。未经用户实跑确认，
不合并main、不移动成功标签、不创建“已通过过洞验收”的里程碑。

撤销可先动态关闭上述开关；精确源码撤销使用新revert提交，再用安全脚本编译。
不回退分支历史，不能只切Git却继续使用旧install二进制。
