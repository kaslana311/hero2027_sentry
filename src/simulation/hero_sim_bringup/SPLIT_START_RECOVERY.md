# Split：静态先验内的起点恢复

日期：2026-10-21（UTC+08:00）。算法分支 `codex/split-static-planning`，
父提交 `2440452`。不修改另外三种代价地图模式，也不修改仿真模型或镜像。

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

1. 起点在先验图中合法时，仍原样调用原Smac Hybrid A*。未知通行策略也保持原设置。
2. 仅当先验碰撞检查拒绝起点时，搜索默认0.60 m内的先验合法栅格中心。
   优先最短脱离距离，等距时偏向目标（忽略浮点地图带来的亚微米误差）。
   接回点不能是未知或硬碰撞，并保留与原转弯半径/格子大小匹配的起步余量，
   避免Dubin模型面对紧贴的墙、没有有效第一步。
3. 从真实起点到接回点的整条短直线，按两张图最细分辨率的半格采样，使用Nav2
   同类的圆形/footprint和航向离散碰撞检查。动态已占据、未知、越界或地图未就绪均拒绝。
   该短段允许麦轮底盘保持航向侧移，不宣称适用于非全向汽车底盘。
4. 只执行一次原Smac全局搜索：接回点到目标仍全部使用不变的静态先验，
   不增加多次全图搜索，不修改其时间预算、搜索模型或代价参数。
5. 搜索后重查短连接的最新动态代价，再拼成
   **真实起点 → 短连接 → 原静态A星路线 → 目标**，没有起点瞬移或缺失连接段。

PGM和两个costmap均不写入、不清墙、不复制成另一张可修改先验图；也没有空点云、
旧路线缓存、全场动态A星回退、额外ROS节点/线程或速度发布器。
正常起点的搜索行为不变，MINCO仍只用动态global_costmap。
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
