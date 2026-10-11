# 感知与导航迭代分支

日期：2026-10-11（+08:00）。算法分支：`codex/perception-navigation-iteration`。
本分支从已提交的 `codex/point-lio-navigation` 创建，不移动原分支或里程碑标签。

## 配对起点与实验边界

- 算法起点：`e3607f8e7ce5059a74f4487bdd5484be25a46b2f`。
- 本轮实现：`fe8bf8a4f5e7e0a5215e0131383b6d4205d8fc85`，提交时间
  `2026-10-11T14:48:16+08:00`；文档另作提交，不混入实现消融。
- 仿真实现配对：`70813d9771581bf4bdd317caa1053c9fff1cbe5e`，分支仍为
  `codex/point-lio-navigation`；后续纯文档提交不改变模型或镜像。
- 开始前两个仓库的已跟踪、暂存和未跟踪状态均干净，无需创建空提交。
- 用户已反馈完整 Point-LIO 导航能正常通过部分洞口；随后发现四个洞中不同方向
  的洞口出现不同封堵结果。不能将之前的定性反馈写成全部四洞验收通过。
- 同洞转向后封堵消失的反馈支持朝向/采样相关性，尚无完整前后位姿、相同网格的
  高度统计或 A→B→A 重复实验。`(-4.7992, 4.5162)` 的单份真值航向约 0.084°，
  不能单独证明完成了 90° 对照。截图中的红点是 `cloud_registered`，不是原始三维
  占据状态；`rog_map/ground` 也不是分割后的纯地面。

## 五项工作及当前优先级

1. **当前优先：洞口感知。** 在 DOG 输出分类中加入默认关闭的高处整列过滤，
   保留旧条件作为对照；先验证四洞、双向和不同车身朝向，再决定是否作为默认方案。
2. **待讨论：短时路径被堵。** 区分暂态误占据、真实新增障碍、控制失败和物理接触；
   比较等待/重规划/保留参考路径。A* 本身也是规划算法，不等于可以安全穿过当前障碍；
   保留旧路径时仍须保留碰撞检查。此轮不改规划器、路径更新频率或重试预算。
3. **后续：小台阶、坡面侧缘。** 三维接触能力与二维通行判定分别验证；不得通过
   无条件删除低障碍掩盖物理卡滞。此轮不改变底盘、地形、地面阈值或脱困动作。
4. **后续：Nav2 辅助插件审计。** 逐项检查进度/到达判定、恢复行为、速度平滑、
   行为树调用关系、超时和话题契约，以独立提交记录；不预设全部插件都需要替换。
5. **规则发布后：信息流与决策树。** 新赛事规则尚未发布，先记录接口清单和需求，
   发布后再核对消息、裁判状态、决策树及叶子节点。此轮不猜测规则、不改决策代码。

## 第一项：高处整列输出过滤

启动参数（同时适用于 truth、C1、C2 三个已有算法入口）：

- `filter_high_columns:=false`：默认关闭，使用旧的分类和发布行为。
- `filter_high_columns:=true`：开启实验分类，同时允许发布空障碍点云替换旧输出。
- `high_column_min_z:=0.30`：非负有限数，单位米；是 **map 坐标绝对 Z**，不是
  雷达高度、车身相对高度或某区域地面之上的高度。0.30 只是可调整的实验起点，
  尚未通过四洞实测或真车标定，不是适用于所有赛场的通用数值。

每个 XY 网格柱先查找内部最低/最高占据体素。只有最低占据体素的下表面满足
`lowest_layer * resolution_z - cloud_z_offset > high_column_min_z`，才忽略整列
的二维障碍输出。使用下表面而不是中心，阈值相等/跨越阈值的体素保留；比较使用
1e-9 m 数值容差。双 MID360 的实际内部平移是 0.30 m，Odin 入口是 0.26 m；
不使用历史 `fake_layer_z` 的 0.35 m 参考来还原地图高度。

这是**整列最低占据判断，不是拆分多个连续占据段**，也不是逐点删除高处点云。
只要同列有低于阈值的有效占据，新条件就不删除该列，仍执行原跨度/高处薄层/
最大空洞过滤。内部地面和洞顶体素、射线更新、占据阈值、记忆衰减都不变。
开启时空结果也发布，防止 fast_layer/RViz 缓存上一份非空障碍云；关闭时保留
原有非空才发布的行为。Nav2 静态地图障碍不会被这个 DOG 过滤清除。

风险：墙脚漏扫时，高处墙面也可能被当成高处整列忽略；低台阶若完全没有有效
占据，也不会凭此条件恢复。高地面区域的绝对高度会造成误判。未来语义分区/局部
地面参考必须单独实现和验证，不能声称当前已支持相对高度或严格三维净空判断。
底盘碰撞高度也应纳入阈值选择；此开关不应未经验证用于真车。

## 构建和启动

只有算法代码改变；不新增场地、Docker 镜像、容器或算法入口。
停止原算法再编译，不需要换一套 build/install：

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
./build_hero2026_safe.sh --jobs 4
```

仿真入口不变（已有相同仿真在运行时，不重复启动）：

```bash
cd /home/apocalypse_linux_00/build_sim_docker
SIM_ROBOT_PROFILE=tunnel ./scripts/start_sim_rmuc_2026_dual_lidar.sh
```

新算法实验，干净终端，仅启动一套算法；先静止初始化约 10 秒：

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
source /opt/ros/humble/setup.bash
source "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))/install/setup.bash"
ros2 launch hero_sim_bringup navigation_point_lio_sim.launch.py \
  field:=rmuc_2026 lidar_mode:=dual costmap_mode:=combined \
  relay_cmd_vel:=true publish_debug_visualizations:=false point_lio_rviz:=true \
  filter_high_columns:=true high_column_min_z:=0.30
```

同版本消融：停止算法，把最后一行改为
`filter_high_columns:=false high_column_min_z:=0.30`，重新启动。
truth/C1 入口使用相同两个新参数；truth 不传 `point_lio_rviz`。
参数启动时读取，不提供运行中动态修改的假承诺。

```bash
ros2 param get /red_standard_robot1/dog_map_node filter_high_columns
ros2 param get /red_standard_robot1/dog_map_node high_column_min_z
```

## 验证与验收

已完成：

- 使用 `./build_hero2026_safe.sh --jobs 4` 完整构建与安装检查：42 个安装前缀、
  118 个 ELF，断链/缺失动态库/临时 RPATH 均为零。第一次进程以 143 终止，
  重跑安全脚本成功；没有绕过脚本或修改编译流程。
- 隔离 ROS 域 187，全部算法适配测试 **77 项通过，48.15 秒**；旧 55 项加新 22 项。
- 新增 17 项覆盖生产 C++ 分类器的阈值边界、人工平移、非法高度、生成参数边界；
  5 项启动实际 `dog_map_node_lidars`，验证开/关、不同阈值、低处障碍、地面连接墙柱、
  最后低处障碍衰减后发布空替换。不是复制一份算法到 Python 来验证。
- 已安装的 truth/C1/C2 三个 launch 文件与源码逐字节一致，`--show-args` 均列出
  两个新参数及默认值。真实导航 YAML、规划器、MINCO、行为树文件没有修改。
- 仿真仓库原有 14 项测试通过（1.28 秒）；模型/镜像/启动脚本无改动。

Odin 变体一起编译，但上述节点运行测试是双雷达 DOG 变体，不是 Odin 联合实跑。
空云测试验证输出契约，不等于完整 Nav2 costmap/四洞通行验收。
本轮不启动 Gazebo、Nav2 或向实际机器人发送控制指令；不将自动测试等同于
Gazebo 联合过洞通过。

实跑清单：

- 为四个洞固定编号、目标点和车身朝向；同路线做开/关对照，至少重复三轮。
- 对旧失败洞先比较 DOG `rog_map/inf_occ` 和 local/global costmap，再验证自主通过。
- 低台阶、墙脚、地面连接墙体必须仍被有效感知；专门检查漏扫墙脚时的高墙风险。
- 坡上、坡下、跨坡时检查绝对高度阈值的局限；记录定位误差和空点云清除情况。
- 记录阈值、两仓库 SHA、镜像 ID、成功/失败及 Nav2 action 状态，不只保存截图。

## 切回原实现

参数关闭即可恢复旧分类；要精确切回源码，在停止算法且提交完新分支修改后：

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
git switch codex/point-lio-navigation
./build_hero2026_safe.sh --jobs 4
```

另开干净终端重新 source，运行原命令。Git 只切换源码，不会自动恢复 install、
进程或环境。仿真实现此轮不变，无需切仿真分支或重建镜像。
不合并稳定分支、不移动历史标签、不推送远程；通过实测后再讨论里程碑。

## 2026-10-11：MINCO 软安全距离增加 0.50 m

本次仍在 `codex/perception-navigation-iteration` 上作一次独立提交，不新建分支。
算法修改前为 `108afdbead0fea9ed5f502a95766a93672e84c31`；仿真配对为
`40adfd22abcf782fd42bce49fbf81d66830f8ee3`（移除 tunnel 后方灯条几何），
本次不改仿真仓库、不构建镜像。

只修改 `hero2025_nav_bringup/config/nav2_params.yaml` 中
`smoother_server.ros__parameters.minco_smoother.safe_distance`：
**0.30 m → 0.80 m**，增加 0.50 m。它是 MINCO 的软避障惩罚作用距离，
相对于 ESDF 的硬障碍边界；当前该边界已包含代价地图内切圆硬膨胀带。
不改变 costmap 膨胀半径 0.40 m、衰减因子 2.5、两阶段避障权重、规划/曲线频率、
感知过滤、Point-LIO、MPC、行为树或车体尺寸。

现有代码用 `10 * safe_distance` 初始化 ESDF 距离截断上限，因此它自然从
3.0 m 变成 8.0 m；ROI 外扩仍为 4.0 m，没有单独修改 C++ 实现。
0.80 m 是软偏好，不是新增硬禁行圈，但窄洞内可能持续受罚、产生优化困难，
不能据此保证仍能通行；未扫描到的障碍也不会凭该参数产生斥力。

配置范围：这是共享的主 YAML 默认值，**所有加载该 YAML 的入口**都会使用 0.80 m，
不只 `static_only`。没有新增启动参数，也没有改动其它 YAML 变体。
当前 install 中该 YAML 是指向源码的符号链接；不必重编译 C++，但必须停止旧算法
并重启，启动时生成的新临时参数文件才会采用新值。若今后 install 变成普通副本，
应通过 `./build_hero2026_safe.sh --jobs 4` 更新安装，不手改 install 冒充源码修改。

本次沿用已存在的 C2 入口，只将地图模式设成 `static_only`：

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hero_sim_bringup navigation_point_lio_sim.launch.py \
  field:=rmuc_2026 lidar_mode:=dual costmap_mode:=static_only \
  relay_cmd_vel:=true publish_debug_visualizations:=false point_lio_rviz:=true \
  filter_high_columns:=true high_column_min_z:=0.30
```

启动后可只读确认（在另一个已 source 的终端执行）：

```bash
ros2 param get /red_standard_robot1/smoother_server minco_smoother.safe_distance
ros2 param get /red_standard_robot1/local_costmap/local_costmap plugins
ros2 param get /red_standard_robot1/global_costmap/global_costmap plugins
```

预期为 0.8，以及两份 `['static_layer', 'inflation_layer']`。这里的 `static_only`
仅表示不加载 `fast_layer`；既有 `UpdateStaticLayer` 仍订阅 `rog_map/fi`，可清除
静态地图中被观测为空的格子，并不是完全冻结 PGM。不为本次试验修改这项旧行为。

只比较距离参数时，应固定同一地图模式、路线和其它参数，分别测试 0.30/0.80 m；
不要把同时从 combined 切为 static_only 的效果全归因于 ESDF。恢复旧参数只需把
源码同一项改回 0.3 并重启算法，不需要回退其它实现；如需保留进展，作新的配置提交。
验证：YAML 结构差异严格只有上述一个数值；combined/static_only/dynamic_only
三种模式的前后生成参数也只有这一项变化，源码和安装路径结果一致；7 项参数/恢复
契约测试通过。安全脚本 `--verify-only` 检查 42 个前缀、118 个 ELF，断链、缺失
动态库和临时 RPATH 均为零；已安装 C2 入口 `--show-args` 检查通过。
尚未在 Gazebo 中运行导航或发送运动指令，实际轨迹间距和洞口通行由用户测试。

## 2026-10-11：MINCO 两阶段避障权重扩大 10 倍

按用户要求，在 `codex/perception-navigation-iteration` 上继续作一次独立配置提交，
不新建分支。修改前算法版本为 `7df64c4e26683becc55173fa29014632abf8cafa`，
仿真仍配对 `40adfd22abcf782fd42bce49fbf81d66830f8ee3`，本次不改仿真仓库或镜像。

只修改共享主 YAML 的两项：

- `minco_smoother.stage1_weight_obstacle`：8000 → **80000**。
- `minco_smoother.weight_obstacle`：7500 → **75000**。

`safe_distance` 保持 **0.8 m**，其余代价权重、采样、频率、感知、Point-LIO、MPC、
行为树、梯度处理和时间换算均保持不变。这里扩大的是避障代价及其梯度的权重，
不是再增大 ESDF 作用距离，也不将软约束变为硬碰撞保证。两阶段一并调整，
避免第二阶段继续采用旧避障权重；但不保证实际净空增加或洞口仍然可通行。

该 YAML 是所有引用它的启动入口共享的默认配置，三种 costmap 模式都会受影响。
install 中的配置仍为源码符号链接，不需要重新编译；停止旧算法后重新运行原入口。
保持上一轮的地图模式、路线、初始状态和其它启动参数，才是单独比较权重。
最近日志中的 `dynamic_only` 与之前计划的 `static_only` 是两种不同实验，
不要在比较权重时同时切换地图模式。

若按之前约定用静态模式实验，算法命令仍为：

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hero_sim_bringup navigation_point_lio_sim.launch.py \
  field:=rmuc_2026 lidar_mode:=dual costmap_mode:=static_only \
  relay_cmd_vel:=true publish_debug_visualizations:=false point_lio_rviz:=true \
  filter_high_columns:=true high_column_min_z:=0.30
```

在另一个已 source 的终端核对：

```bash
ros2 param get /red_standard_robot1/smoother_server minco_smoother.stage1_weight_obstacle
ros2 param get /red_standard_robot1/smoother_server minco_smoother.weight_obstacle
ros2 param get /red_standard_robot1/smoother_server minco_smoother.safe_distance
```

预期分别为 **80000.0、75000.0、0.8**。如果需要恢复原权重，只将同一源码 YAML
两项改回 8000.0/7500.0 并重启算法；保留本次进展时作新的恢复提交，不回退其它代码。
实际贴墙距离、跟随稳定性和过洞成功率待用户实跑验证，本次没有发送运动指令。

提交前验证：YAML 结构差异严格只有上述两项权重；源码和安装配置一致，
combined/static_only/dynamic_only 三种模式的生成参数均保留 80000/75000 和 0.8；
7 项参数/恢复契约测试通过。执行 `./build_hero2026_safe.sh --verify-only`，
42 个前缀、118 个 ELF 检查通过，断链、缺失动态库、临时 RPATH 均为零。

## 2026-10-11：结束距离/权重实验，恢复原始 MINCO 参数

用户反馈扩大软安全距离和避障权重的测试效果不理想，要求恢复原配置，
暂不实施其它优化器修改，继续思考原因。该反馈不等于已经确定了故障根因。
在 `codex/perception-navigation-iteration` 上作新的恢复提交，保留
`7df64c4`（距离实验）和 `0547dba`（权重实验）及其文档，不撤销历史。

仅恢复主 YAML 三项：`stage1_weight_obstacle=8000.0`、
`weight_obstacle=7500.0`、`safe_distance=0.3`。既有实现自动将 ESDF 距离
截断上限恢复为 3.0 m，ROI 外扩仍为 4.0 m；不修改其它源码、感知过滤、
Point-LIO、MPC、行为树或 costmap 模式。仿真配对仍为 `40adfd2`，不改镜像。

install YAML 仍链接到源码，因此不需要重新编译。停止旧算法，再按此前使用的
启动入口和参数重启；本次没有改变入口或地图模式。启动后检查
`minco_smoother.stage1_weight_obstacle`、`minco_smoother.weight_obstacle` 和
`minco_smoother.safe_distance`，应分别返回 8000.0、7500.0、0.3。

恢复验证：主 YAML 与实验前的 `108afdb` 版本逐字节一致，源码与 install 配置
一致；7 项参数/恢复契约测试通过。没有启动仿真或发送运动指令。
