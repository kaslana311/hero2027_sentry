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
