# split：洞口附近的结构引导局部软规划（实验功能）

日期：2026-11-03。当前算法分支：`codex/point-lio-backend`；修改前 HEAD 为 `33d5126`。
本功能默认关闭，只在 RMUC 2027 的 split 模式下显式启用。仿真仓库、PGM、定位、TF、DOG、
MINCO 参数/PVA 热启动和 MPC/速度接口均不修改。不是定位后端，不会消除定位漂移。

## 1. 接入的是什么方法

保留原来静态 Smac Hybrid A* 的全局路线；在六个隧道候选区域附近，增加一次有界的局部软代价 A*，
将修正后的路线交给原来两阶段 MINCO，而不是让 MINCO 单独承担全部大幅度横向修正。

```text
独立静态 PGM → 原 Smac（含原起点恢复）→ 静态全局路线
                                           ↓
动态 global_costmap 快照 → 区域门控＋双墙拟合＋局部软 A*
                                           ↓
                              修正局部前缀＋原静态后缀
                                           ↓
                          原 MINCO（原动态地图）→ 原 MPC
```

研究依据是已保存实验中表现较好的“区域分型＋局部侧墙软引导”方向。本次部署是其**二维代价地图适配版本**：
使用当前占据单元拟合双墙，而不是照搬离线研究的 PCA 点云法，也不是把未经部署审核的六区域 PCD 直接上线。
所以不能将先前研究里的成功计数不加区分地算成这份新代码的真实导航成绩。

本轮没有接入 GICP、因子图、Scan Context、额外点云旁路或全场刚体修正。

## 2. 为什么不容易从动态墙的缺口钻出去

长洞先以候选区域的轴向为初值，在 ±8° 范围内拟合两侧平行墙：

- 只使用动态 master costmap 中的实际 lethal 单元，不拿膨胀后的整圈 253 当墙面观测。
- 两侧要夹住当前机器人，横向位移不超过 1.0 m，宽度需合理且与候选区域近似相容。
- 两面墙各至少支持 5 个 15-cm 纵向格，共同支持至少 4 格。
- 一条横向的洞口上沿不具有足够纵向支持，不能单独成为双墙证据。
- 通过检查后，在该局部区域补一个有限的**软通道代价**。短时缺失的墙面单元不会让软通道立即消失。

这里只估计通道横向位置、轴向和宽度，不假装沿长墙方向也完全可观。区域的纵向范围仍来自粗略激活模板，
不是完成了三维先验配准。六个区域是激活/置信检查模板，不是新加入代价地图的六组硬障碍。

长洞没有双墙证据时，返回原静态路线，不依据单侧墙硬猜另一侧。当前**没有跨帧通道缓存**；
严重漏扫时本功能可能不生效，原 MINCO 仍按原方式处理。

短洞（模板长度小于 1.5 m）不强求长墙证据，使用有限范围动态软 A*。其可靠性仍依赖正确的感知和通道几何，
不保证任何未过滤的巨大假障碍都能被忽略。

## 3. “软”具体指什么

局部 A* 的单位长度代价为：

```text
1 + dynamic_weight × smoothL1(safe_distance − signed_dynamic_ESDF)
  + guide_weight × excess_from_supported_corridor²
```

动态 ESDF 由原 master costmap 的 `>=253 且 <255` 单元生成，含其现有内切圆膨胀；
没有修改 MINCO 的 ESDF 或权重。默认 dynamic_weight=1、safe_distance=0.30、guide_weight=100，
对应研究性局部搜索的量级，而不是把原 MINCO 避障权重扩大 100 倍。

动态 lethal/253 **不是这次局部搜索的硬禁区**：洞口薄横条仍可付出有限代价穿过；两侧长墙与结构软带累计代价较高。
这不等于能保证不碰墙，也不意味着知道哪一块占据一定是假洞顶。窄薄的真实障碍也可能被软穿越，
因此这是需要观察的实验性能力，不能当作新增安全认证。

仅以下部分硬限制局部搜索：

- 从现有 `boundary_layer` 独立快照取得的场地外圈，再计入机器人半径；不能把外圈也当软代价穿出场地。
- 未知或地图外区域。
- 距原局部静态路线超过 1.25 m 的信任带，以及有限 ROI 边界。

最多修正前方 12 m；接回点应在选中长洞模板末端之外，不会只因短 lookahead 在长洞中段接回错位旧路线。
接回点和全局终点仍保留原静态路线的坐标；本功能不会偷偷移动用户导航目标。
过大的漂移、错误目标、邻洞不同长度、错误区域模板和观测不足仍可能使路线不理想。

## 4. 与原功能的隔离

源码仅在 `hero_static_prior_planner` 内增加返回路径的可选处理。原静态规划/起点恢复主体被原样移到
`createStaticPlan()`，开关关闭时直接返回其结果，没有动态快照或附加搜索。

不修改任何 costmap 单元，不回写 PGM，不向 MINCO 增加新约束，不改变参考速度，不清零热启动，
不订阅或使用真值速度，不新增恢复行为树。原静态 Smac 若已经无法给出全局路线，本功能不替代其原恢复逻辑。

仅在读取快照时短暂加锁；EDT、墙面拟合和局部 A* 不持有 costmap 锁，不同时持有两张地图的锁。
默认额外计算预算为 120 ms / 50,000 扩展。超时、无可信墙或无局部路线时返回原静态结果。
预算不是操作系统级抢占：单次快照/OpenCV 调用不能被中途打断，特别繁忙时总耗时仍可能超过该目标。

另外三种模式与未开启功能的 split 的生成参数保持原样，有原始哈希/全参数回归检查。
这不是让 combined、static_only、dynamic_only 开始用动态局部 A*。

## 5. 启动与关闭

仿真仍使用正确的 **2027** 场地入口，不需要新 Docker 镜像或第三个常驻节点。
算法命令：

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch hero_sim_bringup navigation_point_lio_sim.launch.py \
  field:=rmuc_2027 \
  lidar_mode:=dual \
  costmap_mode:=split \
  split_local_refinement:=true \
  relay_cmd_vel:=true \
  publish_debug_visualizations:=false \
  point_lio_rviz:=true \
  filter_high_columns:=false \
  high_column_min_z:=0.20
```

运行中可以关闭或重新开启本次功能；下一次规划调用读取开关，不会中断当前同步搜索：

```bash
ros2 param set /red_standard_robot1/planner_server GridBased.local_refinement_enabled false
ros2 param set /red_standard_robot1/planner_server GridBased.local_refinement_enabled true
```

重新启动时省略 `split_local_refinement:=true` 或改为 false，即恢复原 split 路径行为。
本轮其余参数是启动配置，只读，位于 `config/split_local_refinement.yaml`；避免运行中更新一半参数的混合状态。
启动开关目前只接受 `rmuc_2027 + split`，错误组合会明确报错，不静默扩散到其它模式。

## 6. 如何看它是否真的生效

- `/<robot>/plan`：最终返回给 Nav2 的规划路线，开启后可能包含修正过的前缀。
- `/<robot>/GridBased/static_plan`：本次原始静态 Smac 路线。
- `/<robot>/GridBased/refined_plan`：本次实际返回路线，回退时与静态路线相同。
- `/<robot>/GridBased/refinement_status`：`result=applied` 表示生效；`outside_tunnel_regions`、
  `no_supported_wall_pair`、`budget_exhausted` 等表示本次没有采用附加修正。
- 原来的 MPC reference / MINCO 显示仍是原平滑链路的输出，不是上述静态原始路线。
- 运行时关掉开关后不再发布本功能的诊断话题；RViz 可能还留着最后一帧 debug 路线。
  此时以持续更新的 `/plan` 为准，不能把残留 debug 图像当成仍在启用。

可选的只读观察命令：

```bash
ros2 topic echo /red_standard_robot1/GridBased/refinement_status std_msgs/msg/String
```

不要只看“没有漂移时能过”就认定抗漂移生效；应在洞前记录两条规划路径、状态和实际通洞结果。
统一 TF 下，可使用已存在的 `sim_point_lio_navigation_bridge` 的 map_odom_offset 参数做受控漂移，
先测 ±0.50 m、再测 ±0.60 m；长洞横向通常是 map Y，短洞通常是 map X。
原 goal 坐标仍是 map 坐标，过大的偏移会同时改变实际目标的物理位置，需将其与通洞能力区分。

## 7. 验证记录与不能承诺的内容

- 精确编译本次 C++ 核心：±0.50/0.60/0.70 m × 横/纵方向 × 正反向共 24 个几何工况。
- 另检查墙面缺口、轴向初值误差、单墙拒绝、短洞薄横条可软穿越、无观测、预算耗尽、硬边界、非法输入和接回窗口。
- 真实 Nav2 PlannerServer＋本插件＋标准静态/膨胀层＋原 FastLayer，用合成地图、TF、点云进行隔离联调。
  先验通道中心 y=1.4，动态通道中心 y=2.0，入口仍有未过滤横条；开启后中段路线约 y=2.025，
  关闭后约 y=1.375，重新启用可再次修正；先验地图字节不变。一次测得附加处理约 4.4 ms，不是最坏耗时保证。
- 正式源树单元/配置/原起点恢复/目标切换回归 **257 项通过**；正式安装插件的真实 Nav2 隔离联调 **3 项通过**，合计 **260 项**，无跳过。
- 完整安装审计：47 个安装前缀、143 个 ELF，断链、缺失共享库、临时 RPATH 均为 0。
- 原静态规划与起点恢复函数（仅改函数名后）与修改前逐字节相同，未改其算法主体。

以上不是完整 Gazebo 动力学通洞验收，也不是 Point-LIO 实际随机漂移统计，更不是“定位误差已被永久限制在 50 cm”。
接入后仍需要用户用当前机器人和实际场地试运行，尤其观察漏墙时回退、两个长洞串洞及短洞目标接回。

## 8. 代码位置

- `src/planning/hero_static_prior_planner/include/hero_static_prior_planner/local_refiner.hpp`
- `src/planning/hero_static_prior_planner/src/local_refiner.cpp`：独立几何/双墙拟合/软 A*。
- `src/planning/hero_static_prior_planner/src/local_refinement_ros.cpp`：只读快照、ROS 参数、路径拼接、诊断。
- `static_prior_planner.cpp`：原静态流程＋可关闭入口。
- `src/simulation/hero_sim_bringup/config/split_local_refinement.yaml`：六区域、预算和搜索参数。
- `test/test_split_local_refinement*.py` 与规划包内 `test/test_local_refiner.cpp`：本次回归入口。

构建经过：先执行原 `./build_hero2026_safe.sh --jobs 12`，但重启后的 `/tmp` 缓存缺失，
无关的 BLASFEO/HPIPM/GTSAM 固定依赖下载缓慢且发生 TLS 失败，故在安装前停止全量下载。
随后使用临时增量驱动导入**同一未修改安全脚本**的函数（不执行其全量 main），复用已有已安装依赖，
只对 `hero_static_prior_planner`、`hero_sim_bringup` 调用原 `configure_build_install()`，
继续走无空格 staging、逐包/12 任务、禁止测试/软链接安装等相同设置，再运行原
`generate_colcon_metadata()`、`bootstrap_top_level_setup()`、`verify_install()` 的完整安装审计。
不是手工复制 `.so` 到 install，也没有修改生产安全脚本或把停下的全量构建说成成功。
本次增量驱动和构建/测试记录保存在仓库内被忽略的
`local_research/2026-11-03_split_local_refinement/`。未来正常全量构建仍使用原安全脚本入口。
