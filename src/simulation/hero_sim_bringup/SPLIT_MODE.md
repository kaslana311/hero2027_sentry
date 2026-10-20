# Split：静态先验选路，动态地图平滑

日期：2026-10-20，UTC+08:00。

同日后续（父提交 `815b666`）：用户要求 DOG X/Y 恢复 5 厘米，现仿真入口默认
`dog_resolution_xy:=0.05`，四模式一致；Z、PGM、Nav2 地图及过滤规则/阈值不变。
`dog_resolution_xy:=0.0` 可重现本文件最初提交的 PGM 跟随行为。该变更不属于
split 的地图职责分离，详见 [DOG_XY_RESOLUTION.md](./DOG_XY_RESOLUTION.md)。
下文“旧模式参数不变”描述最初 split 提交；当前快照测试单独断言新 XY 设置，
移除这一新增键后继续核对原摘要，其他参数不放宽。

## 版本与范围

- 算法：从 `codex/rmuc-2027-field` / `1c04855` 创建
  `codex/split-static-planning`，仅在 `costmap_mode:=split` 启用新插件。
- 仿真：保持 `codex/rmuc-2027-field` / `e22c5ce`，无需修改、重建镜像或新增容器。
- 原有 combined、static_only、dynamic_only 的参数与启动节点保持不变。
- 不修改PGM、Point-LIO、DOG滤波/范围/衰减、ESDF、MINCO/MPC公式和权重、
  规划/控制频率、行为树或脱困；暂不实现语义分区与云台协议。

## 实际地图链路

以机器人命名空间 `/red_standard_robot1` 为例：

| 消费者 | 地图/数据 | 组成 |
|---|---|---|
| Smac Hybrid A* | 独立 `static_prior_costmap` | 原PGM + 标准只读StaticLayer + InflationLayer |
| MINCO两阶段 | `global_costmap/costmap_raw` | 与dynamic_only完全相同：边界底图 + FastLayer + InflationLayer |
| 局部检查/脱困 | `local_costmap` | 同dynamic_only，仍为5×5米滚动地图 |

`planner_server` 和 `GridBased` ID不变。新薄适配插件
`hero_static_prior_planner/StaticPriorPlanner` 原样调用Nav2的
`nav2_smac_planner::SmacPlannerHybrid`，只替换传给Smac的地图；不复制/重写A星。
它不把host提供的动态global_costmap传给Smac。原 `GridBased.*` 搜索参数全部保留。

独立先验图由插件管理configure/activate/deactivate/cleanup及执行线程：

- PGM发布者：`planning_prior/static_prior_map_server`。
- PGM话题：`/red_standard_robot1/planning_prior/map`。
- 先验costmap节点：`/red_standard_robot1/static_prior_costmap/static_prior_costmap`。
- 先验costmap显示：`/red_standard_robot1/static_prior_costmap/costmap`。
- 先验图不订阅DOG点云，也不用队内可被自由点云清除的UpdateStaticLayer。
- 先验图缺配置或未准备好时明确失败，不偷偷回退到动态全局图。

Nav2原global_costmap仍由planner_server维护；这不表示A星仍用它搜索。
该服务器原有的位姿/TF/地图就绪检查以及 `is_path_valid` 服务没有改写；后者仍检查
原动态global_costmap。当前仿真行为树没有用该服务来选择全局路线。

两张全场图都处于 `map`，使用同一选定场地YAML的原点、分辨率。
动态图沿用dynamic_only的整数米外框、0.35米内收边界、内部空闲和未知策略；
先验图加载原PGM的真实像素范围、墙体和未知值。DOG更新范围没有扩大。
区域暂未观察到、显示空闲，不代表雷达已经证明那里安全。

## 启动

先正常启动2027双雷达仿真，等待机器人落地。随后在算法终端执行：

```bash
source /opt/ros/humble/setup.bash
source "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))/install/setup.bash"
ros2 launch hero_sim_bringup navigation_point_lio_sim.launch.py \
  field:=rmuc_2027 \
  lidar_mode:=dual \
  costmap_mode:=split \
  relay_cmd_vel:=true \
  publish_debug_visualizations:=false \
  point_lio_rviz:=true \
  filter_high_columns:=true \
  high_column_min_z:=0.20
```

除 `costmap_mode:=split` 外，参数继续使用自己已验证的设置；这里的0.20不是新的调参。
无需第三个终端。真值入口和C1点云入口也通过同一个工厂支持split。
无运动地图检查可设置 `relay_cmd_vel:=false`，关闭RViz不会关闭先验地图服务器。
不要同时在相同命名空间运行旧算法和新算法。

RViz原有Map背景显示先验PGM，Global/Local Costmap显示真正的动态代价。
`plan`仍是A星输出；`FollowPath/reference_path`仍是MPC跟踪的MINCO参考轨迹。
若需看A星实际膨胀图，可添加Map显示 `static_prior_costmap/costmap`。

只读确认命令：

```bash
ros2 param get /red_standard_robot1/planner_server GridBased.plugin
ros2 param get /red_standard_robot1/smoother_server costmap_topic
ros2 param get /red_standard_robot1/global_costmap/global_costmap plugins
ros2 param get /red_standard_robot1/static_prior_costmap/static_prior_costmap plugins
```

依次应为新适配插件、`global_costmap/costmap_raw`、
`boundary_layer/fast_layer/inflation_layer`、`prior_static_layer/inflation_layer`。

## 不包含的保证

- 静态初始路线的中间点可以在动态图障碍内；MINCO允许移动它们，但不保证局部优化
  找到无碰撞路线，起终点仍固定，整条通道堵死也不能保证自动换通道。
- 没有增加路径锁定/缓存/滞回。动态点云不再直接影响A星地图，但起点变化或近似等价
  路线仍可能改变输出；不能把该方案写成“全局路径永不变化”。
- 原MINCO软避障、错误返回和多项式发布行为保持原样，本次没有顺带添加硬碰撞检查。
  因此返回成功不代表无碰撞，先验路线也不是平滑轨迹的硬边界。
- PGM里的误封洞口仍可能让A星无法选中这条通道，需要校正PGM，不能期待MINCO凭空
  生成A星没有提供的全局路线。
- 未实现持久封路的全局重选、云台到位互锁、真车串口；不将合成测试等同于实际过洞。

## 编译、测试与回退

使用现有安全脚本，避免在有括号/空格的源码路径直接手写colcon编译：

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
./build_hero2026_safe.sh --jobs 4
```

配置回归：`test/test_costmap_modes.py` 和 `test/test_split_mode.py`；
后者保存创建分支前 `1c04855` 的全部生成参数规范化摘要，验证旧模式未变，
并比较split与dynamic_only仅存在先验图/规划插件/显式平滑地图来源的差别。

隔离运行测试（不启动Gazebo、DOG、MPC、底盘，不发布任何cmd_vel）：

```bash
source /opt/ros/humble/setup.bash
source "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))/install/setup.bash"
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=196 ROS_LOG_DIR=/tmp/hero_split_test_logs \
  PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q \
  src/simulation/hero_sim_bringup/test/test_split_planner_runtime.py \
  -o cache_dir=/tmp/hero_split_pytest_cache
```

测试运行真正的planner_server/Smac插件、独立costmap和已配置的smoother_server。
检查先验墙迫使A星绕行；新增动态图障碍确实进入global_costmap，
MINCO订阅该图，而先验图与A星路径不受影响；另检查停用/清理/再启动生命周期。
测试记录会保留在pytest临时目录，不纳入源码提交。

实际验收待用户完成：同一条路线连续导航、多次目标、双向过洞、临时动态障碍、
确实无法绕行时的表现，并记录是否仍有两条全局路线来回切换。
确认前不合并旧分支、不创建“通过验收”里程碑标签。

2026-10-20已完成：安全脚本构建成功，43个安装前缀、119个ELF；断链、缺失动态库、
临时RPATH均为0。配置测试192项通过；独立ROS域已验证真实Smac/地图输入分离、
MINCO订阅和规划器重复生命周期。另覆盖正式启动使用的TF命名空间重映射。
上述测试不运行轨迹、不产生底盘速度指令，不替代用户的Gazebo完整导航验收。

仅对照三种旧模式：停止当前算法，将启动参数改回combined/static_only/dynamic_only；
不需要切分支或重编译。源码精确回退：先保存自己的修改，再切回
`codex/rmuc-2027-field`（基线 `1c04855`）并运行安全编译脚本，重新source。
Git切分支不会自动切换install二进制或已经启动的进程。
