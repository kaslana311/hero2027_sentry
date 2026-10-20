# DOG X/Y 网格恢复为 5 厘米

日期：2026-10-20，UTC+08:00。算法分支：`codex/split-static-planning`；
父提交：`815b666`。仿真仓库保持 `codex/rmuc-2027-field` / `e22c5ce`。

## 原因与改动边界

原 DOG 从选定 PGM 的 YAML 读取 X/Y 分辨率，Z 独立读取 `resolution_z`。
2027 精细 PGM 在 `a87d761`（2026-10-16 21:30:30 +08:00）改为 0.025 米时，
DOG X/Y 也间接变为 2.5 厘米；2026 PGM 则仍是 5 厘米。
上游本身也曾使用 X/Y=2.5 厘米，不能单凭数值断言这是全部故障原因。
小网格让稀疏墙面回波分散到不同柱，旧的高度跨度/空洞过滤可能更易拒绝它们。

本次按用户要求进行分辨率对照：

- 三个仿真导航入口、四种代价地图模式统一默认 `dog_resolution_xy:=0.05`。
- 只改变 DOG 占据网格的 X/Y；Z 仍为 0.03 米。
- 2027 PGM、YAML、Nav2 地图分辨率/边界/原点、Point-LIO、TF 均不改。
- 不改高度跨度/空洞/高柱过滤规则、阈值、概率、衰减、处理范围或空点云发布行为。
- 不改 Smac、MINCO、MPC、行为树、频率或 ESDF；不修改仿真模型/镜像。
- 真车入口不注入新参数；DOG 节点默认 `resolution_xy=0.0`，继续跟随原 PGM。

## 如何保持尺寸和坐标

新 `resolution_xy` 是 DOG 节点的只读启动参数。0 表示原行为；正数必须是
PGM 分辨率的整数倍，不做插值细化。非法值明确报错，不偷偷回退。

DOG 仅在内存中从地图左下角聚合像素块，并保留块内最暗像素，避免平均掉细墙。
原 PGM 不写回、不覆盖。1120×642、2.5 厘米的 2027 PGM 对应 DOG 的
560×321、5 厘米网格，场地仍为 28×16.05 米，原点不动。
非整除的图像末尾允许一个不完整单元，但点云裁剪仍采用原图精确物理边界，
不会将补齐部分扩展成可处理场地。禁止只把原 YAML 的 0.025 改成 0.05，
否则静态地图尺寸会翻倍。

## 编译与启动

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
./build_hero2026_safe.sh --jobs 4
```

停止旧算法，重新 source 后使用原命令即可，默认已经为 5 厘米。
例如当前 C2/split 对照（需配对 2027 仿真，而不是 2026 启动脚本）：

```bash
source /opt/ros/humble/setup.bash
source "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))/install/setup.bash"
ros2 launch hero_sim_bringup navigation_point_lio_sim.launch.py \
  field:=rmuc_2027 lidar_mode:=dual costmap_mode:=split \
  relay_cmd_vel:=true publish_debug_visualizations:=false point_lio_rviz:=true \
  filter_high_columns:=false high_column_min_z:=0.20 dog_resolution_xy:=0.05
```

保持其他参数相同，重启时改为 `dog_resolution_xy:=0.0` 即恢复旧的跟随 PGM
行为（2027 为 2.5 厘米、2026 为 5 厘米），无需切分支或重编译。
该参数不能运行中用 `ros2 param set` 修改，避免参数值和已分配网格不一致。

只读检查（显式设置 0.05 时应返回 0.05；0 则查看启动日志中的实际网格）：

```bash
ros2 param get /red_standard_robot1/dog_map_node resolution_xy
ros2 param get /red_standard_robot1/dog_map_node resolution_z
```

## 验证范围

配置测试：旧参数快照只允许多出 `resolution_xy`，保留原快照摘要；
四模式、三场地对照 0/0.05 时，其余生成参数必须相同。
C++ 测试使用生产网格函数，覆盖左下角对齐、奇数尺寸、精确物理范围、
薄墙/未知像素保留、旧路径不拷贝、实际2027地图尺寸和非法值。
隔离 ROS 测试向真实 DOG 节点输入相同稀疏墙面：2.5 厘米时被旧跨度规则
拒绝，5 厘米时可形成占据柱，同时保留低墙对照；不启动底盘或发布速度指令。

上述合成验证不等于实际场地验收。仍需用户在正确 2027 场地上比较同一墙面、
洞口、坡面和连续导航；5 厘米可能改善墙面保留，也可能将地面/洞顶合并进同一柱。
本次不宣称所有漏障碍或过洞问题已经解决，不合并稳定分支、不创建验收标签。

2026-10-20 已完成验证：

- `./build_hero2026_safe.sh --jobs 4` 编译/安装成功：43个前缀、119个ELF，
  断链、缺失动态库、临时RPATH均为0。
- `test_costmap_modes.py`、`test_split_mode.py`、`test_dog_xy_resolution.py`
  的无节点测试：219项通过，3项运行测试暂不选择；原参数摘要只排除新增XY键。
- 隔离ROS域197运行 `test_high_column_filter.py` 和 `test_dog_xy_resolution.py`：
  52项通过，含8项真实DOG节点测试；其中部分无节点网格测试与上组重复。
- 源码/安装目录全部地图资产逐文件一致，两个主启动文件与源码一致；
  PGM、Nav2原始配置、MINCO、MPC和Point-LIO无源码改动。
- 未启动/停止用户的仿真或导航，未发送速度指令；现有进程不会自动加载新代码。

复现测试（从算法仓库根目录，日志与缓存只放临时目录）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=197 ROS_LOG_DIR=/tmp/hero_dog_xy_test_logs \
  PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q \
  src/simulation/hero_sim_bringup/test/test_costmap_modes.py \
  src/simulation/hero_sim_bringup/test/test_split_mode.py \
  src/simulation/hero_sim_bringup/test/test_high_column_filter.py \
  src/simulation/hero_sim_bringup/test/test_dog_xy_resolution.py \
  --maxfail=1 --tb=short -o cache_dir=/tmp/hero_dog_xy_pytest_cache
```
