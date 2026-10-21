# DOG：暂时停用高处薄层过滤

日期：2026-10-21（UTC+08:00）。算法分支 `codex/split-static-planning`，
父提交 `2ab5335`。按用户要求作一次新提交，不切分支或回退历史。

## 改动与范围

只将 `src/planning/dog_map/src/occ_map.cpp` 的 `OccMap::PubRes` 中以下条件
及其 `continue` 注释，保留原代码方便后续恢复：

```cpp
if (dzf < 0.12 &&
    (min_z_occ_index - fake_layer_z) * resolution_z > 0.2)
    continue;
```

`dzf` 是同一XY柱最高与最低占据层索引差乘以Z分辨率，不是物体真实厚度；
第二项使用旧的固定高度参考，不会随高地或坡面抬升，可能排除高地上的真实障碍。
这不是 `filter_high_columns` / `high_column_min_z` 控制的可选整列过滤。

地面跨度判断、最大空洞判断、可选整列过滤、点云裁剪、分辨率、射线、占据概率、
衰减、非空才发布机制全部不变。不增加新参数/节点/发布机制；不改地图/PGM、
TF、Point-LIO、A星、起点恢复、MINCO/MPC或仿真模型/镜像。

该代码是共用DOG实现，因此四种代价地图模式及任何复用该DOG的入口均受影响，
不是只针对split或高地区域。只改变通过分类后的DOG输出；不直接清除内部体素。

## 不应误判为已解决的漏检

当前默认 `resolution_z=0.03`、`heighter_than_ground_threshold=0.091`：
跨度按3厘米量化，低于12厘米的常见跨度（0/3/6/9厘米）已被前面的
9.1厘米地面跨度判断排除，12厘米则不满足旧规则的严格小于。因此该默认组合下，
停用这一条通常不会独自恢复原来漏掉的障碍；不能据此宣称高地感知已修复。

若启动仍传 `filter_high_columns:=true`，整列最低高度超过 `high_column_min_z`
的过滤继续生效，也可能拒绝高处墙面。用户若要关闭该可选规则，需另行明确传
`filter_high_columns:=false`；本轮不擅自改变其参数或任何其他过滤。

## 编译、验证与生效

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
./build_hero2026_safe.sh --jobs 4
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=197 ROS_LOG_DIR=/tmp/hero_dog_thin_layer_logs \
  PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q \
  src/simulation/hero_sim_bringup/test/test_high_column_filter.py \
  src/simulation/hero_sim_bringup/test/test_dog_xy_resolution.py \
  src/simulation/hero_sim_bringup/test/test_costmap_modes.py \
  src/simulation/hero_sim_bringup/test/test_split_mode.py \
  src/simulation/hero_sim_bringup/test/test_recovery_contract.py \
  -o cache_dir=/tmp/hero_dog_thin_layer_pytest_cache --tb=short --maxfail=1
```

新增真实DOG合成测试：在仅测试用5厘米地面门槛下，9厘米跨度的高处柱不再被旧
薄层规则吞掉；恢复生产9.1厘米门槛仍被地面跨度规则排除；可选整列过滤开启时
仍正常拒绝高柱。低墙对照始终保留，不用放宽生产阈值来得到测试通过。
其余原高柱、XY网格、四模式参数摘要和恢复契约继续检查。

安全脚本编译/安装完成，43个前缀和119个ELF通过审计，断链、缺少共享库、
临时RPATH均为0。上述限定测试共250项通过（34.94秒），其中包含11项真实
DOG节点合成测试（本轮新增3项）；另外三种模式的既有参数摘要断言仍通过。
测试在隔离ROS域运行，不发送底盘命令；不是实际高地行驶或整仓库全量验收。
停止旧算法、重新source并沿用原启动命令即可；没有新启动参数，不需重建镜像。
现有运行中的DOG不会自动加载修改。实际高地/坡面/过洞效果仍需用户验收，
不合并main、不创建成功标签。仿真仓库保持 `codex/rmuc-2027-field` / `d9efd5e`。
