# C2：Point-LIO 位姿和配准云共同接入导航

2026-10-08，两仓库 `codex/point-lio-navigation`，从配对标签
`sim/point-lio-cloud-baseline-v1` 新建。旧 C1/真值/旁路入口保留；原标签不移动。
C2 是新实验，尚未由用户实跑验收，不宣称过洞成功或真车定位完全复现。

实现提交 `1bbf80c53122f7d13918dc8abdd76848a4a2c97b`，
2026-10-08T16:54:58+08:00，`feat(sim): add isolated Point-LIO navigation takeover`。
算法分支起点 `1530e45eb1a1449598b56a9abc6bf522d32a261a`；仿真起点
`32cd4f4cec776200983628d551be0270d88bace5`。两边同名旧标签不移动。

安全脚本 `./build_hero2026_safe.sh --jobs 4` 完成38包构建、42安装前缀/118 ELF；
断链、缺库、临时RPATH均0。13项新增测试单独通过；全55项算法适配回归
42.39秒通过，包含旧42项和实际点云桥/生产DOG map接收测试。仿真14项1.32秒
通过。已安装新入口 `--show-args` 解析成功，关键launch与源码相同。
隔离域183、随机命名空间，仅合成测试指令，不运行Gazebo/Nav2，不发布实际
机器人的运动指令。首轮编译发现Humble时钟now接口的const限制，已修正后重编。
这些是接口/回归证据，不替代完整实跑，更不等于真实逐点扫描去畸变验收。

## 启动

仅运行以下一套算法入口。先停止旧算法、不要重复启动独立 Point-LIO 旁路。
仿真仍是2026双雷达/tunnel，两个终端即可，没有新镜像。

```bash
# 终端一，已有相同仿真可跳过
cd /home/apocalypse_linux_00/build_sim_docker
SIM_ROBOT_PROFILE=tunnel ./scripts/start_sim_rmuc_2026_dual_lidar.sh
```

```bash
# 终端二，干净环境
source /opt/ros/humble/setup.bash
source "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))/install/setup.bash"
ros2 launch hero_sim_bringup navigation_point_lio_sim.launch.py \
  field:=rmuc_2026 lidar_mode:=dual costmap_mode:=combined \
  relay_cmd_vel:=true publish_debug_visualizations:=false point_lio_rviz:=true
```

先静止初始化约10秒，再短距离平地运动、往返、小坡和过洞。主导航位姿现在是
LIO；对照窗口绿色真值、蓝色 LIO、黄色 map/odom 原点。

## 接口契约

- `pointlio/odometry`（pointlio_odom→pointlio_imu）经过新增导航适配，输出
  `odometry/filtered`（odom→base_link），只此一个导航里程计发布者。
- `T_map_chassis = T_map_lio × T_lio_imu × inverse(T_chassis_imu)`；固定安装
  偏移 `(0, -0.217, 0.121) m`。定位与点云使用同一个持久化初始对齐。
- `map→odom` 恒等；`odom→base_footprint` XY/yaw；`base_footprint→base_link`
  动态保留高度、roll/pitch。采样时间戳不被现在时间替换。
- 真值桥改发 `odometry/truth_planar`，TF 隔离到 `truth/tf[_static]`，关闭
  其速度转发。`odometry/truth_3d`、真值云仅用于对照和首次对齐。
- Point-LIO 快照没有输出速度，新增适配用连续底盘位姿的采样时间差计算速度，
  将速度表达在 base_link，先扣除 IMU 安装偏移，避免转向时出现假平移。
  这仍是差分区间速度，存在噪声，未额外宣称协方差或电机反馈质量。
- 新桥不订阅真值，也不在漂移/断流时回退真值。首次起点仍由真值初始化，
  不属于无需起点的自主重定位。真实逐点扫描去畸变仍未复现。
- DOG map、Nav2/MINCO/MPC/行为树/规划频率等均不改；C1/C2生成的导航 YAML
  全文相同。仅公共 launch 工厂增加默认关闭的诊断隔离选项。

适配状态：`pointlio/navigation/status`。`ready` 需有效冻结对齐、连续两帧位姿
（间隔≤0.5秒）、非空配准云，采样时间和墙钟年龄≤1秒。控制指令0.5秒未更新
则停止活动指令；断流/暂停时停止，恢复不重放旧命令。时间重置或对齐改变会
锁定 `restart_required`，必须重启整个算法入口。健康状态不是 DOG map 接收
或导航成功的证明；原点云输出仍是 Reliable，DOG map分类输出另行确认。
`relay_cmd_vel:=false` 只禁止这个桥转发，不是硬件急停。

## 对照与版本

停止 C2 后，把上述入口换成 `perception_point_lio_sim.launch.py`，参数不变，
回到 C1（LIO 云 + 真值导航）。换成 `perception_truth_sim.launch.py` 并移除
`point_lio_rviz` 参数，回到原纯真值。都在同一install，不需要重建容器。
原精确版本保留在 `codex/point-lio-sim` 和配对标签；Git不恢复二进制。
构建仍用工作区根目录 `./build_hero2026_safe.sh --jobs 4`。

仿真仓库详细流程和配对记录：`docs/POINT_LIO_NAVIGATION.md`、
`experiments/2026-10-08_point-lio-navigation.md`。版本标签是固定提交名，
不是提交说明，也不是会随新提交前进的分支。
