# 仿真雷达内移与外参同步（2026-10-25）

用户要求雷达不要突出车侧，避免外壳撞上障碍。当前算法分支
`codex/split-static-planning`，修改前 `45da0f3`；配对仿真分支
`codex/rmuc-2027-field`，修改前 `03489a4`。作为两端当前分支的新提交，
不回退历史，不改真车标定、PGM、出生点、map→odom 或导航参数。

## 新安装坐标（相对 chassis，米/弧度）

| link | X | Y | Z | roll / pitch / yaw |
| --- | ---: | ---: | ---: | --- |
| left_mid360 | -0.031029689 | 0.157956386 | 0.103822859 | 0.004491681 / 0.593489259 / 1.574538158 |
| right_mid360 | 0 | -0.167 | 0.121 | 0 / 0.61086 / -1.57079 |

左右各向内移动 5 cm，其他安装分量、传感器频率/噪声/扫描范围不改。
仿真移动安装 joint，外观、真实碰撞、扫描光心及 IMU 随同一 link 移动。
雷达完整倾斜碰撞盒的新 Y 外沿是左 +0.199380083、右 -0.208435862 m，
相对固定轮组侧边分别内缩约 17.2 mm / 8.2 mm；没有删碰撞、缩放模型或改车高。

## 四处配对变换

- `sim_ground_truth_bridge.cpp` 更新左右 mount；仍用采样时刻的完整三维
  真值把点云变到 map，不影响原来的时间配准、地图原点和速度转发。
- `sim_point_lio_input_bridge.cpp` 更新左右 mount；由新的右 IMU 位置建立
  轴向与 chassis 平行的虚拟 IMU：`T_chassis_imu = translation(0,-0.167,0.121)`。
- `sim_point_lio_evaluation.cpp` 使用同一个 IMU 偏移作一次初始对齐与底盘换算。
- `sim_point_lio_navigation_bridge.cpp` 使用同一个 IMU 偏移输出底盘位姿/速度。

对原始雷达点，融合输入仍为：

```text
p_imu = inverse(T_chassis_imu) * T_chassis_lidar * T_lidar_ray * p_ray
T_lidar_ray = translation(0,0,0.03)
```

IMU 加速度/角速度只按右雷达原有朝向转到 chassis 轴向，实际测量点也已在
Gazebo 随 link 移动，不模拟一个位于旧位置的 IMU。
`point_lio_snapshot.yaml` 保留零平移/单位旋转的外参，因为输入点云已经处于
虚拟 IMU 坐标系；再加一次新的安装平移反而会重复补偿。
定位输出仍为：

```text
T_map_chassis = T_map_lio * T_lio_imu * inverse(T_chassis_imu)
```

少改 evaluation/navigation 的任意一处，转弯时就可能把雷达的杆臂运动当成
底盘移动。保留旧值 `-0.217` 与新模型混用是不允许的。
ROS 雷达 TF 从模型的新固定 joint 生成，不另加一份竞争的静态 TF。
原 `lidar_merge` 及真车 Point-LIO YAML 不在此仿真入口的融合链路中，保持不变。

## 验证与生效

新源码契约检查覆盖四处外参一致、ray 偏移与 snapshot 零外参保留。
合成传感器检查用新默认安装值将左右雷达各自看到的同一墙点融合，要求两点
在虚拟 IMU 中重合，同时检查重力/角速度转轴。既有定位检查覆盖旋转杆臂、
倾斜底盘 TF、初始对齐与点云输出，不用真值持续纠正估计来掩盖错误。
算法按用户指定以 `./build_hero2026_safe.sh --jobs 12` 编译通过：39 个工作区包，
43 个安装前缀、120 个 ELF，断链/缺库/临时 RPATH 均为 0。
56 项隔离 ROS 回归全部通过（43.49 s），含新默认外参下的同墙点融合和 IMU
转轴；仿真仓库另有 25 项模型/驱动检查通过（2.79 s）。
JUnit 记录为本轮可视化目录的 `lidar_inset_20261025/runtime_tests.xml`，
没有为此向实际机器人发送指令。合成链路检查不等于实车/仿真过洞验收。

必须先停止旧算法/仿真，用原 `start_sim_rmuc_2027_dual_lidar.sh` 重新构建
派生镜像并启动；等车落稳后重新运行原 C2 算法命令。没有新增启动参数。
当前工具无法免密 sudo 构建仿真镜像，不能把文件修改称为镜像已更新。
若使用 2026，也先用其原启动脚本重建，不能复用旧传感器布局的缓存镜像。
两个 `normal/tunnel` 模型布局相同；`single` 官方传感器 TF 路径不受此修改影响。

雷达内移不可避免会稍改视野/自遮挡；复测贴墙及过洞前，不宣称观测密度或
定位精度完全不变。滤波、规划/平滑、控制增益和 PGM 字节均未因本轮改变。
