# 统一定位、点云和后端的主 TF 链路

2026-10-31；算法分支 `codex/point-lio-backend`。本次不改 Point-LIO 状态估计、
IMU/雷达外参、MPC、MINCO 热启动、代价地图模式、PGM 或 Gazebo 模型。
适用入口：`navigation_point_lio_sim.launch.py`，以及包含它的后端旁路评估入口。
旧的纯真值、独立 Point-LIO shadow 和 C1 感知实验保留旧契约。

## 1. 原来的分歧到底在哪里

原 C2 中，Point-LIO 在 `pointlio_odom` 下输出局部里程计和已配准云。
导航桥和点云桥分别缓存相同的初始对齐 A，直接算 `A × 位姿`、`A × 点`。
前者虽然把消息标成 `odom`，数值已经是 map 下的；后者把点标成 `map`。
主 TF 的 `map → odom` 却始终是恒等变换。

因此调整主 TF 不会改变已烘焙到 map 的点坐标；另一个 RViz 还可能显示独立冻结
对齐的前端结果。这是适配层设计造成的两套全球定位口径，不是 TF2 忽略中间坐标系。
当点云确实处在雷达/body/odom 系，并经主 TF 转到 map 时，用户关于它应随
`map → odom` 一起移动的理解是正确的。

## 2. 现在谁负责哪条变换

C2 的导航和配准点云共同读取 `/<robot>/tf`、`/<robot>/tf_static`：

```text
map                                      静态 PGM 固定在这里
 └─ odom                                 唯一全局修正：map → odom
     └─ base_footprint                    导航桥：初始地面基准下的投影
         ├─ base_link                     导航桥：剩余六维变换
         │   └─ pointlio_imu              导航桥：固定虚拟传感器安装外参
         └─ chassis → 模型各关节/雷达 link  仿真原有 robot_state_publisher 子树，未改
```

- 默认 `sim_point_lio_navigation_bridge` 独占发布 `map → odom`，动态 TF 为 20 Hz。
  它只用一次初始化种子，之后不再用真值修正；三个可调偏移用于人为注入定位偏差。
- 同一导航桥发布 `odom → base_footprint → base_link`，保持 Point-LIO 的采样时间。
- `base_link → pointlio_imu` 为固定 `(0, -0.167, 0.121) m`、单位旋转。
  这是融合输入所用的虚拟右侧 IMU 坐标，不应与倾斜安装的物理雷达壳体 link 混淆。
- Point-LIO 原生输出的世界坐标系在 C2 改名为 `odom`，它自身的 TF 广播关闭，
  避免再发布另一条竞争的 `odom → pointlio_imu`。数值估计完全不变。
- C2 的估计器、评估器和两个 RViz 都使用上述主 TF；评估器不再发另一份定位 TF。
- 真值仍在 `/<robot>/truth/tf[_static]`，只作对照和一次初始化，不参与实时导航。
  后端 shadow 的 `/<robot>/z_backend/tf[_static]` 也仍是隔离诊断；此次没有自动让
  尚未修好的定位后端接管导航。

坐标名字相同而 TF 话题不同，不会自动组成同一棵树。上面的“同一条主链路”特指
共同读取 `/<robot>/tf[_static]`，不是只把 frame_id 改成相同字符串。

## 3. 计算关系和时间戳

设原生 Point-LIO 位姿为 `L = T_odom_imu`，固定安装变换为 `E = T_body_imu`，
初始地图对齐为 `A = T_map_odom`：

```text
odometry/filtered.pose = L × inverse(E)                 # 仍在 odom
主 TF 下机器人位姿     = T_map_odom(t) × L × inverse(E)  # 在 map
原生配准点云           = p_odom                          # frame_id=odom
DOG 输入点云           = T_map_odom(t_scan) × p_odom     # frame_id=map
```

默认修正 `T_map_odom=A` 时，机器人和点云在 map 中的位置与修改前数学等价。
PGM 原点、出生点、传感器高度和滤波高度没有改变。现在 `map` 与原生 `odom` 的
原点不必重合：原生里程计的零点可在初始化传感器处，而 PGM 仍沿用原地图零点。

`base_footprint` 的分解使用初始化地面基准，以保留原模型的地面锚点：
`F_odom = inverse(A) × project_to_initial_ground(A × L × inverse(E))`，
`T_footprint_body = inverse(F_odom) × L × inverse(E)`。两段相乘仍是原生
`L × inverse(E)`，不是把 A 再乘入本地位姿；后续 map/odom 偏移不进入该分解。
交给外部后端发布时，不依赖真值种子，地面基准采用 odom 自身水平面；完整
`base_link`/传感器位姿仍与原生估计严格一致。

`pointlio/cloud_registered_map` 这个兼容话题仍存在，但它不再是一条独立定位旁路：
点云桥每帧查询主 TF 的采样时刻变换，没有独立冻结的 A。保留该输出是为了让 DOG
继续得到 map 系点云，避免把已解决的“运动后用最新倾角变换旧 body 云”问题带回来。
缺少该时刻 TF 时沿用有限队列和等待上限，不取最新 TF、不回退真值、不伪造空点云。
原始字段、强度、法向、行填充和采样时间保持原契约。

全局修正不参与本地速度差分。因此人为平移 map/odom，或后端修正 map/odom，
不会被当成一次底盘瞬间加速。MPC 的原有速度解耦与 MINCO 热启动实现未修改。

## 4. 如何测试 X/Y 各偏移 50 厘米

原仿真、算法启动命令不变，重新 source 安装环境并重启整个算法入口即可。
可以先取消导航、停车，再注入偏差；不要为了测试再起一个竞争的 TF 发布器。

```bash
# X 正方向偏移 0.5 m（绝对偏移设置，不是每执行一次再累加 0.5）
ros2 param set /red_standard_robot1/sim_point_lio_navigation_bridge map_odom_offset_x 0.5

# 恢复 X，改成只测试 Y 正方向偏移 0.5 m
ros2 param set /red_standard_robot1/sim_point_lio_navigation_bridge map_odom_offset_x 0.0
ros2 param set /red_standard_robot1/sim_point_lio_navigation_bridge map_odom_offset_y 0.5

# 恢复默认
ros2 param set /red_standard_robot1/sim_point_lio_navigation_bridge map_odom_offset_y 0.0
ros2 param set /red_standard_robot1/sim_point_lio_navigation_bridge map_odom_offset_yaw 0.0
```

`map_odom_offset_yaw` 单位为弧度。组合关系为 `T_user_offset × A`：XY 沿 map 轴，
旋转围绕 map 原点。位置、配准云、新感知障碍一起改变；PGM 不动；真值标记不动。
对照窗口蓝色 Point-LIO 标记现在也使用主 TF，误差指标包含人为注入的全局误差。
历史 Path、已经发出的 map 系云以及 DOG 的占据记忆不会被 TF 自动追溯重写，
可能短暂出现旧痕迹，需等待后续帧覆盖/衰减。这不是又出现一条定位旁路。
参数偏移重启恢复为启动值，启动时也可传 `map_odom_offset_x:=0.5` 等。

## 5. 后端接入点

未来让可靠的定位后端接管时，完整重启 C2 并传：

```text
publish_map_to_odom:=false
```

此时导航桥不订阅仿真初始化种子，不发布 `map → odom`，但继续提供本地里程计、
刚性传感器 TF 和原有速度保护。外部后端必须独占在主 `/<robot>/tf` 上发布
`map → odom`，且覆盖点云采样时刻；不能只发在后端自己的私有 TF 话题。
该发布权不能在运行时动态切换，以免留下重复 TF 发布者。

后端输入的 `common.odom_frame` 已同步为 `odom`；原始位姿和云数值不变。
如果后端输出的是全局 IMU 位姿，可计算：
`T_map_odom = T_map_imu_backend × inverse(T_odom_imu_frontend)`，必须使用同一时刻、
同一实体的位姿。不要把 IMU 位姿与底盘位姿混乘，也不要把修正再乘进本地里程计。
现有 Z-Localization shadow 入口仍不具备导航接管权限，不能仅关闭种子发布器
就认为 shadow 已替代它；误匹配和崩溃问题仍需另行解决。

## 6. 与实车相同和仍然不同的地方

相同的是坐标契约：里程计局部连续，全球修正在 map/odom，传感器刚性安装，
点云和机器人通过同一 TF 投到地图。实车也可把点云发布在 body 或 odom 中，
不必为了这个结构修改前端估计器。

仍不同的是仿真传感器、扫描采样方式、噪声/碰撞和一次真值初始化种子。
实车的种子应来自已知起点/重定位或后端，不能依赖 Gazebo 真值。
所以这次统一不等于仿真传感器物理特性已经完全复现真车，也不是修复后端漂移。
旧 rosbag 的 `pointlio_odom` frame 契约与新 C2 的 `odom` 不同，回放时需使用
对应旧版本或做显式且一致的边界转换，不能只改一个 header 而忽略 TF。

## 7. 验证范围

新增隔离 ROS 测试运行实际导航桥、点云桥、评估器和 TF2：X/Y 分别 0.5 m、
90° 全局修正、采样时刻插值、缺 TF 不回退、外部后端独占发布、对照窗口一致性、
安装外参和旧 shadow 契约。旧点云、导航保护、评估器和启动配置测试继续保留。
它们不是 Gazebo 整车实跑验收，不向正在运行的机器人发送目标或速度。
安全安装使用工作区根目录 `./build_hero2026_safe.sh --jobs 12`。

### 本轮实际结果

- 安全脚本成功：42 个工作区包步骤、47 个安装前缀、143 个 ELF；断链、缺库、
  临时 RPATH 均为 0。三个生产适配源码与实际编译的 staging 副本逐字节一致；
  三个安装后的 launch 与仓库源码一致，正式入口 `--show-args` 通过。
- 配置/旧入口兼容性检查：209 项通过（包括模式、场地、命名空间和后端输入）。
- 使用正式 install 下的生产程序，在隔离 ROS 域 184 运行：48 项通过，36.94 秒。
  包括新增 10 项主 TF 测试，以及导航桥、C1 点云桥和评估器旧契约回归。
  旧点云组还实际运行生产 DOG map，确认可靠接收及非空占据输出。
- 第一轮主 TF 测试中，测试监听器在初始化结束后才加入，却查询了加入前的动态
  TF，按采样时刻要求正确等待/丢弃而非输出。已只修正测试时序为监听器就绪后的
  新样本；没有因此放宽生产代码的 TF 时刻、队列或超时要求。
- 前端估计器源码、MPC/MINCO、DOG、PGM、代价参数与仿真仓库均无修改。
  这些结果不等于实际 Gazebo 过洞或真车验收，也不代表 Z-Localization 已修好。

构建日志、最终测试日志/XML、首轮测试诊断保存在：
`/home/apocalypse_linux_00/.codex/visualizations/2026/09/24/01a0d39c-2651-74c3-88fa-9206c221d0f1/unified_tf_20261031_YB05Jx/`。
