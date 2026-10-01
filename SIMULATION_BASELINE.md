# 仿真真值定位过洞基线

日期：2026-10-01（+08:00）。用户确认当前配置顺畅过洞，原导航自动失败、地形卡滞
和配准偏差在本次运行中不再出现。本记录不声称已经获得三轮统计或真车级精度。

后续实验状态：`codex/point-lio-sim` 已增加独立 Point-LIO 旁路并通过宿主机合成测试，
未接管导航、未完成 Gazebo 运动验收。见
[`Point-LIO 说明`](src/simulation/hero_sim_bringup/POINT_LIO.md)。下文保留基线归档当时的
版本与验证记录；同名标签、稳定分支和原真值启动命令不变。

## 配对版本

| 仓库 | 本次整理前最后实现提交 | 稳定分支 |
|---|---|---|
| `/home/apocalypse_linux_00/build_sim_docker` | `9d72770f7d4bc8efd7ef4cae9981a31e0b691422` | `main` |
| 当前算法仓库 | `75e858e781eced8456da0883832e859b4b7de94f` | `sim-perception-integration` |

两仓库用同名 annotated tag **`sim/truth-tunnel-baseline-v1`** 指向本次纯文档提交，
运行代码与上表相同。两边从该标签创建并切到 **`codex/point-lio-sim`**；初始行为不变。
标签在不同仓库对应不同 SHA，这是正常的配对版本管理，不是两个仓库 SHA 应相同。
本次不修改算法、频率、模型、启动入口或镜像，不合并到真车 `master`，不推送远程。

## 已确认运行配置

- 仿真：2026 清理版场地、双 MID360、默认 `SIM_ROBOT_PROFILE=tunnel`。
- 算法：`field:=rmuc_2026 lidar_mode:=dual costmap_mode:=combined`。
- 定位：Gazebo 二维真值导航；独立三维真值按采样时间注册两路点云到 `map`。
- local/global costmap 仍为 static + fast + inflation，没有实现近远场分层。
- 当前绕过 Point-LIO 和重定位，不启动真实雷达、串口或其他真车驱动。

启动仿真（终端一）：

```bash
cd /home/apocalypse_linux_00/build_sim_docker
SIM_ROBOT_PROFILE=tunnel ./scripts/start_sim_rmuc_2026_dual_lidar.sh
```

启动算法（干净终端二，不混入 `pb_ws_nav` 的环境）：

```bash
source /opt/ros/humble/setup.bash
source "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))/install/setup.bash"
ros2 launch hero_sim_bringup perception_truth_sim.launch.py \
  field:=rmuc_2026 lidar_mode:=dual costmap_mode:=combined
```

本次仅准备分支，启动命令不变，Point-LIO 尚未接入。

## 本次检查与未测项

仿真 14 项单元测试通过，STL 清理输出哈希核对一致；算法适配桥 8 项黑盒点云测试
及 3 项行为树契约测试通过。安全脚本 `--verify-only` 检查 42 个安装前缀、114 个
ELF，断链、缺少共享库和临时 RPATH 均为 0。没有重新编译、启动仿真或发送运动指令。

用户未提供确切轮数、逐轮 action 状态、固定目标坐标或定量定位误差；本次也未
取得 Docker image ID。保留这些边界，不把未测项写成通过。详细记录位于仿真仓库
`experiments/2026-10-01_truth-tunnel-baseline.md`。

## 成对切回与产物恢复

先停止自己的算法/仿真进程并提交实验修改，确认两边干净，再切回：

```bash
git -C /home/apocalypse_linux_00/build_sim_docker switch main
git -C "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))" switch sim-perception-integration
```

若稳定分支日后继续前进，精确基线以两边的同名标签为准，可在各仓库执行
`git switch --detach sim/truth-tunnel-baseline-v1` 运行对照。继续开发时两边分别执行
`git switch codex/point-lio-sim`，不要在 detached HEAD 上开发。

Git 只版本管理源码，不切换当前二进制、环境、进程或镜像。以后改变算法源码后，
切回需用已有安全脚本编译，继续使用原 build/install：

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
./build_hero2026_safe.sh --jobs 4
./build_hero2026_safe.sh --verify-only
```

另开干净终端重新 source。若改变了仿真 overlay/插件，切回后用原 2026 脚本重建
匹配镜像，再启动新容器。仅本次纯文档/初始分支切换不需要重编译。
镜像 `:local` 可被覆盖，未作离线镜像归档；下一阶段若需要改镜像，先使用独立
实验名称保留基线镜像。当前标签是本地版本记录，不是异地备份。

日期与变更目的可查：

```bash
git show --no-patch sim/truth-tunnel-baseline-v1
git log -n 8 --date=iso-strict --format='%h %ad %cd %s'
```

## Point-LIO 下一阶段

1. 核对队内双雷达合并、Point-LIO 点类型/逐点时间、IMU、外参、frame 与仿真时间契约。
2. 输入未配准扫描及 IMU，先旁路输出用于对照；不把真值 `cloud_registered` 喂给前端。
3. 通过独立实验入口成对替换导航位姿和 DOG map 点云输入，保持每条 TF 一个发布者。
4. 维持当前感知/规划/控制与地图参数，按真车方案关闭重定位；真值仅用于误差评估，
   不偷偷融合进实验定位或当隐藏回退。
5. 用相同路线记录成功率、漂移、坡面误障碍和延迟，再决定是否合并回稳定仿真分支。

更详细的分阶段验收计划在仿真仓库 `docs/POINT_LIO_NEXT_STAGE.md`。
