# Hero2026 哨兵导航仓库编译复盘与正确构建方法

## 1. 文档目的

本文记录本工作区第一次完整构建时遇到的问题、问题的真实原因、已经实施的修复、推荐的构建流程，以及以后修改代码时应该如何增量编译。

工作区：

```text
/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))
```

目标环境：

- Ubuntu 22.04
- ROS 2 Humble
- CMake + Ninja
- 隔离安装布局：`install/<包名>`
- 默认最多 8 个包内编译任务，包与包之间串行
- 使用 Livox 雷达，明确跳过 `odin_ros_driver`

本文只讨论编译系统和依赖，不评价导航算法本身。

## 2. 最终结果

本次最终状态：

- 源码树识别到 42 个构建目标。
- 成功安装 41 个前缀。
- 唯一跳过：`odin_ros_driver`。
- ROS 2 能发现 37 个 ament 功能包。
- 检查 113 个 ELF 可执行文件或动态库。
- 缺失动态库：0。
- 断开的软链接：0。
- 指向 `/tmp/hero2026...` 的临时 RPATH：0。
- 安装目录中的临时构建路径引用：0。

最终使用方式：

```bash
cd '/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))'
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 3. 为什么这个仓库第一次编译特别困难

这不是单一报错，而是多个构建体系和仓库维护问题叠加的结果。

### 3.1 仓库不是“下载后直接全量 colcon build”的状态

仓库中同时存在：

1. ROS 2 ament 包，例如 `hero_serial`、`fast_lio`、`bt`。
2. 普通 CMake 包，例如 `acados`、`small_gicp`、`align_cloud`。
3. 内嵌第三方库，例如 Bstyle 下的 NLopt。
4. 厂商 SDK，例如 Livox SDK2。
5. 缺失闭源库的硬件驱动，例如 Odin 驱动。

Colcon 可以发现其中多数项目，但并不代表它能自动推断所有第三方库的特殊构建参数和安装位置。

### 3.2 `package.xml` 与 `CMakeLists.txt` 的依赖不一致

CMake 中的 `find_package(X REQUIRED)` 只告诉 CMake 需要 X；colcon 构建依赖图时主要读取 `package.xml`。如果 CMake 写了依赖而 `package.xml` 没写，colcon 可能先构建使用者，再构建被依赖包。

本次修复的依赖清单：

- `src/serial/hero_serial/package.xml`
  - 增加 `interfaces`、`serial_interfaces`、`std_msgs`、`nav_msgs`、`action_msgs`、`tf2_geometry_msgs`。
- `src/perception/tracker_layer/package.xml`
  - 增加 `interfaces`。
- `src/planning/pb_minco_smoother/package.xml`
  - 将错误的 `rmoss_interfaces` 改为实际使用的 `interfaces`。
- `src/perception/nav_cv_bridge/package.xml`
  - 增加 `serial_interfaces`。
- `src/localization/relocalization/package.xml`
  - 补齐 ROS、PCL、TBB、OpenMP、Qt、Eigen 和 `small_gicp` 依赖。

这些修改不改变算法，只修正构建图。

### 3.3 工作区路径包含空格和括号

当前目录名包含：

```text
hero2026_-sentry (to_compile_not_develop))
```

现代 CMake 通常能处理空格和括号，但仓库中的部分老脚本会把路径拼进 `/bin/sh` 命令而没有完整引用。NLopt 生成接口时曾出现：

```text
/bin/sh: Syntax error: "(" unexpected
```

正确做法不是修改用户名，也不是随意删代码，而是将源码复制到没有空格和括号的 `/tmp` staging 路径构建，再安装回真实工作区。

新脚本使用：

```text
/tmp/hero2026_sentry_build_<uid>/source
```

### 3.4 Bstyle 与 NLopt 的安装方式写死

旧的 Bstyle CMake：

- 覆盖整个 `CMAKE_PREFIX_PATH`，可能抹掉 ROS 和其他工作区前缀。
- 把 NLopt 的源码内安装目录写进永久 RPATH。
- 假定 NLopt 已经由其他人提前构建。

旧的 NLopt CMake 又无条件覆盖 `CMAKE_INSTALL_PREFIX`。这会导致调用者传入的安装位置无效。

已实施的正确修复：

- Bstyle 使用可配置的 `NLOPT_ROOT`。
- 使用 `list(PREPEND CMAKE_PREFIX_PATH ...)`，不破坏已有搜索路径。
- 将 `libnlopt.so*` 一起安装到 Bstyle 的 `lib` 目录。
- Bstyle 和 NLopt 使用 `$ORIGIN` RPATH，不依赖临时绝对路径。
- NLopt 尊重调用者的 `CMAKE_INSTALL_PREFIX`。

### 3.5 `align_cloud` 无视 colcon 安装前缀

旧配置直接执行：

```cmake
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)
```

这会把可执行文件装回源码目录，破坏 colcon 的 `install/<包名>` 结构。现已删除这条覆盖，并将程序安装到：

```text
install/align_cloud/bin/align_cloud_node
```

### 3.6 Acados 不是普通 ROS 包

`hero_mpc_controller` 需要：

- `libacados.so`
- `libhpipm.so`
- `libblasfeo.so`
- Acados、HPIPM 和 BLASFEO 头文件

控制器原始默认路径依赖源码树相对位置。在 staging 路径构建时，这个默认推导会失效，因此必须明确传入：

```text
-DACADOS_INSTALL_DIR=<工作区>/install/acados
```

Acados 必须先作为普通 CMake 工程构建，再构建控制器。

### 3.7 Livox 驱动需要指定 ROS 版本

`livox_ros_driver2` 同时包含 ROS 1 和 ROS 2 构建逻辑。正确构建 ROS 2 Humble 版本时需要：

```text
-DROS_EDITION=ROS2
-DHUMBLE_ROS=humble
```

并且必须先安装 Livox SDK2。新脚本将 SDK 和驱动都安装在工作区内，不再执行 `sudo make install` 污染 `/usr/local`。

### 3.8 Odin 驱动缺少厂商闭源库

`odin_ros_driver` 需要：

```text
lib/liblydHostApi_amd.a
```

该文件未随仓库提供。它不是通过修改 CMake 就能生成的开源依赖。当前机器人使用 Livox，因此正确处理方式是明确跳过 Odin，而不是伪造空库或删除链接语句。

### 3.9 顶层 oneTBB 子模块记录损坏

Git 索引把以下目录记录为子模块：

```text
src/third_party/oneTBB
```

但仓库根目录没有 `.gitmodules`，所以：

```bash
git submodule status --recursive
```

会报告找不到 oneTBB 的子模块映射。目前该目录为空。

这不是本次构建的阻塞项，因为相关包实际使用 Ubuntu 的 `libtbb-dev` 和 `find_package(TBB REQUIRED)`。新脚本会提示这个仓库问题，但不会尝试初始化损坏的子模块。

Acados 自己保留了上游 `.gitmodules`。当前 Gitee 仓库虽然将 BLASFEO、HPIPM 的许多 `.c` 文件作为普通文件提交，却没有包含它们的公开 `include/*.h`：`external/blasfeo/include` 为空，`external/hpipm/include` 不存在。因此这些目录仍不足以从零构建。

新脚本会检查关键头文件；缺失时仅在受保护的 `/tmp` staging 目录中从官方上游补齐，并固定到本次验证过的提交：

```text
BLASFEO 22b5a67753781879c1d2d61969ac28c87e00e3ff
HPIPM   57c5dba41909787ea5460aebf7fa03f384f871be
```

不会覆盖工作区中的原始 Gitee 源码。第一次补齐需要访问 GitHub；之后 `/tmp` 缓存仍存在时可复用已下载内容。

### 3.10 旧脚本已经与仓库现状不匹配

旧的 `HERO2026_Sentry_Build.sh` 原本就是为解决第一次构建困难而写，但目前存在：

- 尝试编译已不存在的 `cloud_analys`。
- 不构建 Acados 和新的 MPC 控制器链路。
- 不处理 Bstyle/NLopt。
- 不跳过 Odin。
- 不验证 Acados 外部源码。
- 默认 `NPROC=$(nproc)`，可能让 CPU 长时间满载并触发过热。
- 将 SDK 或库安装到 `/usr/local`，使构建结果依赖当前电脑的历史状态。
- 依靠人工硬编码顺序，不能跟随新的 `package.xml` 依赖图变化。

旧脚本已保留作为历史参考，新脚本不会覆盖它。

### 3.11 过高并行度造成系统不稳定

编译 C++ 模板、PCL、Nav2 和点云算法时，单个包内部已经可以使用很多编译线程。如果 colcon 同时并行多个包，每个包又各自使用全部 CPU，会出现嵌套并行，导致：

- CPU 长时间满载或睿频。
- 风扇高速运行。
- WSL/桌面失去响应。
- 内存瞬时占用增大。
- 强制重启后留下不完整缓存。

正确策略：

- 包与包之间串行。
- 单包默认 8 线程。
- 使用 `nice -n 10` 降低编译调度优先级。
- 线程数可以通过 `--jobs` 调整。
- CPU 频率限制属于机器管理，应由用户单独设置，构建脚本不自动修改系统策略。

## 4. 新脚本采用的正确流程

新脚本：

```text
build_hero2026_safe.sh
```

执行顺序：

1. 检查 ROS、CMake、Ninja、colcon、编译器和关键 Acados 源码。
2. 提示损坏的 oneTBB gitlink，但统一使用系统 TBB。
3. 将源码同步到无特殊字符的 `/tmp` staging 目录。
4. 检查 BLASFEO/HPIPM 公开头文件，必要时从官方仓库补齐固定版本。
5. 构建内嵌 NLopt。
6. 构建并安装 Livox SDK2。
7. 构建并安装 Acados、HPIPM 和 BLASFEO。
8. 使用 ROS 2 Humble 参数构建 Livox 驱动。
9. 读取 colcon 的拓扑顺序，但逐包使用 CMake/Ninja 构建，绕过本机 colcon 包环境收尾卡住的问题。
10. 最后构建无法由 `package.xml` 表达依赖关系的 `align_cloud`。
11. 生成标准 colcon 包索引和顶层 `install/setup.bash`。
12. 检查 ROS 包发现、断链、缺失动态库和临时 RPATH。

## 5. 首次使用新脚本

先赋予执行权限：

```bash
chmod +x build_hero2026_safe.sh
```

如果系统依赖尚未安装：

```bash
./build_hero2026_safe.sh --install-deps --jobs 4
```

依赖已经安装时：

```bash
./build_hero2026_safe.sh --jobs 8
```

只检查现有安装结果：

```bash
./build_hero2026_safe.sh --verify-only
```

如果 `/tmp` 中的 staging/CMake 缓存损坏，可以只清理脚本自己的临时缓存：

```bash
./build_hero2026_safe.sh --clean-stage --jobs 8
```

该选项不会删除工作区中的源码和 `install`。

## 6. 日常修改代码后的增量编译

最稳妥的方式是再次运行新脚本。它复用 `/tmp` 下的 Ninja/CMake 缓存，未修改的目标通常显示无工作可做：

```bash
./build_hero2026_safe.sh --jobs 8
```

如果工作区路径改为不含空格和括号的正常路径，并且只修改普通 ROS 包，也可以使用标准 colcon 增量编译：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select <包名> --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

注意：

- 修改 `.cpp/.hpp`：通常只需重编对应包。
- 修改 `.msg/.srv/.action`：需要重编接口包和所有使用者。
- 修改 `package.xml` 或 CMake：应让 CMake 重新配置。
- 修改 Acados、Livox、NLopt：建议运行完整新脚本。
- 修改 launch/YAML：使用 `--symlink-install` 时通常无需编译；本脚本使用实体安装，为保证结果同步应重新运行对应安装步骤或完整脚本。

## 7. 中断、重启和残留处理

### 7.1 可以安全中断吗

可以使用 `Ctrl+C` 中止。CMake/Ninja 的构建目录可能保留一个未完成目标，但下一次运行会重新生成该目标。

不要在编译进程仍运行时直接删除它正在使用的目录。

### 7.2 哪些目录可以删除

- `/tmp/hero2026_sentry_build_<uid>`：新脚本的 staging 和编译缓存；可通过 `--clean-stage` 安全清理。
- 工作区 `log/build_v2`：日志；确认不需要排错后可以删除。
- 工作区 `build/`：旧构建缓存和恢复归档；删除前确认是否还需要恢复归档。

不要随意删除：

- `src/`：源码。
- `install/`：当前可运行结果。
- `src/planning/Bstyle_smoother/third_party/nlopt/install/`：NLopt 的构建后安装目录。

### 7.3 本次保留的恢复归档

```text
build/codex_resume_20260920.tar
```

大小约 239 MiB，SHA-256：

```text
afc9a25855c26bc4a57de52deee0da4ff9f5cd4cd78519f6e9f05e8c118a95b7
```

它不参与运行。完成仿真或真机验证后，可自行决定是否保留。

## 8. 验收标准

编译命令返回 0 还不够。正确安装至少应满足：

1. `source install/setup.bash` 不报错。
2. `ros2 pkg prefix hero_serial` 指向当前工作区。
3. `ros2 pkg prefix livox_ros_driver2` 指向当前工作区。
4. `ros2 pkg prefix hero_mpc_controller` 指向当前工作区。
5. `ros2 interface show serial_interfaces/msg/Refree` 能显示消息字段。
6. 所有 ELF 文件执行 `ldd` 后没有 `not found`。
7. `install` 中没有断开的软链接。
8. 安装后的 RPATH 不指向 `/tmp/hero2026...`。

新脚本在末尾自动执行这些核心检查。

## 9. 不推荐的处理方式

- 不要为了通过编译删除 `find_package()` 或链接库。
- 不要伪造 Odin 的闭源静态库。
- 不要把所有第三方库都 `sudo make install` 到 `/usr/local`。
- 不要在两个含有同名包的 ROS 工作区之间反复叠加 source。
- 不要把缺依赖导致的 aborted 包当作该包自身错误。
- 不要默认使用全部 CPU 核心并同时并行多个大包。
- 不要下载 ZIP 后假设其中一定包含所有 Git 子模块内容。
- 不要在没有记录的情况下反复手工修改生成目录。

## 10. 建议的后续维护

1. 将当前构建修复提交为独立 Git commit。
2. 在 Gitee 仓库补齐或移除损坏的 oneTBB gitlink。
3. 为第三方库记录确定版本或提交号。
4. 将系统依赖维护在 rosdep/package.xml 中。
5. CI 中至少执行一次无缓存构建和动态库检查。
6. 将机器人实际使用的 launch、命名空间、话题和 TF 约定写入部署文档。
7. 开发机和机器人使用相同 Ubuntu、ROS 发行版和编译器版本。

做到这些后，第一次构建才会从“依赖个人电脑历史状态”变成可重复、可验证的工程流程。
