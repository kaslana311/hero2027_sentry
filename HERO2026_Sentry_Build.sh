#!/usr/bin/env bash
# build_all.sh
# Usage:
#   ./build_all.sh            # 以普通用户运行，需要时会 sudo 提示密码
#   sudo -v && ./build_all.sh # 先缓存 sudo，或者直接运行（不推荐用 root 全程运行）
#
# 注意：不要把明文密码写入脚本。若需要自动输入密码，请参考下方的 expect 示范（不安全，不推荐）。

set -euo pipefail
IFS=$'\n\t'

# --------- 配置项（必要时修改） ----------
ROS_DISTRO=${ROS_DISTRO:-humble}
WS_ROOT="$(cd "$(dirname "$0")" && pwd)"
NPROC=${NPROC:-$(nproc)}
KEEP_SUDO_ALIVE=true   # 设置为 false 可禁用后台 keep-alive
APT_NONINTERACTIVE=true # apt install 是否自动 -y

# --------- 颜色/日志 ----------
info() { printf "\e[1;34m[INFO]\e[0m %s\n" "$*"; }
warn() { printf "\e[1;33m[WARN]\e[0m %s\n" "$*"; }
err()  { printf "\e[1;31m[ERROR]\e[0m %s\n" "$*" >&2; }

# --------- sudo helper ----------
SUDO=""
if [ "$(id -u)" -eq 0 ]; then
  SUDO=""  # 已经为 root
else
  SUDO="sudo"
fi

# 尝试缓存 sudo（会提示密码一次）
if [ -n "$SUDO" ]; then
  info "请求 sudo 密码（仅用于后续 make install / apt install）。"
  if ! $SUDO -v; then
    warn "无法获取 sudo 权限，某些步骤可能会失败。"
  fi
fi

# 若启用，保持 sudo 时间戳（后台）
if [ "$KEEP_SUDO_ALIVE" = true ] && [ -n "$SUDO" ]; then
  ( while true; do
      $SUDO -n true 2>/dev/null || true
      sleep 60
      # 退出条件：父进程结束
      kill -0 "$$" 2>/dev/null || exit
    done ) &
  SUDO_KEEPALIVE_PID=$!
  trap 'info "killing sudo keep-alive ($SUDO_KEEPALIVE_PID)"; kill $SUDO_KEEPALIVE_PID 2>/dev/null || true' EXIT
fi

# --------- 公共函数 ----------
run_in_dir() {
  local dir="$1"; shift
  if [ ! -d "$dir" ]; then
    err "目录不存在: $dir"
    return 1
  fi
  pushd "$dir" >/dev/null
  "$@"
  local rc=$?
  popd >/dev/null
  return $rc
}

build_cmake_project() {
  local repo_rel="$1"  # 相对于 WS_ROOT 的路径
  local repo="$WS_ROOT/$repo_rel"
  info "=== 构建 cmake 项目: $repo_rel ==="
  if [ ! -d "$repo" ]; then
    err "未找到目录 $repo"
    return 1
  fi
  run_in_dir "$repo" bash -c '
    set -e
    mkdir -p build
    cd build
    cmake .. || ( echo "cmake 失败: '"$repo_rel"'" ; exit 1 )
    make -j'"$NPROC"' || ( echo "make 失败: '"$repo_rel"'" ; exit 1 )
    '"$SUDO"' make install || ( echo "make install 失败: '"$repo_rel"'" ; exit 1 )
  '
}

colcon_build_pkg() {
  local pkg="$1"
  info "=== colcon build 包: $pkg ==="
  # 在工作空间根运行 colcon
  pushd "$WS_ROOT" >/dev/null
  colcon build --packages-select "$pkg" --symlink-install || ( err "colcon build 失败: $pkg" && popd >/dev/null && return 1 )
  popd >/dev/null
}

safe_source_install() {
  local f="$WS_ROOT/install/setup.bash"
  if [ -f "$f" ]; then
    info "source $f"
    # 避免 set -u 导致未定义变量报错
    set +u
    # shellcheck disable=SC1090
    . "$f"
    set -u
  else
    warn "未找到 $f（跳过 source）"
  fi
}

# --------- 开始按顺序运行（按你提供的流程） ----------
info "工作空间: $WS_ROOT"
info "使用 CPU 并行度: $NPROC"
info "ROS 发行: $ROS_DISTRO"

# 1
build_cmake_project "src/third_party/Livox-SDK2"

# 2
info "cd src/third_party/livox_ros_driver2/ && ./build.sh humble"
if [ -d "$WS_ROOT/src/third_party/livox_ros_driver2" ]; then
  pushd "$WS_ROOT/src/third_party/livox_ros_driver2" >/dev/null
  ./build.sh "$ROS_DISTRO" || { err "livox_ros_driver2 build.sh 失败"; popd >/dev/null; exit 1; }
  popd >/dev/null
else
  warn "目录不存在: src/third_party/livox_ros_driver2"
fi

# 3
build_cmake_project "src/third_party/small_gicp"

# 4 colcon包（逐个）
colcon_build_pkg "sophus"
colcon_build_pkg "vikit_common"
colcon_build_pkg "vikit_ros"

# 5
colcon_build_pkg "fast_livo"
colcon_build_pkg "point_lio"
colcon_build_pkg "interfaces"
colcon_build_pkg "serial_interfaces"

# 6 source 如果有
safe_source_install

# 7
# colcon_build_pkg "tf_manager"
colcon_build_pkg "fast_lio"
colcon_build_pkg "pcd2pgm"
colcon_build_pkg "fake_vel_transform"

# 8 source
safe_source_install

# 9
colcon_build_pkg "hero_serial"

# 10 apt 安装（可选）
if [ "$APT_NONINTERACTIVE" = true ]; then
  APT_OPTS="-y"
else
  APT_OPTS=""
fi

info "安装 ros-humble-navigation2 及 bringup（需要 sudo）"
$SUDO apt update || warn "apt update 失败（继续）"
$SUDO apt install $APT_OPTS ros-humble-navigation2 ros-humble-nav2-bringup || warn "apt install navigation2 失败（继续）"

# 11 后续 colcon 包
colcon_build_pkg "map_server"
colcon_build_pkg "pb_omni_pid_pursuit_controller"
colcon_build_pkg "pb_nav2_plugins"
colcon_build_pkg "terrain_analysis"
colcon_build_pkg "terrain_analysis_ext"
colcon_build_pkg "pcl_tool"

# 12 behaviortree
colcon_build_pkg "behaviortree_cpp"
safe_source_install
colcon_build_pkg "bt"

info "全部构建完成（可能仍需手动检查错误或再次 source install/setup.bash）。"
info "如果中间某步失败，你可以修复问题后在工作空间根运行： colcon build --packages-select <pkg> --symlink-install"
