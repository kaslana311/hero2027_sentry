#!/usr/bin/env bash
# hero2026 sentry reproducible build helper
#
# Default behavior:
#   - ROS 2 Humble
#   - at most 8 compiler jobs
#   - one package at a time
#   - skip odin_ros_driver (missing proprietary vendor library)
#   - keep all third-party libraries inside this workspace
#   - use a punctuation-free staging path to protect old CMake scripts

set -Eeuo pipefail
IFS=$'\n\t'

ROS_DISTRO="${ROS_DISTRO:-humble}"
JOBS="${JOBS:-8}"
INSTALL_DEPS=false
CLEAN_STAGE=false
VERIFY_ONLY=false

# Pinned upstream revisions used to repair the incomplete Acados dependency
# snapshots shipped by this repository.
BLASFEO_COMMIT="22b5a67753781879c1d2d61969ac28c87e00e3ff"
HPIPM_COMMIT="57c5dba41909787ea5460aebf7fa03f384f871be"
GTSAM_COMMIT="4f66a491ffc83cf092d0d818b11dc35135521612"

usage() {
  cat <<'EOF'
Usage: ./build_hero2026_safe.sh [options]

Options:
  --jobs N          Compiler jobs inside one package (default: 8)
  --install-deps    Install Ubuntu/ROS dependencies with apt and rosdep
  --clean-stage     Remove only this script's /tmp staging/build cache first
  --verify-only     Do not build; audit the existing install directory
  -h, --help        Show this help

Environment variables:
  ROS_DISTRO        ROS 2 distribution (default: humble)
  JOBS              Same as --jobs
  HERO_BUILD_STAGE  Staging root. It must be under /tmp/hero2026_sentry_build_*

Examples:
  ./build_hero2026_safe.sh --jobs 8
  ./build_hero2026_safe.sh --install-deps --jobs 4
  ./build_hero2026_safe.sh --verify-only
EOF
}

while (($#)); do
  case "$1" in
    --jobs)
      [[ $# -ge 2 ]] || { echo "--jobs requires a value" >&2; exit 2; }
      JOBS="$2"
      shift 2
      ;;
    --install-deps)
      INSTALL_DEPS=true
      shift
      ;;
    --clean-stage)
      CLEAN_STAGE=true
      shift
      ;;
    --verify-only)
      VERIFY_ONLY=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ "$JOBS" =~ ^[1-9][0-9]*$ ]] || {
  echo "JOBS must be a positive integer, got: $JOBS" >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$SCRIPT_DIR"
STAGE_BASE="${HERO_BUILD_STAGE:-/tmp/hero2026_sentry_build_$(id -u)}"

case "$STAGE_BASE" in
  /tmp/hero2026_sentry_build_*) ;;
  *)
    echo "Refusing unsafe staging path: $STAGE_BASE" >&2
    echo "HERO_BUILD_STAGE must start with /tmp/hero2026_sentry_build_" >&2
    exit 2
    ;;
esac

STAGE_SRC="$STAGE_BASE/source"
BUILD_ROOT="$STAGE_BASE/build"
LOG_ROOT="$WS_ROOT/log/build_v2"
INSTALL_ROOT="$WS_ROOT/install"

info() { printf '\033[1;34m[INFO]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[WARN]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[ERROR]\033[0m %s\n' "$*" >&2; exit 1; }

show_log_tail() {
  local log_file="$1"
  if [[ -f "$log_file" ]]; then
    printf '\nLast 120 lines of %s:\n' "$log_file" >&2
    tail -120 "$log_file" >&2
  fi
}

install_dependencies() {
  info "Installing system and ROS dependencies (sudo is expected once)"
  sudo apt update
  sudo apt install -y \
    build-essential cmake ninja-build git pkg-config \
    python3-colcon-common-extensions python3-rosdep \
    libeigen3-dev libpcl-dev libopencv-dev libyaml-cpp-dev \
    libtbb-dev libomp-dev libapr1-dev qtbase5-dev \
    libzmq3-dev libsqlite3-dev \
    "ros-${ROS_DISTRO}-navigation2" \
    "ros-${ROS_DISTRO}-nav2-bringup" \
    "ros-${ROS_DISTRO}-grid-map"

  if command -v rosdep >/dev/null 2>&1; then
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
      sudo rosdep init || warn "rosdep init failed; continuing with explicit apt dependencies"
    fi
    rosdep update || warn "rosdep update failed; check the network and retry later"
    rosdep install --from-paths "$WS_ROOT/src" --ignore-src \
      --rosdistro "$ROS_DISTRO" -r -y || \
      warn "rosdep reported unresolved optional/vendor dependencies; the build will perform its own checks"
  fi
}

preflight() {
  local command_name
  local required=(cmake ninja colcon git g++ find awk sed readelf ldd)

  [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || \
    die "ROS 2 ${ROS_DISTRO} is not installed under /opt/ros/${ROS_DISTRO}"
  [[ -d "$WS_ROOT/src" ]] || die "Workspace has no src directory: $WS_ROOT"

  for command_name in "${required[@]}"; do
    command -v "$command_name" >/dev/null 2>&1 || \
      die "Missing command: $command_name (run this script once with --install-deps)"
  done

  local cpu_count
  cpu_count="$(nproc)"
  if ((JOBS > cpu_count)); then
    warn "Requested $JOBS jobs but only $cpu_count CPUs are visible; using $cpu_count"
    JOBS="$cpu_count"
  fi
  if ((JOBS > 8)); then
    warn "More than 8 jobs can cause high temperature on this machine"
  fi

  # The repository contains a gitlink for oneTBB but no root .gitmodules.
  # It is intentionally not initialized: all packages use Ubuntu's libtbb-dev.
  if git -C "$WS_ROOT" ls-files --stage 2>/dev/null | \
      awk '$1 == 160000 && $4 == "src/third_party/oneTBB" {found=1} END {exit !found}'; then
    if [[ ! -f "$WS_ROOT/.gitmodules" ]]; then
      warn "Broken oneTBB gitlink detected; using system libtbb-dev instead"
    fi
  fi

  local acados_root="$WS_ROOT/src/third_party/acados"
  if [[ ! -f "$acados_root/external/blasfeo/include/blasfeo_common.h" ]]; then
    warn "Acados BLASFEO headers are incomplete; build mode will fetch the pinned upstream revision"
  fi
  if [[ ! -f "$acados_root/external/hpipm/include/hpipm_common.h" ]]; then
    warn "Acados HPIPM headers are incomplete; build mode will fetch the pinned upstream revision"
  fi

  info "Workspace: $WS_ROOT"
  info "ROS: $ROS_DISTRO"
  info "Compiler jobs per package: $JOBS"
  info "Package concurrency: 1 (sequential)"
  info "odin_ros_driver: skipped (proprietary liblydHostApi_amd.a is absent)"
}

source_ros_cleanly() {
  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH || true
  unset LD_LIBRARY_PATH PYTHONPATH || true
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
}

sync_stage() {
  if [[ "$CLEAN_STAGE" == true && -e "$STAGE_BASE" ]]; then
    info "Removing staging cache: $STAGE_BASE"
    rm -rf -- "$STAGE_BASE"
  fi

  mkdir -p "$STAGE_BASE" "$BUILD_ROOT" "$LOG_ROOT" "$INSTALL_ROOT"
  if [[ -e "$STAGE_SRC" ]]; then
    case "$STAGE_SRC" in
      /tmp/hero2026_sentry_build_*/source) rm -rf -- "$STAGE_SRC" ;;
      *) die "Refusing to replace unexpected source stage: $STAGE_SRC" ;;
    esac
  fi
  mkdir -p "$STAGE_SRC"

  info "Copying sources to punctuation-free staging path"
  cp -a "$WS_ROOT/src/." "$STAGE_SRC/"
}

fetch_pinned_dependency() {
  local dependency_name="$1"
  local repository_url="$2"
  local commit="$3"
  local target_dir="$4"
  local required_file="$5"
  local cache_dir="$STAGE_BASE/vendor_cache/${dependency_name}-${commit}"
  local clone_dir="${cache_dir}.fetching"
  local log_file="$LOG_ROOT/fetch-${dependency_name}.log"

  [[ -f "$target_dir/$required_file" ]] && return

  case "$target_dir" in
    /tmp/hero2026_sentry_build_*/source/third_party/acados/external/*) ;;
    *) die "Refusing to replace dependency outside the protected stage: $target_dir" ;;
  esac

  if [[ ! -f "$cache_dir/$required_file" ]]; then
    info "Fetching $dependency_name at pinned commit $commit"
    mkdir -p "$STAGE_BASE/vendor_cache"
    rm -rf -- "$clone_dir"
    if ! git clone "$repository_url" "$clone_dir" >"$log_file" 2>&1; then
      show_log_tail "$log_file"
      die "Unable to clone $dependency_name; check network access"
    fi
    if ! git -C "$clone_dir" checkout --detach "$commit" >>"$log_file" 2>&1; then
      show_log_tail "$log_file"
      die "Unable to check out pinned $dependency_name commit $commit"
    fi
    [[ -f "$clone_dir/$required_file" ]] || \
      die "$dependency_name checkout is still missing $required_file"
    mv "$clone_dir" "$cache_dir"
  fi

  rm -rf -- "$target_dir"
  cp -a "$cache_dir" "$target_dir"
}

ensure_acados_sources() {
  local external_root="$STAGE_SRC/third_party/acados/external"

  fetch_pinned_dependency \
    BLASFEO https://github.com/giaf/blasfeo.git "$BLASFEO_COMMIT" \
    "$external_root/blasfeo" include/blasfeo_common.h
  fetch_pinned_dependency \
    HPIPM https://github.com/giaf/hpipm.git "$HPIPM_COMMIT" \
    "$external_root/hpipm" include/hpipm_common.h
}

prefix_path=""
add_prefix() {
  local prefix="$1"
  if [[ -z "$prefix_path" ]]; then
    prefix_path="$prefix"
  else
    prefix_path="$prefix:$prefix_path"
  fi
}

configure_build_install() {
  local package_name="$1"
  local source_dir="$2"
  local build_dir="$3"
  local install_dir="$4"
  shift 4
  local -a extra_args=("$@")
  local log_file="$LOG_ROOT/${package_name}.log"

  info "Building $package_name"
  mkdir -p "$build_dir" "$install_dir"

  if ! CMAKE_PREFIX_PATH="$prefix_path" nice -n 10 cmake \
      -S "$source_dir" -B "$build_dir" -G Ninja \
      "-DCMAKE_INSTALL_PREFIX=$install_dir" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      -DBUILD_TESTING=OFF \
      "${extra_args[@]}" >"$log_file" 2>&1; then
    show_log_tail "$log_file"
    die "CMake configure failed: $package_name"
  fi

  if ! nice -n 10 cmake --build "$build_dir" --parallel "$JOBS" >>"$log_file" 2>&1; then
    show_log_tail "$log_file"
    die "Compilation failed: $package_name"
  fi

  if ! cmake --install "$build_dir" >>"$log_file" 2>&1; then
    show_log_tail "$log_file"
    die "Installation failed: $package_name"
  fi
}

build_third_party() {
  local nlopt_install="$WS_ROOT/src/planning/Bstyle_smoother/third_party/nlopt/install"

  # Optional localization backend: never replace a system GTSAM installation.
  if [[ -f "$STAGE_SRC/localization/z_localization/src/backend/package.xml" ]]; then
    local gtsam_source="$STAGE_BASE/vendor_cache/gtsam-$GTSAM_COMMIT"
    if [[ ! -f "$gtsam_source/CMakeLists.txt" ]]; then
      local gtsam_fetch="$STAGE_BASE/vendor_cache/gtsam-$GTSAM_COMMIT.fetching"
      [[ ! -e "$gtsam_fetch" ]] || die "Incomplete GTSAM download: $gtsam_fetch (inspect before retrying)"
      info "Fetching pinned GTSAM 4.2 ($GTSAM_COMMIT) into the protected build cache"
      git clone --depth 1 --branch 4.2 https://github.com/borglab/gtsam.git "$gtsam_fetch"
      [[ "$(git -C "$gtsam_fetch" rev-parse HEAD)" == "$GTSAM_COMMIT" ]] || die "Unexpected GTSAM tag revision"
      mv "$gtsam_fetch" "$gtsam_source"
    fi
    [[ "$(git -C "$gtsam_source" rev-parse HEAD)" == "$GTSAM_COMMIT" ]] || die "GTSAM cache revision mismatch"
    configure_build_install gtsam "$gtsam_source" "$BUILD_ROOT/gtsam" "$INSTALL_ROOT/gtsam" \
      -DBUILD_SHARED_LIBS=ON -DGTSAM_USE_SYSTEM_EIGEN=ON \
      -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_PYTHON=OFF \
      -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
    add_prefix "$INSTALL_ROOT/gtsam"
  fi

  configure_build_install \
    nlopt \
    "$STAGE_SRC/planning/Bstyle_smoother/third_party/nlopt" \
    "$BUILD_ROOT/nlopt" \
    "$nlopt_install" \
    -DNLOPT_PYTHON=OFF -DNLOPT_MATLAB=OFF -DNLOPT_OCTAVE=OFF \
    -DNLOPT_GUILE=OFF -DNLOPT_SWIG=OFF -DNLOPT_TESTS=OFF

  configure_build_install \
    livox_sdk2 \
    "$STAGE_SRC/third_party/Livox-SDK2" \
    "$BUILD_ROOT/livox_sdk2" \
    "$INSTALL_ROOT/livox_sdk2"
  add_prefix "$INSTALL_ROOT/livox_sdk2"

  configure_build_install \
    acados \
    "$STAGE_SRC/third_party/acados" \
    "$BUILD_ROOT/acados" \
    "$INSTALL_ROOT/acados" \
    "-DACADOS_INSTALL_DIR=$INSTALL_ROOT/acados" \
    -DACADOS_EXAMPLES=OFF -DACADOS_UNIT_TESTS=OFF \
    -DBLASFEO_TARGET=X64_AUTOMATIC -DHPIPM_TARGET=GENERIC
  add_prefix "$INSTALL_ROOT/acados"

  configure_build_install \
    livox_ros_driver2 \
    "$STAGE_SRC/third_party/livox_ros_driver2" \
    "$BUILD_ROOT/livox_ros_driver2" \
    "$INSTALL_ROOT/livox_ros_driver2" \
    -DAMENT_CMAKE_SYMLINK_INSTALL=OFF \
    -DCMAKE_SKIP_RPATH=TRUE \
    -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
  add_prefix "$INSTALL_ROOT/livox_ros_driver2"
}

build_workspace_packages() {
  local nlopt_install="$WS_ROOT/src/planning/Bstyle_smoother/third_party/nlopt/install"
  local package_name source_dir _package_type row package_listing
  local -a package_rows extra_args

  if ! package_listing="$(
    colcon list --base-paths "$STAGE_SRC" --topological-order \
      --packages-skip \
        odin_ros_driver align_cloud acados livox_sdk2 livox_ros_driver2 \
      2>>"$LOG_ROOT/colcon-list.log"
  )"; then
    show_log_tail "$LOG_ROOT/colcon-list.log"
    die "Unable to calculate the colcon package order"
  fi
  [[ -n "$package_listing" ]] || die "Colcon did not discover any workspace packages"
  mapfile -t package_rows <<<"$package_listing"

  local index=0
  local total="${#package_rows[@]}"
  for row in "${package_rows[@]}"; do
    IFS=$'\t' read -r package_name source_dir _package_type <<< "$row"
    index=$((index + 1))
    info "[$index/$total] $package_name"

    extra_args=(
      -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
      -DBTCPP_UNIT_TESTS=OFF
      -DBTCPP_EXAMPLES=OFF
      "-DACADOS_INSTALL_DIR=$INSTALL_ROOT/acados"
      "-DNLOPT_ROOT=$nlopt_install"
    )

    case "$package_name" in
      interfaces|serial_interfaces|localization_interfaces|fast_lio)
        extra_args+=(-DCMAKE_SKIP_RPATH=TRUE)
        ;;
    esac

    configure_build_install \
      "$package_name" "$source_dir" "$BUILD_ROOT/$package_name" \
      "$INSTALL_ROOT/$package_name" "${extra_args[@]}"
    add_prefix "$INSTALL_ROOT/$package_name"
  done

  # align_cloud is a plain CMake project and has no package.xml dependency
  # graph, so build it only after small_gicp is installed.
  configure_build_install \
    align_cloud \
    "$STAGE_SRC/common/align_cloud" \
    "$BUILD_ROOT/align_cloud" \
    "$INSTALL_ROOT/align_cloud" \
    -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON
  add_prefix "$INSTALL_ROOT/align_cloud"
}

write_pure_cmake_hook() {
  local package_name="$1"
  local variable_name="$2"
  local relative_value="$3"
  local hook_name="$4"
  local share_dir="$INSTALL_ROOT/$package_name/share/$package_name"

  mkdir -p "$share_dir/hook"
  printf 'prepend-non-duplicate;%s;%s\n' "$variable_name" "$relative_value" \
    >"$share_dir/hook/$hook_name.dsv"
  printf 'source;share/%s/hook/%s.dsv\n' "$package_name" "$hook_name" \
    >>"$share_dir/package.dsv"
}

generate_colcon_metadata() {
  local package_name prefix marker dependency_file

  # Recreate package.dsv files for plain CMake packages.
  local -a plain_packages=(acados livox_sdk2 small_gicp align_cloud)
  if [[ -d "$INSTALL_ROOT/gtsam" ]]; then plain_packages+=(gtsam); fi
  for package_name in "${plain_packages[@]}"; do
    mkdir -p "$INSTALL_ROOT/$package_name/share/$package_name"
    : >"$INSTALL_ROOT/$package_name/share/$package_name/package.dsv"
  done
  write_pure_cmake_hook acados CMAKE_PREFIX_PATH '' cmake_prefix_path
  write_pure_cmake_hook acados LD_LIBRARY_PATH lib ld_library_path_lib
  write_pure_cmake_hook livox_sdk2 CMAKE_PREFIX_PATH '' cmake_prefix_path
  write_pure_cmake_hook livox_sdk2 LD_LIBRARY_PATH lib ld_library_path_lib
  write_pure_cmake_hook small_gicp CMAKE_PREFIX_PATH '' cmake_prefix_path
  write_pure_cmake_hook small_gicp LD_LIBRARY_PATH lib ld_library_path_lib
  write_pure_cmake_hook align_cloud PATH bin path_bin
  if [[ -d "$INSTALL_ROOT/gtsam" ]]; then
    write_pure_cmake_hook gtsam CMAKE_PREFIX_PATH '' cmake_prefix_path
    write_pure_cmake_hook gtsam LD_LIBRARY_PATH lib ld_library_path_lib
  fi

  # Direct CMake installation does not create colcon package markers. The
  # top-level setup utility reads these markers dynamically.
  for prefix in "$INSTALL_ROOT"/*; do
    [[ -d "$prefix" ]] || continue
    package_name="$(basename "$prefix")"
    marker="$prefix/share/colcon-core/packages/$package_name"
    dependency_file="$prefix/share/ament_index/resource_index/package_run_dependencies/$package_name"
    mkdir -p "$(dirname "$marker")"
    if [[ -f "$dependency_file" ]]; then
      tr ';' ':' <"$dependency_file" >"$marker"
    elif [[ ! -f "$marker" ]]; then
      : >"$marker"
    fi
  done
}

bootstrap_top_level_setup() {
  # Colcon's top-level setup scripts are generic and dynamically read all
  # markers. Build one tiny package through colcon only when those scripts do
  # not exist. Running outside WS_ROOT avoids the stale colcon_defaults.yaml.
  if [[ -f "$INSTALL_ROOT/setup.bash" && -f "$INSTALL_ROOT/_local_setup_util_sh.py" ]]; then
    return
  fi

  local log_file="$LOG_ROOT/colcon-setup-bootstrap.log"
  local defaults_file="$STAGE_BASE/empty_colcon_defaults.yaml"
  printf '{}\n' >"$defaults_file"
  info "Generating top-level install/setup.bash"

  if ! (
    cd "$STAGE_BASE"
    CMAKE_PREFIX_PATH="$prefix_path" \
    CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" \
    COLCON_DEFAULTS_FILE="$defaults_file" \
    nice -n 10 colcon --log-base "$LOG_ROOT/colcon_setup" build \
      --base-paths "$STAGE_SRC" \
      --build-base "$BUILD_ROOT/colcon_setup" \
      --install-base "$INSTALL_ROOT" \
      --executor sequential \
      --packages-select sophus \
      --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  ) >"$log_file" 2>&1; then
    show_log_tail "$log_file"
    die "Failed to generate top-level setup scripts"
  fi
}

verify_install() {
  [[ -f "$INSTALL_ROOT/setup.bash" ]] || \
    die "Missing $INSTALL_ROOT/setup.bash"
  mkdir -p "$LOG_ROOT"

  source_ros_cleanly
  set +u
  # shellcheck disable=SC1090,SC1091
  source "$INSTALL_ROOT/setup.bash"
  set -u

  local package_name prefix
  local key_packages=(
    livox_ros_driver2 interfaces serial_interfaces hero_serial
    Bstyle_smoother pb_minco_smoother hero_mpc_controller
    relocalization fast_lio point_lio fast_livo
  )
  if [[ -f "$WS_ROOT/src/localization/z_localization/src/backend/package.xml" ]]; then
    key_packages+=(backend localization_interfaces hero_localization_backend)
  fi
  for package_name in "${key_packages[@]}"; do
    prefix="$(ros2 pkg prefix "$package_name" 2>/dev/null || true)"
    [[ "$prefix" == "$INSTALL_ROOT/$package_name" ]] || \
      die "ROS cannot discover $package_name from this workspace"
  done

  local broken_count missing_count temp_rpath_count elf_count
  local missing_log="$LOG_ROOT/verify-missing-libraries.log"
  local rpath_log="$LOG_ROOT/verify-temporary-rpaths.log"
  : >"$missing_log"
  : >"$rpath_log"
  elf_count=0

  while IFS= read -r -d '' candidate; do
    if readelf -h "$candidate" >/dev/null 2>&1; then
      elf_count=$((elf_count + 1))
      ldd "$candidate" 2>/dev/null | \
        awk -v file="$candidate" '/not found/{print file ": " $0}' \
        >>"$missing_log" || true
      readelf -d "$candidate" 2>/dev/null | \
        awk -v file="$candidate" '/RPATH|RUNPATH/ && /\/tmp\/hero2026/{print file ": " $0}' \
        >>"$rpath_log" || true
    fi
  done < <(find -L "$INSTALL_ROOT" -type f \
    \( -perm /111 -o -name '*.so' -o -name '*.so.*' \) -print0)

  broken_count="$(find -L "$INSTALL_ROOT" -type l | wc -l)"
  missing_count="$(wc -l <"$missing_log")"
  temp_rpath_count="$(wc -l <"$rpath_log")"

  info "Installed prefixes: $(find "$INSTALL_ROOT" -mindepth 1 -maxdepth 1 -type d | wc -l)"
  info "ELF files checked: $elf_count"
  info "Broken symlinks: $broken_count"
  info "Missing shared libraries: $missing_count"
  info "Temporary RPATHs: $temp_rpath_count"

  [[ "$broken_count" -eq 0 ]] || die "Broken symlinks remain in install"
  [[ "$missing_count" -eq 0 ]] || { show_log_tail "$missing_log"; die "Missing shared libraries detected"; }
  [[ "$temp_rpath_count" -eq 0 ]] || { show_log_tail "$rpath_log"; die "Temporary RPATHs detected"; }
}

main() {
  if [[ "$INSTALL_DEPS" == true ]]; then
    install_dependencies
  fi
  preflight

  if [[ "$VERIFY_ONLY" == true ]]; then
    verify_install
    info "Verification completed successfully"
    return
  fi

  source_ros_cleanly
  prefix_path="${CMAKE_PREFIX_PATH:-/opt/ros/${ROS_DISTRO}}"
  sync_stage
  ensure_acados_sources
  build_third_party
  build_workspace_packages
  generate_colcon_metadata
  bootstrap_top_level_setup
  # The bootstrap can replace sophus metadata, so refresh all markers once.
  generate_colcon_metadata
  verify_install

  info "Build completed successfully"
  info "Open a new terminal and run:"
  printf "  source /opt/ros/%s/setup.bash\n" "$ROS_DISTRO"
  printf "  source '%s/install/setup.bash'\n" "$WS_ROOT"
}

main "$@"
