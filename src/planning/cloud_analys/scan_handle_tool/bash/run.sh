#! /bin/bash

#yaml 路径 +pcd 路径变量
YAML_PATH="/home/z/ros_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/rmuc_2026.yaml"
PCD_PATH="/home/z/ros_ws/src/pb2025_sentry_nav/cloud_analys/tool/bin/filtered_output.pcd"
min_z="-0.5"
max_z="1.0"
SAVE_PATH="/home/z/ros_ws/src/pb2025_sentry_nav/cloud_anal/high_map.png"
HEIGHT_PERCENT="0.95"

# 获得当前脚本所在路径
DIR=$(cd "$(dirname "$0")"; pwd)
# 切换到项目根目录
cd "$DIR/../bin"
# 运行scan_handle_tool节点
./scan_handle_tool  "$YAML_PATH" "$PCD_PATH"

