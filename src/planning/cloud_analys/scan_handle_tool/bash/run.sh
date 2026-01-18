#! /bin/bash

#yaml 路径 +pcd 路径变量
YAML_PATH="/home/dji/project/HERO2025_Sentry/src/planning/map_server/map/map.yaml"
PCD_PATH="/home/dji/project/HERO2025_Sentry/src/common/point_cloud_files/filtered_pcd/fast_lio_latest_map.pcd"
min_z="-0.8"
max_z="1.2"
SAVE_PATH="/home/dji/project/HERO2025_Sentry/src/planning/cloud_analys/high_map/high_map.png"
HEIGHT_PERCENT="0.95"

# 获得当前脚本所在路径
DIR=$(cd "$(dirname "$0")"; pwd)
# 切换到项目根目录
cd "$DIR/../bin"
# 运行scan_handle_tool节点
 ./scan_handle_tool  "$YAML_PATH" "$PCD_PATH" "$min_z" "$max_z"  "$SAVE_PATH" "$HEIGHT_PERCENT"

#r /home/dji/project/HERO2025_Sentry/src/planning/map_server/map/map.yaml  /home/dji/project/HERO2025_Sentry/src/common/point_cloud_files/filtered_pcd/fast_lio_latest_map.pcd  -0.8  1.2 /home/dji/project/HERO2025_Sentry/src/planning/cloud_analys/high_map/high_map.png 0.95
#