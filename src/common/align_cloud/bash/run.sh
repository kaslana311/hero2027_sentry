#!/usr/bin/env bash
set -e

WORKSPACE="/home/z/ros_ws/src/hero/align_cloud"
CONFIG="/home/z/ros_ws/src/hero/align_cloud/config/cfg.yaml"

echo "Running align_cloud with config: ${CONFIG}"
cd ${WORKSPACE}/install
export VTK_DISABLE_RENDERER_EXTENSIONS=1
export DISPLAY=:0
./align_cloud_node ${CONFIG}