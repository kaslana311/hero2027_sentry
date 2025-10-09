#!/usr/bin/env bash
# kill_all_ros2_nodes.sh — 先列 ROS2 节点名，再挨个强杀对应进程

# 1. 读取当前图中的节点名（去掉开头的斜杠）
mapfile -t NODES < <(ros2 node list | sed 's#^/##')

# 2. 对每个节点名：
for node in "${NODES[@]}"; do
  echo ">>> Killing node process for '$node'…"
  # 用 pgrep 在完整命令行里匹配“节点名”
  pids=$(pgrep -f -- "$node")
  if [[ -z "$pids" ]]; then
    echo "    No PID found for $node"
  else
    echo "    PIDs: $pids"
    kill -9 $pids
  fi
done