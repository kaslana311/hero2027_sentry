#!/usr/bin/env bash
# kill_all_ros2_nodes.sh — 优雅关闭所有 ROS2 节点

# 1. 读取当前图中的节点名（去掉开头的斜杠）
mapfile -t NODES < <(ros2 node list | sed 's#^/##')

# 2. 对每个节点名，先发 SIGINT (类似Ctrl+C)，允许优雅退出
for node in "${NODES[@]}"; do
  echo ">>> Requesting graceful exit for '$node'…"
  pids=$(pgrep -f -- "$node")
  if [[ -n "$pids" ]]; then
    echo "    Sending SIGINT to PIDs: $pids"
    # 使用 -2 (SIGINT) 或 -15 (SIGTERM)，应用中的 signal_handler 都能接住
    kill -2 $pids
  fi
done

# 3. 给程序执行清理工作的时间 (例如断开设备硬件流、关闭日志文件)
echo "Waiting for nodes to shutdown gracefully..."
sleep 3.0

# 4. 终极保险：万一有节点死锁不走，再用强制手段清理遗骸
for node in "${NODES[@]}"; do
  pids=$(pgrep -f -- "$node")
  if [[ -n "$pids" ]]; then
    echo ">>> Force killing stuck node '$node' (PIDs: $pids)"
    kill -9 $pids
  fi
done

echo "All node cleanup finished."