#!/bin/bash

# --- 配置区域 ---你说
#gnome-terminal -- 
#sudo apt update
#sudo apt install ros-humble-rosbag2-storage-mcap
#
# 1. 定义你的命名空间 (如果不需要命名空间，留空 "" 即可)
NS="/red_standard_robot1" 

# 2. 定义相对话题名（不要以 / 开头）
RELATIVE_TOPICS="odometry tf tf_static rog_map/inf_occ plan plan_smoothed global_costmap/costmap"

# 3. 存储路径配置
BAG_PATH_BASE="$HOME/ros_bags"
BAG_NAME="esdf_session_$(date +%Y%m%d_%H%M%S)"

# --- 自动处理话题拼接 ---
# 遍历相对话题，自动加上命名空间前缀，形成绝对路径
FULL_TOPICS=""
for t in $RELATIVE_TOPICS; do
    # 拼接结果示例: /dog1/esdf_map
    FULL_TOPICS="$FULL_TOPICS $NS/$t"
done

# 创建存储目录
mkdir -p "$BAG_PATH_BASE"

echo "========================================"
echo "开始录制 ROS2 Bag (带命名空间模式)"
echo "当前命名空间: $NS"
echo "最终话题列表: $FULL_TOPICS"
echo "存储位置: $BAG_PATH_BASE/$BAG_NAME"
echo "========================================"
source /opt/ros/humble/setup.bash
# --- 执行录制 ---
# 使用 -o 指定输出目录
# 使用 -s mcap 推荐格式（确保你安装了 ros-humble-rosbag2-storage-mcap）
ros2 bag record $FULL_TOPICS -o "$BAG_PATH_BASE/$BAG_NAME" -s mcap
