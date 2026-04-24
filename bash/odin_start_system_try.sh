#!/bin/bash
gnome-terminal --tab --title="delete" -- bash -c \
"bash /home/dji/hero2026_-sentry/bash/kill_all_ros2_nodes.sh; bash "
sleep 2
# 设置必要的环境变量
export DISPLAY=:0
export XAUTHORITY=/home/dji/.Xauthority
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source /home/dji/hero2026_-sentry/install/setup.bash
# 创建日志目录
LOG_DIR="$HOME/ros_logs/$(date +"%Y%m%d_%H%M%S")"
mkdir -p "$LOG_DIR"

# 函数：等待ROS话题出现
wait_for_topic() {
    local topic_name=$1
    local timeout=${2:-30}  # 默认超时30秒
    echo "等待话题 $topic_name 上线..."
    local start_time=$(date +%s)
    while ! ros2 topic list | grep -q "$topic_name"; do
        sleep 1
        local current_time=$(date +%s)
        if (( current_time - start_time >= timeout )); then
            echo "错误：等待话题 $topic_name 超时！"
            exit 1
        fi
    done
}

# zenoh 网线通信桥接
gnome-terminal --tab --title="zenoh_bridge" -- bash -c \
"ros2 run zenoh_bridge_dds zenoh_bridge_dds -d 1 -e tcp/192.168.123.20:7447 -a '.*/tracker/autoaim\$|.*/carrot_pose\$' 2>&1 | tee \"$LOG_DIR/zenoh_bridge.log\"; exec bash"

# 启动底盘节点
gnome-terminal --tab --title="chassis" -- bash -c \
"ros2 launch hero_serial chassis_node.launch.py 2>&1 | tee  \"$LOG_DIR/chassis.log\" ; exec bash"

# 启动云台节点
gnome-terminal --tab --title="bridge" -- bash -c \
"ros2 run nav_cv_bridge autoaim_tracker_node 2>&1 | tee \"$LOG_DIR/bridge.log\"; exec bash"

# 启动雷达节点
# gnome-terminal --tab --title="start_lidar" -- bash -c \
# "ros2 launch livox_ros_driver2 msg_MID360_launch.py 2>&1 | tee \"$LOG_DIR/lidar.log\" ; exec bash"

# 等待雷达初始化完成
# wait_for_topic "/livox/imu" 60

gnome-terminal --tab --title="odin" -- bash -c \
"source /home/dji/hero2026_-sentry/install/setup.bash && ros2 launch odin_ros_driver odin1_ros2.launch.py; bash"

# wait_for_topic "/aft_mapped_to_init" 60

# gnome-terminal --tab --title="ekf_fusion" -- bash -c \
# "source /home/dji/hero2026_-sentry/install/setup.bash && ros2 launch tf_manager fusion_launch.py; bash"

# 启动重定位节点
# gnome-terminal --tab --title="relocalization" -- bash -c \
# "ros2 launch relocalization relocalization.launch.py 2>&1 | tee \"$LOG_DIR/relocalization.log\" ; exec bash"

sleep 5  # 适当调整时间或使用话题检测

# 启动导航规划节点
gnome-terminal --tab --title="plan" -- bash -c \
"ros2 launch hero2025_nav_bringup navigation_launch.py 2>&1 | tee \"$LOG_DIR/navigation.log\" ; exec bash"

# 启动地图服务器
gnome-terminal --tab --title="map" -- bash -c \
" export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/home/dji/hero2026_-sentry/src/third_party/acados/lib;
     ros2 launch map_server map_server.launch.py 2>&1 | tee \"$LOG_DIR/map_server.log\" ; exec bash"

# 启动RViz可视化
gnome-terminal --tab --title="rviz" -- bash -c \
"ros2 launch hero2025_nav_bringup rviz_launch.py 2>&1 | tee \"$LOG_DIR/rviz.log\"; exec bash"

# sleep 10
# 启动决策节点（特殊日志处理）
# DECISION_LOG="$LOG_DIR/decision.log"
# gnome-terminal --tab --title="decision" -- bash -c \
# "script -q -f \"$DECISION_LOG\" -c 'ros2 launch bt decision_launch.py'; exec bash"

echo "所有节点已启动，系统初始化完成！"
echo "完整日志目录: $LOG_DIR"