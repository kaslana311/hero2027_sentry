gnome-terminal --tab --title="chassis_Odometry" -- bash -c "source /home/dji/hero2026_-sentry/install/setup.bash && ros2 launch hero_serial chassis_node.launch.py; bash"
gnome-terminal --tab --title="start_lidar" -- bash -c "source /home/dji/hero2026_-sentry/install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360_launch.py; bash"
sleep 6
gnome-terminal --tab --title="fast_livo" -- bash -c "source /home/dji/hero2026_-sentry/install/setup.bash && ros2 launch fast_livo mapping_avia.launch.py; bash"
gnome-terminal --tab --title="ekf_fusion" -- bash -c "source /home/dji/hero2026_-sentry/install/setup.bash && ros2 launch tf_manager fusion_launch.py; bash"
