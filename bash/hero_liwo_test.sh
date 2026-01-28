gnome-terminal --tab --title="start_lidar" -- bash -c "source /home/dji/hero2026_-sentry/install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360_launch.py; bash"
gnome-terminal --tab --title="chassis_Odometry" -- bash -c "source /home/dji/hero2026_-sentry/install/setup.bash && ros2 launch hero_serial chassis_node.launch.py; bash"
sleep 5
gnome-terminal --tab --title="lio" -- bash -c "source /home/dji/hero2026_-sentry/install/setup.bash && ros2 launch hero_liwo hero_liwo_launch.py; bash"