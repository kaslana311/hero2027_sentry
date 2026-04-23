gnome-terminal --tab --title="start_odin" -- bash -c "source /home/dji/hero2026_-sentry/install/setup.bash && ros2 launch odin_ros_driver odin1_ros2.launch.py; bash"
gnome-terminal --tab --title="lio" -- bash -c "source /home/dji/hero2026_-sentry/install/setup.bash && ros2 launch livox_ros_driver2 msg_mixed_launch.py; bash"
gnome-terminal --tab --title="fusion" -- bash -c "source /home/dji/hero2026_-sentry/install/setup.bash && ros2 launch hero_lidar_scan hero_lidar_scan.launch.py; bash"
gnome-terminal --tab --title="fox-glove" -- bash -c "source /home/dji/hero2026_-sentry/install/setup.bash && ros2 run foxglove_bridge foxglove_bridge; bash"
