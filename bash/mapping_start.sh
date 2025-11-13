gnome-terminal --tab --title="start_lidar" -- bash -c "source /home/dji/project/HERO2025_Sentry/install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360_launch.py; bash"
sleep 2
gnome-terminal --tab --title="lio" -- bash -c "source /home/dji/project/HERO2025_Sentry/install/setup.bash && ros2 launch point_lio point_lio.launch.py; bash"