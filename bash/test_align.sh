 gnome-terminal --tab --title="lio" -- bash -c "source /home/dji/hero2026_-sentry/install/setup.bash && ros2 launch livox_ros_driver2 msg_mixed_launch.py; bash"
sleep 2
gnome-terminal --tab --title="lio" -- bash -c "source /home/dji/hero2026_-sentry/install/setup.bash && ros2 launch fast_lio mapping.launch.py; bash"
