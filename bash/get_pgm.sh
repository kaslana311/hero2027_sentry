#!/bin/bash
gnome-terminal --tab --title="livox" -- bash -c "source /home/dji/hero2026_-sentry/install/setup.bash && ros2 launch pcd2pgm pcd2pgm.launch.py; bash"
sleep 10
gnome-terminal --tab --title="map_saver" -- bash -c "source /home/dji/hero2026_-sentry/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f /home/dji/hero2026_-sentry/src/planning/map_server/map/map; bash"