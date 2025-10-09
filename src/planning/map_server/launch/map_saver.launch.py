from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    map_saver_node = Node(
    package='nav2_map_server',
    executable='map_saver_cli',
    name='map_saver_cli',
    output='screen',
    arguments=['-f', '/home/dji/HERO2025_Sentry/src/planning/map_server/map/map']
    )
    return LaunchDescription([map_saver_node])