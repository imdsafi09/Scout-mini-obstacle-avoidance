from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_avoidance',
            executable='avoid_obstacles',
            name='obstacle_avoidance_node',
            output='screen',
            parameters=[],
        ),
    ])


