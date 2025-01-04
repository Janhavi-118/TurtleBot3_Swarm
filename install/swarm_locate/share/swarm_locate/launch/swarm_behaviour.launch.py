from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_locate',
            executable='detect_ball',
            output='screen',
        ),
        Node(
            package='swarm_locate',
            executable='wall_follow',
            output='screen',
        ),
        Node(
            package='swarm_locate',
            executable='ball_navigation',
            output='screen',
        ),
    ])
