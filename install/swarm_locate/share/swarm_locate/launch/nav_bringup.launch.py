from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robots = ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']
    nodes = []

    for robot in robots:
        nodes.append(Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            namespace=robot,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'map': '/path/to/map.yaml',
                'autostart': True,
                'use_lifecycle_mgr': True
            }]
        ))

    return LaunchDescription(nodes)
