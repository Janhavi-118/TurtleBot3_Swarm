from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robots = ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']
    nodes = []

    for robot in robots:
        nodes.append(Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            namespace=robot,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'slam_params_file': '/path/to/slam_params.yaml'
            }]
        ))

    return LaunchDescription(nodes)
