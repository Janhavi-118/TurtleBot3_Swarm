from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world_path = '/home/janhavi/TurtleBot3_Swarm/src/my_gazebo/worlds/turtlebot3_house.world'
    
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_tb3_0 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3_0',
            '-x', '1.0', '-y', '1.5', '-z', '0.01',
            '-file', urdf_path,
            '-robot_namespace', 'tb3_0'
        ],
        output='screen'
    )
    spawn_tb3_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3_1',
            '-x', '6.0', '-y', '-1.0', '-z', '0.01',
            '-file', urdf_path,
            '-robot_namespace', 'tb3_1'
        ],
        output='screen'
    )
    spawn_tb3_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3_2',
            '-x', '2.0', '-y', '-2.0', '-z', '0.01',
            '-file', urdf_path,
            '-robot_namespace', 'tb3_2'
        ],
        output='screen'
    )
    spawn_tb3_3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3_3',
            '-x', '-4.0', '-y', '5.0', '-z', '0.01',
            '-file', urdf_path,
            '-robot_namespace', 'tb3_3'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        spawn_tb3_0,
        spawn_tb3_1,
        spawn_tb3_2,
        spawn_tb3_3
    ])
