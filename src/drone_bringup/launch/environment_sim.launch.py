import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('drone_bringup')
    basic_sim = os.path.join(bringup_share, 'launch', 'basic_sim.launch.py')
    environment_parameters = os.path.join(
        bringup_share, 'config', 'environment.yaml')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 through the included basic simulation.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(basic_sim),
            launch_arguments={'use_rviz': use_rviz}.items(),
        ),
        Node(
            package='drone_planning',
            executable='static_environment_node',
            name='static_environment_node',
            output='screen',
            parameters=[environment_parameters],
        ),
    ])
