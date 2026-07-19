import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('drone_bringup')
    core = os.path.join(share, 'launch', 'simulation_core.launch.py')
    dynamics = os.path.join(share, 'config', 'dynamics.yaml')
    controller = os.path.join(share, 'config', 'controller.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('setpoint_source', default_value='pose_goal'),
        DeclareLaunchArgument('dynamics_config', default_value=dynamics),
        DeclareLaunchArgument('controller_config', default_value=controller),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core), launch_arguments={
                'use_rviz': LaunchConfiguration('use_rviz'),
                'setpoint_source': LaunchConfiguration('setpoint_source'),
                'dynamics_config': LaunchConfiguration('dynamics_config'),
                'controller_config': LaunchConfiguration('controller_config'),
            }.items()),
        Node(
            package='drone_mission', executable='goal_visualizer_node',
            name='goal_visualizer_node', output='screen'),
    ])
