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
    environment = os.path.join(share, 'config', 'environment.yaml')
    astar_default = os.path.join(share, 'config', 'astar.yaml')
    trajectory = os.path.join(share, 'config', 'planned_trajectory.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('astar_config', default_value=astar_default),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core), launch_arguments={
                'use_rviz': LaunchConfiguration('use_rviz'),
                'setpoint_source': 'trajectory'}.items()),
        Node(package='drone_planning', executable='static_environment_node',
             name='static_environment_node', output='screen', parameters=[environment]),
        Node(package='drone_planning', executable='astar_planner_node',
             name='astar_planner_node', output='screen',
             parameters=[environment, LaunchConfiguration('astar_config')]),
        Node(package='drone_planning', executable='planned_trajectory_node',
             name='planned_trajectory_node', output='screen', parameters=[
                 environment, LaunchConfiguration('astar_config'), trajectory,
                 {'execution_enabled': True}]),
    ])
