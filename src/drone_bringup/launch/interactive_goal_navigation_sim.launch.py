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
    astar = os.path.join(share, 'config', 'astar.yaml')
    trajectory = os.path.join(share, 'config', 'planned_trajectory.yaml')
    editor = os.path.join(share, 'config', 'interactive_goal_editor.yaml')
    executor = os.path.join(share, 'config', 'interactive_goal_executor.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core), launch_arguments={
                'use_rviz': LaunchConfiguration('use_rviz'),
                'setpoint_source': 'trajectory'}.items()),
        Node(package='drone_planning', executable='static_environment_node',
             name='static_environment_node', output='screen', parameters=[environment]),
        Node(package='drone_planning', executable='interactive_goal_editor_node',
             name='interactive_goal_editor_node', output='screen', parameters=[
                 environment, astar, trajectory, editor, {'execution_enabled': True}]),
        Node(package='drone_planning', executable='multi_goal_static_avoidance_node',
             name='multi_goal_static_avoidance_node', output='screen',
             parameters=[environment, astar, trajectory, executor]),
    ])
