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
        DeclareLaunchArgument('yaw_mode', default_value='fixed'),
        DeclareLaunchArgument('fixed_yaw', default_value='0.0'),
        DeclareLaunchArgument('tangent_speed_threshold', default_value='0.10'),
        DeclareLaunchArgument('terminal_blend_distance', default_value='0.80'),
        DeclareLaunchArgument('yaw_filter_time_constant', default_value='0.30'),
        DeclareLaunchArgument('max_yaw_rate', default_value='0.80'),
        DeclareLaunchArgument('nominal_speed', default_value='0.35'),
        DeclareLaunchArgument('max_reference_speed', default_value='0.70'),
        DeclareLaunchArgument('max_reference_acceleration', default_value='0.35'),
        DeclareLaunchArgument('shortcut_preferred_clearance', default_value='0.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core), launch_arguments={
                'use_rviz': LaunchConfiguration('use_rviz'),
                'setpoint_source': 'trajectory'}.items()),
        Node(package='drone_planning', executable='static_environment_node',
             name='static_environment_node', output='screen', parameters=[environment]),
        Node(package='drone_planning', executable='interactive_goal_editor_node',
             name='interactive_goal_editor_node', output='screen', parameters=[
                 environment, astar, trajectory, editor,
                 {'execution_enabled': True,
                  'nominal_speed': LaunchConfiguration('nominal_speed'),
                  'max_reference_speed': LaunchConfiguration('max_reference_speed'),
                  'max_reference_acceleration': LaunchConfiguration(
                      'max_reference_acceleration'),
                  'shortcut_preferred_clearance': LaunchConfiguration(
                      'shortcut_preferred_clearance')}]),
        Node(package='drone_planning', executable='multi_goal_static_avoidance_node',
             name='multi_goal_static_avoidance_node', output='screen',
             parameters=[environment, astar, trajectory, executor,
                         {'yaw_mode': LaunchConfiguration('yaw_mode'),
                          'fixed_yaw': LaunchConfiguration('fixed_yaw'),
                          'tangent_speed_threshold': LaunchConfiguration(
                              'tangent_speed_threshold'),
                          'terminal_blend_distance': LaunchConfiguration(
                              'terminal_blend_distance'),
                          'yaw_filter_time_constant': LaunchConfiguration(
                              'yaw_filter_time_constant'),
                          'max_yaw_rate': LaunchConfiguration('max_yaw_rate'),
                          'nominal_speed': LaunchConfiguration('nominal_speed'),
                          'max_reference_speed': LaunchConfiguration(
                              'max_reference_speed'),
                          'max_reference_acceleration': LaunchConfiguration(
                              'max_reference_acceleration'),
                          'shortcut_preferred_clearance': LaunchConfiguration(
                              'shortcut_preferred_clearance')}]),
    ])
