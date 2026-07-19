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
    mission_default = os.path.join(share, 'config', 'multi_goal_mission.yaml')
    dynamics_default = os.path.join(share, 'config', 'dynamics.yaml')
    controller_default = os.path.join(share, 'config', 'controller.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('mission_config', default_value=mission_default),
        DeclareLaunchArgument('dynamics_config', default_value=dynamics_default),
        DeclareLaunchArgument('controller_config', default_value=controller_default),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core), launch_arguments={
                'use_rviz': LaunchConfiguration('use_rviz'),
                'setpoint_source': 'trajectory',
                'dynamics_config': LaunchConfiguration('dynamics_config'),
                'controller_config': LaunchConfiguration('controller_config')}.items()),
        Node(package='drone_planning', executable='static_environment_node',
             name='static_environment_node', output='screen', parameters=[environment]),
        Node(package='drone_planning', executable='multi_goal_static_avoidance_node',
             name='multi_goal_static_avoidance_node', output='screen', parameters=[
                 environment, astar, trajectory, LaunchConfiguration('mission_config')]),
    ])
