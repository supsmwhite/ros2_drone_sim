import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('drone_bringup')
    planning_sim = os.path.join(
        bringup_share, 'launch', 'planning_sim.launch.py')
    environment_parameters = os.path.join(
        bringup_share, 'config', 'environment.yaml')
    astar_parameters = os.path.join(
        bringup_share, 'config', 'astar.yaml')
    trajectory_parameters = os.path.join(
        bringup_share, 'config', 'planned_trajectory.yaml')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 through the included planning simulation.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planning_sim),
            launch_arguments={'use_rviz': use_rviz}.items(),
        ),
        Node(
            package='drone_planning',
            executable='planned_trajectory_node',
            name='planned_trajectory_node',
            output='screen',
            parameters=[
                environment_parameters,
                astar_parameters,
                trajectory_parameters,
            ],
        ),
    ])
