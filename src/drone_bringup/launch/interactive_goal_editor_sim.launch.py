import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('drone_bringup')
    environment_parameters = os.path.join(
        bringup_share, 'config', 'environment.yaml')
    astar_parameters = os.path.join(
        bringup_share, 'config', 'astar.yaml')
    trajectory_parameters = os.path.join(
        bringup_share, 'config', 'planned_trajectory.yaml')
    editor_parameters = os.path.join(
        bringup_share, 'config', 'interactive_goal_editor.yaml')
    rviz_config = os.path.join(bringup_share, 'rviz', 'drone_sim.rviz')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 with the interactive goal editor.',
        ),
        Node(
            package='drone_planning',
            executable='static_environment_node',
            name='static_environment_node',
            output='screen',
            parameters=[environment_parameters],
        ),
        Node(
            package='drone_planning',
            executable='interactive_goal_editor_node',
            name='interactive_goal_editor_node',
            output='screen',
            parameters=[
                environment_parameters,
                astar_parameters,
                trajectory_parameters,
                editor_parameters,
                {'execution_enabled': False},
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
