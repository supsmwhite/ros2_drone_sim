import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('drone_bringup')
    basic_sim = os.path.join(bringup_share, 'launch', 'basic_sim.launch.py')
    default_mission_parameters = os.path.join(bringup_share, 'config', 'mission.yaml')
    environment_parameters = os.path.join(bringup_share, 'config', 'environment.yaml')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 through the included basic simulation.',
        ),
        DeclareLaunchArgument(
            'mission_config', default_value=default_mission_parameters,
            description='Obstacle-free waypoint mission parameter file.'),
        DeclareLaunchArgument(
            'start_with_configured_waypoints', default_value='true',
            description='Start YAML waypoints, or wait for goal_cli multi when false.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(basic_sim),
            launch_arguments={'use_rviz': use_rviz}.items(),
        ),
        Node(
            package='drone_mission',
            executable='waypoint_manager_node',
            name='waypoint_manager_node',
            output='screen',
            parameters=[
                environment_parameters,
                LaunchConfiguration('mission_config'),
                {'start_with_configured_waypoints': ParameterValue(
                    LaunchConfiguration('start_with_configured_waypoints'),
                    value_type=bool)},
            ],
        ),
    ])
