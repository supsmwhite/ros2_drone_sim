import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('drone_bringup')
    navigation = os.path.join(
        share, 'launch', 'interactive_goal_navigation_sim.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument(
            'yaw_mode', default_value='path_tangent',
            description='Yaw reference mode forwarded to the navigation chain.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2 for interactive navigation.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation),
            launch_arguments={
                'use_rviz': LaunchConfiguration('use_rviz'),
                'yaw_mode': LaunchConfiguration('yaw_mode'),
            }.items(),
        ),
    ])
