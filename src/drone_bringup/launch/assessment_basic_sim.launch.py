import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('drone_bringup')
    mission_sim = os.path.join(share, 'launch', 'mission_sim.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2 for the basic assessment.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mission_sim),
            launch_arguments={
                'use_rviz': LaunchConfiguration('use_rviz'),
                'start_with_configured_waypoints': 'false',
            }.items(),
        ),
    ])
