import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('drone_bringup')
    disturbance_demo = os.path.join(
        share, 'launch', 'disturbance_visual_demo.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'profile', default_value='short_gust',
            description='Disturbance profile: short_gust or persistent_release.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2 for the disturbance assessment.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(disturbance_demo),
            launch_arguments={
                'profile': LaunchConfiguration('profile'),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items(),
        ),
    ])
