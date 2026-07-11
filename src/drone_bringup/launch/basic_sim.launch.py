import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    dynamics_parameters = os.path.join(
        get_package_share_directory('drone_bringup'),
        'config',
        'dynamics.yaml',
    )

    return LaunchDescription([
        Node(
            package='drone_dynamics',
            executable='quadrotor_dynamics_node',
            name='quadrotor_dynamics_node',
            output='screen',
            parameters=[dynamics_parameters],
        ),
        Node(
            package='drone_controller',
            executable='position_controller_node',
            name='position_controller_node',
            output='screen',
        ),
    ])
