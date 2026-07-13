import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('drone_bringup')
    dynamics_parameters = os.path.join(
        bringup_share,
        'config',
        'dynamics.yaml',
    )
    controller_parameters = os.path.join(
        bringup_share,
        'config',
        'controller.yaml',
    )
    xacro_file = os.path.join(bringup_share, 'urdf', 'drone.urdf.xacro')
    rviz_config = os.path.join(bringup_share, 'rviz', 'drone_sim.rviz')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 with the drone simulation configuration.',
        ),
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
            parameters=[controller_parameters],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
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
