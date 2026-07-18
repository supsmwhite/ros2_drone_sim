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
    default_dynamics_parameters = os.path.join(
        bringup_share,
        'config',
        'dynamics.yaml',
    )
    default_controller_parameters = os.path.join(
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
    setpoint_source = LaunchConfiguration('setpoint_source')
    dynamics_parameters = LaunchConfiguration('dynamics_config')
    controller_parameters = LaunchConfiguration('controller_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 with the drone simulation configuration.',
        ),
        DeclareLaunchArgument(
            'setpoint_source',
            default_value='pose_goal',
            description='Controller input source: pose_goal or trajectory.',
        ),
        DeclareLaunchArgument('dynamics_config', default_value=default_dynamics_parameters),
        DeclareLaunchArgument('controller_config', default_value=default_controller_parameters),
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
            parameters=[controller_parameters, {'setpoint_source': setpoint_source}],
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
