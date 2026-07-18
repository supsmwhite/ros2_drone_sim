import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('drone_bringup')
    dynamics = os.path.join(bringup, 'config', 'dynamics.yaml')
    controller = os.path.join(bringup, 'config', 'controller.yaml')
    xacro_file = os.path.join(bringup, 'urdf', 'drone.urdf.xacro')
    rviz_config = os.path.join(bringup, 'rviz', 'disturbance_demo.rviz')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    profile = LaunchConfiguration('profile')
    profile_duration = PythonExpression([
        "'10.0' if '", profile, "' == 'persistent_release' else '2.0'",
    ])

    arguments = [
        DeclareLaunchArgument(
            'profile', default_value='short_gust',
            description='Demo preset: short_gust or persistent_release.'),
        DeclareLaunchArgument('target_x', default_value='0.0'),
        DeclareLaunchArgument('target_y', default_value='0.0'),
        DeclareLaunchArgument('target_z', default_value='1.5'),
        DeclareLaunchArgument('force_x', default_value='0.30'),
        DeclareLaunchArgument('force_y', default_value='0.0'),
        DeclareLaunchArgument('force_z', default_value='0.0'),
        DeclareLaunchArgument('start_delay', default_value='5.0'),
        DeclareLaunchArgument(
            'disturbance_duration', default_value=profile_duration,
            description='Force duration; defaults to 2 s or 10 s according to profile.'),
        DeclareLaunchArgument('recovery_duration', default_value='10.0'),
        DeclareLaunchArgument('force_publish_rate', default_value='25.0'),
        DeclareLaunchArgument('force_arrow_scale', default_value='1.5'),
        DeclareLaunchArgument('integral_arrow_scale', default_value='1.5'),
        DeclareLaunchArgument('show_integral_arrow', default_value='true'),
        DeclareLaunchArgument('show_status_text', default_value='true'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2 with the disturbance visualization preconfigured.'),
    ]

    demo_parameters = {
        'profile': profile,
        'target_x': ParameterValue(LaunchConfiguration('target_x'), value_type=float),
        'target_y': ParameterValue(LaunchConfiguration('target_y'), value_type=float),
        'target_z': ParameterValue(LaunchConfiguration('target_z'), value_type=float),
        'force_x': ParameterValue(LaunchConfiguration('force_x'), value_type=float),
        'force_y': ParameterValue(LaunchConfiguration('force_y'), value_type=float),
        'force_z': ParameterValue(LaunchConfiguration('force_z'), value_type=float),
        'start_delay': ParameterValue(LaunchConfiguration('start_delay'), value_type=float),
        'disturbance_duration': ParameterValue(
            LaunchConfiguration('disturbance_duration'), value_type=float),
        'recovery_duration': ParameterValue(
            LaunchConfiguration('recovery_duration'), value_type=float),
        'force_publish_rate': ParameterValue(
            LaunchConfiguration('force_publish_rate'), value_type=float),
        'force_arrow_scale': ParameterValue(
            LaunchConfiguration('force_arrow_scale'), value_type=float),
        'integral_arrow_scale': ParameterValue(
            LaunchConfiguration('integral_arrow_scale'), value_type=float),
        'show_integral_arrow': ParameterValue(
            LaunchConfiguration('show_integral_arrow'), value_type=bool),
        'show_status_text': ParameterValue(
            LaunchConfiguration('show_status_text'), value_type=bool),
    }

    nodes = [
        Node(
            package='drone_dynamics', executable='quadrotor_dynamics_node',
            name='quadrotor_dynamics_node', output='screen',
            parameters=[dynamics, {'enable_external_wrench': True}]),
        Node(
            package='drone_controller', executable='position_controller_node',
            name='position_controller_node', output='screen',
            parameters=[controller, {'setpoint_source': 'pose_goal'}]),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description}]),
        Node(
            package='drone_bringup', executable='disturbance_demo_node',
            name='disturbance_demo_node', output='screen', parameters=[demo_parameters]),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]
    return LaunchDescription(arguments + nodes)
