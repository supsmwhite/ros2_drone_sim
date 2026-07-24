import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('drone_bringup')
    dynamics_default = os.path.join(share, 'config', 'dynamics.yaml')
    controller_default = os.path.join(share, 'config', 'controller.yaml')
    xacro_file = os.path.join(share, 'urdf', 'drone.urdf.xacro')
    rviz_config = os.path.join(share, 'rviz', 'drone_sim.rviz')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('setpoint_source', default_value='pose_goal'),
        DeclareLaunchArgument('dynamics_config', default_value=dynamics_default),
        DeclareLaunchArgument('controller_config', default_value=controller_default),
        DeclareLaunchArgument('max_horizontal_acceleration', default_value='1.12'),
        DeclareLaunchArgument('max_tilt_angle', default_value='0.15'),
        Node(
            package='drone_dynamics', executable='quadrotor_dynamics_node',
            name='quadrotor_dynamics_node', output='screen',
            parameters=[LaunchConfiguration('dynamics_config')]),
        Node(
            package='drone_controller', executable='position_controller_node',
            name='position_controller_node', output='screen',
            parameters=[LaunchConfiguration('controller_config'), {
                'setpoint_source': LaunchConfiguration('setpoint_source'),
                'max_horizontal_acceleration': LaunchConfiguration(
                    'max_horizontal_acceleration'),
                'max_tilt_angle': LaunchConfiguration('max_tilt_angle')}]),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description}]),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
