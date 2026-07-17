import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('drone_bringup')
    dynamics = os.path.join(bringup, 'config', 'dynamics.yaml')
    controller = os.path.join(bringup, 'config', 'controller.yaml')
    xacro_file = os.path.join(bringup, 'urdf', 'drone.urdf.xacro')
    rviz_config = os.path.join(bringup, 'rviz', 'drone_sim.rviz')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    use_rviz = LaunchConfiguration('use_rviz')
    hover_goal = (
        "{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 1.5}, "
        "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}")

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2 for the hover disturbance experiment.'),
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
        TimerAction(
            period=1.0,
            actions=[ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once', '/drone/goal',
                    'geometry_msgs/msg/PoseStamped', hover_goal,
                ],
                output='screen')]),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_config], condition=IfCondition(use_rviz)),
    ])
