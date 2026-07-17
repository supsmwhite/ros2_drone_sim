import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('drone_bringup')
    dynamics = os.path.join(bringup, 'config', 'dynamics.yaml')
    controller = os.path.join(bringup, 'config', 'controller.yaml')
    environment = os.path.join(bringup, 'config', 'environment.yaml')
    astar = os.path.join(bringup, 'config', 'astar.yaml')
    trajectory = os.path.join(bringup, 'config', 'planned_trajectory.yaml')
    editor = os.path.join(bringup, 'config', 'interactive_goal_editor.yaml')
    executor = os.path.join(bringup, 'config', 'interactive_goal_executor.yaml')
    xacro_file = os.path.join(bringup, 'urdf', 'drone.urdf.xacro')
    rviz_config = os.path.join(bringup, 'rviz', 'drone_sim.rviz')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2 for interactive goal navigation.'),
        Node(
            package='drone_dynamics', executable='quadrotor_dynamics_node',
            name='quadrotor_dynamics_node', output='screen',
            parameters=[dynamics]),
        Node(
            package='drone_controller', executable='position_controller_node',
            name='position_controller_node', output='screen',
            parameters=[controller, {'setpoint_source': 'trajectory'}]),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description}]),
        Node(
            package='drone_planning', executable='static_environment_node',
            name='static_environment_node', output='screen',
            parameters=[environment]),
        Node(
            package='drone_planning', executable='interactive_goal_editor_node',
            name='interactive_goal_editor_node', output='screen',
            parameters=[
                environment, astar, trajectory, editor,
                {'execution_enabled': True},
            ]),
        Node(
            package='drone_planning', executable='multi_goal_static_avoidance_node',
            name='multi_goal_static_avoidance_node', output='screen',
            parameters=[environment, astar, trajectory, executor]),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_config], condition=IfCondition(use_rviz)),
    ])
