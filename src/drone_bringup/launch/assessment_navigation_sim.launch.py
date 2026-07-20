import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


SUPPORTED_SCENARIOS = ('obstacle_field', 'narrow_passage')


def resolve_scenario_config(scenario, share):
    filenames = {
        'obstacle_field': 'environment.yaml',
        'narrow_passage': 'environment_narrow_passage.yaml',
    }
    try:
        return os.path.join(share, 'config', filenames[scenario])
    except KeyError as error:
        choices = ', '.join(SUPPORTED_SCENARIOS)
        raise ValueError(
            f"Unsupported assessment scenario '{scenario}'; choose: {choices}") from error


def _include_navigation(context):
    scenario = LaunchConfiguration('scenario').perform(context)
    share = get_package_share_directory('drone_bringup')
    environment_config = resolve_scenario_config(scenario, share)
    navigation = os.path.join(
        share, 'launch', 'interactive_goal_navigation_sim.launch.py')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'yaw_mode': LaunchConfiguration('yaw_mode'),
            'environment_config': environment_config,
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='obstacle_field',
            description='Assessment map view: obstacle_field or narrow_passage.'),
        DeclareLaunchArgument(
            'yaw_mode', default_value='path_tangent',
            description='Yaw reference mode forwarded to the navigation chain.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2 for interactive navigation.'),
        OpaqueFunction(function=_include_navigation),
    ])
