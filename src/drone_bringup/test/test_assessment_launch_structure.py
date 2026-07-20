import ast
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


LAUNCH = Path(__file__).resolve().parents[1] / 'launch'
ASSESSMENT_LAUNCHES = {
    'assessment_basic_sim.launch.py': 'mission_sim.launch.py',
    'assessment_navigation_sim.launch.py': 'interactive_goal_navigation_sim.launch.py',
    'assessment_disturbance_sim.launch.py': 'disturbance_visual_demo.launch.py',
}


def _load_launch(name):
    path = LAUNCH / name
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda _package: str(LAUNCH.parent)
    return module, module.generate_launch_description()


def _declared_defaults(description):
    context = LaunchContext()
    return {
        action.name: ''.join(value.perform(context) for value in action.default_value)
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }


def test_assessment_launches_parse_and_load():
    for name in ASSESSMENT_LAUNCHES:
        source = (LAUNCH / name).read_text(encoding='utf-8')
        ast.parse(source, filename=name)
        _, description = _load_launch(name)
        assert description.entities


def test_assessment_launches_are_thin_includes():
    forbidden_node_symbols = {'Node', 'ComposableNode', 'ComposableNodeContainer'}
    for name, reused_launch in ASSESSMENT_LAUNCHES.items():
        source = (LAUNCH / name).read_text(encoding='utf-8')
        tree = ast.parse(source, filename=name)
        called_names = {
            node.func.id for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        }
        assert forbidden_node_symbols.isdisjoint(called_names), name
        assert reused_launch in source, name


def test_basic_waits_for_runtime_single_or_multi_input():
    _, description = _load_launch('assessment_basic_sim.launch.py')
    includes = [
        action for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
    ]
    assert len(includes) == 1
    arguments = dict(includes[0].launch_arguments)
    assert arguments['start_with_configured_waypoints'] == 'false'
    assert 'mission_config' not in arguments


def test_navigation_public_defaults_and_forwarded_arguments():
    _, description = _load_launch('assessment_navigation_sim.launch.py')
    defaults = _declared_defaults(description)
    assert defaults == {'yaw_mode': 'path_tangent', 'use_rviz': 'true'}
    includes = [
        action for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
    ]
    assert len(includes) == 1
    include = includes[0]
    arguments = dict(include.launch_arguments)
    assert set(arguments) == {'use_rviz', 'yaw_mode'}
    assert isinstance(arguments['use_rviz'], LaunchConfiguration)
    assert isinstance(arguments['yaw_mode'], LaunchConfiguration)


def test_internal_navigation_uses_one_environment_yaml_for_all_consumers():
    _, description = _load_launch('interactive_goal_navigation_sim.launch.py')
    context = LaunchContext()
    environment_consumers = []
    for action in description.entities:
        if not isinstance(action, Node):
            continue
        parameters = action._Node__parameters
        if parameters:
            parameter_path = parameters[0]._ParameterFile__param_file
            resolved_path = perform_substitutions(context, parameter_path)
            if Path(resolved_path).name == 'environment.yaml':
                environment_consumers.append(resolved_path)
    assert len(environment_consumers) == 3
    assert len(set(environment_consumers)) == 1
    assert Path(environment_consumers[0]).is_file()


def test_disturbance_public_default_and_profiles():
    _, description = _load_launch('assessment_disturbance_sim.launch.py')
    defaults = _declared_defaults(description)
    assert defaults['profile'] == 'short_gust'
    source = (LAUNCH / 'disturbance_visual_demo.launch.py').read_text(encoding='utf-8')
    assert "'short_gust'" in source
    assert "'persistent_release'" in source


def test_assessment_entrypoints_do_not_duplicate_runtime_nodes_or_topics():
    combined = '\n'.join(
        (LAUNCH / name).read_text(encoding='utf-8') for name in ASSESSMENT_LAUNCHES)
    assert 'executable=' not in combined
    assert 'name=' not in combined
    assert '/drone/' not in combined
    assert 'environment.yaml' not in combined
    assert '.yaml' not in combined
