import ast
import importlib.util
from pathlib import Path

import yaml
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
INTERNAL_LAUNCH_DEPENDENCIES = {
    'basic_sim.launch.py': 'simulation_core.launch.py',
    'mission_sim.launch.py': 'basic_sim.launch.py',
    'interactive_goal_navigation_sim.launch.py': 'simulation_core.launch.py',
}
RETAINED_INTERNAL_LAUNCHES = {
    'simulation_core.launch.py', 'basic_sim.launch.py', 'mission_sim.launch.py',
    'interactive_goal_navigation_sim.launch.py',
    'disturbance_visual_demo.launch.py',
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


def _node_launch_configuration_overrides(description, node_name):
    context = LaunchContext()
    nodes = [
        action for action in description.entities
        if isinstance(action, Node) and action._Node__node_name == node_name
    ]
    assert len(nodes) == 1
    overrides = nodes[0]._Node__parameters[-1]
    return {
        perform_substitutions(context, key): perform_substitutions(
            context, value[0].variable_name)
        for key, value in overrides.items()
        if isinstance(value, tuple) and len(value) == 1 and
        isinstance(value[0], LaunchConfiguration)
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


def test_retained_internal_launch_dependencies_exist_and_reuse_core():
    for name in RETAINED_INTERNAL_LAUNCHES:
        path = LAUNCH / name
        assert path.is_file(), name
        ast.parse(path.read_text(encoding='utf-8'), filename=name)
    for name, dependency in INTERNAL_LAUNCH_DEPENDENCIES.items():
        source = (LAUNCH / name).read_text(encoding='utf-8')
        assert dependency in source, name


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
    assert defaults == {
        'yaw_mode': 'path_tangent',
        'use_rviz': 'true',
        'nominal_speed': '0.50',
        'max_reference_speed': '0.90',
        'max_reference_acceleration': '0.60',
    }
    includes = [
        action for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
    ]
    assert len(includes) == 1
    include = includes[0]
    arguments = dict(include.launch_arguments)
    assert set(arguments) == set(defaults)
    for name, argument in arguments.items():
        assert isinstance(argument, LaunchConfiguration)
        assert perform_substitutions(LaunchContext(), argument.variable_name) == name


def test_internal_navigation_speed_defaults_and_node_overrides_match():
    _, description = _load_launch('interactive_goal_navigation_sim.launch.py')
    defaults = _declared_defaults(description)
    expected_defaults = {
        'nominal_speed': '0.50',
        'max_reference_speed': '0.90',
        'max_reference_acceleration': '0.60',
    }
    assert {key: defaults[key] for key in expected_defaults} == expected_defaults

    expected_overrides = {key: key for key in expected_defaults}
    assert _node_launch_configuration_overrides(
        description, 'interactive_goal_editor_node') == expected_overrides
    executor_overrides = _node_launch_configuration_overrides(
        description, 'multi_goal_static_avoidance_node')
    assert {
        key: executor_overrides[key] for key in expected_overrides
    } == expected_overrides


def test_formal_navigation_yaml_and_snapshot_use_a2_defaults():
    expected = {
        'nominal_speed': 0.50,
        'max_reference_speed': 0.90,
        'max_reference_acceleration': 0.60,
    }
    repository = LAUNCH.parents[2]
    paths = [
        LAUNCH.parent / 'config' / 'planned_trajectory.yaml',
        repository / 'results' / 'parameters' / 'planned_trajectory.yaml',
    ]
    for path in paths:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
        parameters = next(iter(data.values()))['ros__parameters']
        assert {key: parameters[key] for key in expected} == expected


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


def test_only_disturbance_internal_launch_enables_external_wrench():
    enabled = []
    for path in LAUNCH.glob('*.launch.py'):
        if "'enable_external_wrench': True" in path.read_text(encoding='utf-8'):
            enabled.append(path.name)
    assert enabled == ['disturbance_visual_demo.launch.py']


def test_assessment_entrypoints_do_not_duplicate_runtime_nodes_or_topics():
    combined = '\n'.join(
        (LAUNCH / name).read_text(encoding='utf-8') for name in ASSESSMENT_LAUNCHES)
    assert 'executable=' not in combined
    assert 'name=' not in combined
    assert '/drone/' not in combined
    assert 'environment.yaml' not in combined
    assert '.yaml' not in combined
