import ast
from pathlib import Path


LAUNCH = Path(__file__).resolve().parents[1] / 'launch'


def test_modified_launch_files_are_valid_python():
    names = [
        'simulation_core.launch.py', 'basic_sim.launch.py', 'mission_sim.launch.py',
        'static_avoidance_sim.launch.py', 'multi_goal_static_avoidance_sim.launch.py',
        'interactive_goal_navigation_sim.launch.py',
    ]
    for name in names:
        ast.parse((LAUNCH / name).read_text(encoding='utf-8'), filename=name)


def test_public_scenarios_reuse_simulation_core():
    for name in [
            'basic_sim.launch.py', 'static_avoidance_sim.launch.py',
            'multi_goal_static_avoidance_sim.launch.py',
            'interactive_goal_navigation_sim.launch.py']:
        content = (LAUNCH / name).read_text(encoding='utf-8')
        assert 'simulation_core.launch.py' in content, name
