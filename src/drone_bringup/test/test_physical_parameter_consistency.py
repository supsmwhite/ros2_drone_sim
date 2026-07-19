import math
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def parameters(path, node_name):
    with path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    try:
        return document[node_name]['ros__parameters']
    except (KeyError, TypeError) as error:
        raise AssertionError(
            f'{path}: missing {node_name}.ros__parameters') from error


def test_dynamics_and_controller_physical_parameters_match():
    dynamics = parameters(
        PACKAGE_ROOT / 'config' / 'dynamics.yaml', 'quadrotor_dynamics_node')
    controller = parameters(
        PACKAGE_ROOT / 'config' / 'controller.yaml', 'position_controller_node')
    mapping = {
        'mass': ('mass', 'mass'),
        'gravity': ('gravity', 'gravity'),
        'arm length': ('arm_length', 'arm_length'),
        'thrust coefficient': ('thrust_coefficient', 'thrust_coefficient'),
        'torque coefficient': (
            'drag_torque_coefficient', 'drag_torque_coefficient'),
        'minimum RPM': ('min_rpm', 'min_rpm'),
        'maximum RPM': ('max_rpm', 'max_rpm'),
    }
    failures = []
    for name, (dynamics_key, controller_key) in mapping.items():
        if dynamics_key not in dynamics:
            failures.append(f'{name}: dynamics key {dynamics_key!r} is missing')
            continue
        if controller_key not in controller:
            failures.append(f'{name}: controller key {controller_key!r} is missing')
            continue
        dynamics_value = dynamics[dynamics_key]
        controller_value = controller[controller_key]
        if not (isinstance(dynamics_value, (int, float)) and
                isinstance(controller_value, (int, float)) and
                math.isfinite(dynamics_value) and math.isfinite(controller_value)):
            failures.append(
                f'{name}: non-finite/non-numeric value; dynamics={dynamics_value!r}, '
                f'controller={controller_value!r}')
        elif not math.isclose(
                dynamics_value, controller_value, rel_tol=1.0e-9, abs_tol=1.0e-12):
            failures.append(
                f'{name}: dynamics={dynamics_value!r}, controller={controller_value!r}')
    assert not failures, 'Physical parameter mismatch:\n' + '\n'.join(failures)
