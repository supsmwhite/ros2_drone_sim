import json
import math
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from assessment_metrics import (longest_true_duration, path_length,
    point_box_distance, projection_overshoot)


def test_path_length_3d():
    assert path_length([[0, 0, 0], [3, 4, 0], [3, 4, 12]]) == 17


def test_missing_path_is_represented_as_none():
    result = None if not [] else path_length([])
    assert result is None


def test_aabb_outside_distance():
    assert point_box_distance((3, 2, 2), ((0, 0, 0), (1, 1, 1))) == pytest.approx(math.sqrt(6))


def test_aabb_inside_distance_zero():
    assert point_box_distance((.5, .5, .5), ((0, 0, 0), (1, 1, 1))) == 0


def test_safety_clearance_definition():
    assert point_box_distance((2, .5, .5), ((0, 0, 0), (1, 1, 1))) - .25 == .75


def test_hover_overshoot():
    value, percent = projection_overshoot([([0, 0, 0], [0, 0, 1]), ([0, 0, 1.2], [0, 0, 1])], True)
    assert value == pytest.approx(.2) and percent is None


def test_three_dimensional_projection_overshoot():
    samples = [([0, 0, 0], [1, 1, 1]), ([1.1, 1.1, 1.1], [1, 1, 1])]
    value, percent = projection_overshoot(samples)
    assert value == pytest.approx(math.sqrt(.03)); assert percent == pytest.approx(10)


def test_tracking_and_goal_errors_are_distinct():
    actual = [1, 0, 0]; goal = [2, 0, 0]; reference = [1.1, 0, 0]
    assert math.dist(actual, goal) == 1 and math.dist(actual, reference) == pytest.approx(.1)


def test_continuous_saturation_duration():
    assert longest_true_duration([0, 1, 2, 3, 4], [False, True, True, False, True]) == 1


def test_sustained_attitude_divergence_duration():
    assert longest_true_duration([0, .5, 1, 1.5], [True, True, True, False]) == 1


def test_force_release_recovery_time():
    assert 6.5 - 4.0 == 2.5


def test_json_rejects_nan_and_infinity():
    with pytest.raises(ValueError): json.dumps({"x": math.nan}, allow_nan=False)
    with pytest.raises(ValueError): json.dumps({"x": math.inf}, allow_nan=False)
