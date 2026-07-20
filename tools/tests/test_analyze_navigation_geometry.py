import math
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analyze_navigation_geometry import (  # noqa: E402
    clearance_losses, clearance_profile, cross_track_errors, derivative,
    finite_points, identify_corners, identify_segment_corners, nearest_point_on_polyline,
    point_aabb_distance, project_points_on_polyline, resample_polyline, turning_angle_degrees,
    unwrap_angles)


BOX = ((0.9, -0.1, -0.1), (1.1, 0.1, 0.1))


def test_straight_path_resampling_uses_fixed_arc_step_and_endpoints():
    result = resample_polyline([[0, 0, 0], [0.05, 0, 0]], 0.02)
    assert result == [[0.0, 0.0, 0.0], [0.02, 0.0, 0.0],
                      [0.04, 0.0, 0.0], [0.05, 0.0, 0.0]]


def test_polyline_corner_is_identified():
    corners = identify_corners([[0, 0, 0], [1, 0, 0], [1, 1, 0]])
    assert len(corners) == 1
    assert corners[0]["angle_deg"] == 90.0


def test_straight_polyline_has_no_false_corner():
    assert identify_corners([[0, 0, 0], [1, 0, 0], [2, 0, 0]]) == []


def test_stop_and_replan_join_is_not_a_geometric_corner():
    paths = {"simplified_segments": [
        {"sequence": 0, "goal_index": 0,
         "points": [[0, 0, 0], [1, 0, 0]]},
        {"sequence": 1, "goal_index": 1,
         "points": [[0.99, 0, 0], [1, 1, 0]]},
    ]}
    assert identify_segment_corners(paths) == []


def test_point_to_aabb_distance_and_clearance():
    assert point_aabb_distance([1.4, 0.1, 0.1], BOX) == pytest_approx(0.3)
    profile = clearance_profile([[1.4, 0, 0]], [BOX], 0.25)
    assert profile[0]["safety_clearance_m"] == pytest_approx(0.05)


def test_resampling_finds_segment_interior_minimum_clearance():
    profile = clearance_profile([[0, 0.3, 0], [2, 0.3, 0]], [BOX], 0.1, 0.02)
    assert min(row["safety_clearance_m"] for row in profile) == pytest_approx(0.1)
    assert profile[0]["safety_clearance_m"] > 0.8
    assert profile[-1]["safety_clearance_m"] > 0.8


def test_polyline_nearest_point_and_cross_track_error():
    distance, point, arc, segment = nearest_point_on_polyline(
        [1, 1, 0], [[0, 0, 0], [2, 0, 0]])
    assert distance == 1.0
    assert point == [1.0, 0.0, 0.0]
    assert arc == 1.0 and segment == 0
    assert cross_track_errors([[1, 1, 0]], [[0, 0, 0], [2, 0, 0]]) == [1.0]
    projection = project_points_on_polyline(
        [[1, 1, 0], [3, 0, 0]], [[0, 0, 0], [2, 0, 0]])
    assert projection[0] == (1.0, [1.0, 0.0, 0.0], 1.0, 0)
    assert projection[1] == (1.0, [2.0, 0.0, 0.0], 2.0, 0)


def test_planned_to_simplified_clearance_loss():
    values = summaries(planned=0.20, simplified=0.14, reference=0.14, actual=0.14)
    assert clearance_losses(values)["simplified_clearance_loss_m"] == pytest_approx(0.06)


def test_reference_to_actual_clearance_loss():
    values = summaries(planned=0.20, simplified=0.20, reference=0.18, actual=0.10)
    assert clearance_losses(values)["actual_clearance_loss_m"] == pytest_approx(0.08)


def test_empty_path_handling():
    assert resample_polyline([]) == []
    assert clearance_profile([], [BOX], 0.25) == []
    assert nearest_point_on_polyline([0, 0, 0], [])[1:] == (None, math.nan, None)
    values = summaries(planned=None, simplified=None, reference=None, actual=None)
    assert all(value is None for value in clearance_losses(values).values())


def test_non_finite_points_are_filtered():
    assert finite_points([[0, 0, 0], [math.nan, 1, 2], [1, math.inf, 2], [1, 2, 3]]) == [
        [0.0, 0.0, 0.0], [1.0, 2.0, 3.0]]


def test_turning_angle_is_stable_near_zero_and_180_degrees():
    near_zero = turning_angle_degrees([0, 0, 0], [1, 0, 0], [2, 1e-14, 0])
    near_180 = turning_angle_degrees([0, 0, 0], [1, 0, 0], [0, 1e-14, 0])
    assert 0.0 <= near_zero < 1e-5
    assert 179.99999 < near_180 <= 180.0


def test_jerk_difference_filters_bad_timestamps():
    times = [0.0, 0.02, 0.020001, 0.5, 0.52]
    acceleration = [[0, 0, 0], [0.02, 0, 0], [10, 0, 0], [20, 0, 0], [20.02, 0, 0]]
    result = derivative(times, acceleration)
    assert result[1] == [1.0, 0.0, 0.0]
    assert result[2] is None
    assert result[3] is None
    assert result[4][0] == pytest_approx(1.0)


def test_yaw_unwrap_removes_pi_boundary_jump():
    values = unwrap_angles([math.pi - 0.01, -math.pi + 0.01])
    assert values[1] - values[0] == pytest_approx(0.02)


def summaries(**values):
    return {name: {"minimum_safety_clearance_m": value}
            for name, value in values.items()}


def pytest_approx(value):
    import pytest
    return pytest.approx(value, abs=1e-9)
