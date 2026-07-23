import math

import pytest

from tools.live_monitor_metrics import (
    OnlineExtrema, format_duration, obstacle_boxes, quaternion_yaw, safety_clearance,
    vector_error, wrapped_error)


def test_quaternion_yaw_normalizes_and_rejects_zero_quaternion():
    yaw = math.radians(177.0)
    assert quaternion_yaw((0.0, 0.0, 2.0 * math.sin(yaw / 2.0),
                           2.0 * math.cos(yaw / 2.0))) == pytest.approx(yaw)
    assert quaternion_yaw((0.0, 0.0, 0.0, 0.0)) is None


def test_vector_and_wrapped_errors_match_recorder_semantics():
    result = vector_error((1.0, 2.0, 3.0), (4.0, 6.0, 3.0))
    assert result["vector"] == (3.0, 4.0, 0.0)
    assert result["distance"] == pytest.approx(5.0)
    assert result["horizontal_distance"] == pytest.approx(5.0)
    assert wrapped_error(-math.pi + 0.1, math.pi - 0.1) == pytest.approx(0.2)


def test_safety_clearance_is_raw_distance_minus_radius():
    boxes = obstacle_boxes([2.0, 0.0, 1.0, 2.0, 2.0, 2.0])
    raw, clearance = safety_clearance((0.0, 0.0, 1.0), boxes, 0.25)
    assert raw == pytest.approx(1.0)
    assert clearance == pytest.approx(0.75)
    assert safety_clearance((0.0, 0.0, 1.0), [], 0.25) is None


def test_duration_formatting():
    assert format_duration(None) == "--"
    assert format_duration(4.25) == "  4.2 s"
    assert format_duration(65.5) == "01:05.5"


def test_online_extrema_accumulates_maxima_minimum_and_saturation():
    metrics = OnlineExtrema()
    metrics.observe(
        goal_error=2.0, horizontal_error=1.5, tracking_error=0.04,
        yaw_error=-0.2, speed=0.7, safety_clearance_m=0.8,
        force_magnitude=0.3, saturation=(False, True, False, False))
    metrics.observe(
        goal_error=1.0, horizontal_error=1.8, tracking_error=0.02,
        yaw_error=0.3, speed=0.5, safety_clearance_m=0.4,
        force_magnitude=0.0, saturation=(True, False, False, False))
    result = metrics.snapshot()
    assert result["sample_count"] == 2
    assert result["maximum_goal_error"] == pytest.approx(2.0)
    assert result["maximum_horizontal_error"] == pytest.approx(1.8)
    assert result["maximum_tracking_error"] == pytest.approx(0.04)
    assert result["maximum_absolute_yaw_error"] == pytest.approx(0.3)
    assert result["maximum_speed"] == pytest.approx(0.7)
    assert result["minimum_safety_clearance"] == pytest.approx(0.4)
    assert result["maximum_force"] == pytest.approx(0.3)
    assert result["saturation_observed"] == (True, True, False, False)
