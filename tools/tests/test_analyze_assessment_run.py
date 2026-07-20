import json
import math
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from assessment_metrics import (directional_disturbance_metrics, goal_timing,
    held_condition_start, longest_true_duration, navigation_phase_start,
    path_length, phased_tracking_metrics, point_box_distance,
    projection_overshoot, require_nonnegative_mission_times)


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


def test_arrival_time_is_start_of_held_interval():
    assert held_condition_start([0, .5, 1, 1.5, 2], [False, True, True, True, True], 1) == .5


def test_json_rejects_nan_and_infinity():
    with pytest.raises(ValueError): json.dumps({"x": math.nan}, allow_nan=False)
    with pytest.raises(ValueError): json.dumps({"x": math.inf}, allow_nan=False)


def test_navigation_phase_prefers_first_valid_reference_segment():
    paths = {"reference_segments": [
        {"mission_time_s": None, "goal_index": 7, "points": [[0, 0, 0]]},
        {"mission_time_s": 4.5, "goal_index": 0, "points": [[0, 0, 0]]}],
        "planned_segments": [{"mission_time_s": 3.0, "goal_index": 0, "points": [[0, 0, 0]]}]}
    assert navigation_phase_start(paths) == (4.5, "reference_segment")


def test_navigation_phase_falls_back_to_planned():
    paths = {"reference_segments": [], "planned_segments": [
        {"mission_time_s": 3.0, "goal_index": 0, "points": [[0, 0, 0]]}]}
    assert navigation_phase_start(paths) == (3.0, "planned_segment")


def test_pre_mission_path_is_not_navigation_phase():
    paths = {"reference_segments": [
        {"mission_time_s": None, "goal_index": 0, "points": [[0, 0, 0]]}]}
    assert navigation_phase_start(paths) == (None, None)


def test_tracking_phases_have_independent_max_and_rms():
    full, takeoff, navigation = phased_tracking_metrics([(0, 1.5), (1, .5), (2, .1), (3, .2)], 2)
    assert full["max_error_m"] == 1.5
    assert takeoff["rms_error_m"] == pytest.approx(math.sqrt(1.25))
    assert navigation["max_error_m"] == .2
    assert navigation["rms_error_m"] == pytest.approx(math.sqrt(.025))
    assert navigation["final_error_m"] == .2


def test_missing_tracking_phase_returns_nulls():
    _, takeoff, navigation = phased_tracking_metrics([(2, .1)], None)
    assert takeoff["sample_count"] is None and navigation["max_error_m"] is None


def test_four_goal_activation_arrival_and_duration():
    events = [{"goal_index": i, "mission_time_s": value}
              for i, value in enumerate((0, 3, 8, 12))]
    activation, arrival, duration = goal_timing(4, events, 18)
    assert activation == [0, 3, 8, 12]
    assert arrival == [3, 8, 12, 18]
    assert duration == [3, 5, 4, 6]


def test_goal_timing_does_not_invent_first_activation():
    activation, arrival, duration = goal_timing(2, [{"goal_index": 1, "mission_time_s": 5}], 9)
    assert activation == [None, 5] and arrival == [5, 9] and duration == [None, 4]


def disturbance_sample(x, y, fx, fy, active):
    return {"actual": [x, y], "goal": [0, 0], "force": [fx, fy],
            "force_active": active}


def test_reverse_overshoot_positive_x():
    samples = [disturbance_sample(0, 0, .3, 0, True),
               disturbance_sample(.3, 0, .3, 0, True),
               disturbance_sample(.1, 0, 0, 0, False),
               disturbance_sample(-.08, 0, 0, 0, False)]
    force, peak, reverse, _ = directional_disturbance_metrics(samples)
    assert force == pytest.approx([.3, 0]); assert peak == pytest.approx(.3); assert reverse == pytest.approx(.08)


def test_reverse_overshoot_negative_y_direction():
    samples = [disturbance_sample(0, -.2, 0, -.4, True),
               disturbance_sample(0, .06, 0, 0, False)]
    _, peak, reverse, _ = directional_disturbance_metrics(samples)
    assert peak == pytest.approx(.2) and reverse == pytest.approx(.06)


def test_no_goal_crossing_has_zero_reverse_overshoot():
    samples = [disturbance_sample(.3, 0, .3, 0, True),
               disturbance_sample(.02, 0, 0, 0, False)]
    assert directional_disturbance_metrics(samples)[2] == 0.0


def test_near_zero_horizontal_force_has_null_direction_metrics():
    samples = [disturbance_sample(.1, 0, 1e-8, 0, True),
               disturbance_sample(0, 0, 0, 0, False)]
    assert directional_disturbance_metrics(samples)[:3] == (None, None, None)


def test_negative_mission_time_is_rejected():
    with pytest.raises(ValueError): require_nonnegative_mission_times([None, 0, .1, -.01])
