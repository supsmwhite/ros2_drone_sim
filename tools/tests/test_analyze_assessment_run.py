import json
import math
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from assessment_metrics import (directional_disturbance_metrics, goal_timing,
    held_condition_start, longest_true_duration, navigation_phase_start,
    path_length, phased_tracking_metrics, point_box_distance,
    projection_overshoot, require_nonnegative_mission_times, wrap_to_pi,
    yaw_error_metrics)
from analyze_assessment_run import (commanded_rpm, derived_paths,
    protocol_checks, saturation_timeline)


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


def test_commanded_rpm_reads_new_and_legacy_schema():
    assert commanded_rpm({"commanded_motor_rpm_m1": 123}, 1) == 123
    assert commanded_rpm({"m1_rpm": 456}, 1) == 456


def test_saturation_count_uses_diagnostics_callbacks(tmp_path):
    (tmp_path / "diagnostics.csv").write_text(
        "recording_time_s,mission_time_s,horizontal_saturated,altitude_saturated,attitude_saturated,mixer_saturated,any_saturated\n"
        "0,0,1,0,0,0,1\n1,1,0,0,0,0,0\n")
    odom_rows = [{"mission_time_s": value, "horizontal_saturated": 1,
                  "altitude_saturated": 0, "attitude_saturated": 0,
                  "mixer_saturated": 0} for value in (0, .1, .2, .3)]
    times, flags, source = saturation_timeline(tmp_path, odom_rows)
    assert times == [0, 1] and flags == [True, False]
    assert source == "diagnostics_callbacks"


def passing_metrics(experiment):
    metrics = {"non_finite_attitude_count": 0, "non_finite_rpm_count": 0,
               "attitude_divergence_detected": False, "saturated_at_end": False,
               "final_position_error_m": .01, "final_speed_m_s": .01,
               "recorded_targets": [{"position": [1, 2, 3], "yaw_rad": 0.0}]}
    if experiment == "multi_goal":
        metrics.update({"goal_count": 2, "goal_order": [0, 1],
                        "mission_complete": True,
                        "goal_activation_times_s": [0, 2],
                        "per_goal_arrival_times_s": [2, 4],
                        "per_goal_duration_s": [2, 2]})
    if experiment in ("navigation", "static_avoidance", "narrow_corridor"):
        metrics.update({"navigation_complete": True, "navigation_success": True,
                        "collision_observed": False,
                        "navigation_tracking_max_error_m": .049,
                        "minimum_safety_clearance_m": .085,
                        "saturation_sample_count": 0})
    return metrics


@pytest.mark.parametrize("experiment", [
    "hover", "single_goal", "multi_goal", "static_avoidance", "narrow_corridor"])
def test_five_formal_scenarios_pass(experiment):
    stop = "arrival_and_steady_window_complete" if experiment in ("hover", "single_goal") else "completed"
    checks, overall, reasons = protocol_checks(
        experiment, passing_metrics(experiment), {"stop_reason": stop}, True)
    assert overall and not reasons
    assert all(set(("metric_name", "actual_value", "threshold", "passed", "source")) <= set(item) for item in checks.values())


@pytest.mark.parametrize("experiment", [
    "hover", "single_goal", "multi_goal", "static_avoidance", "narrow_corridor"])
def test_five_formal_scenarios_fail_with_reason(experiment):
    metrics = passing_metrics(experiment); metrics["non_finite_rpm_count"] = 1
    stop = "arrival_and_steady_window_complete" if experiment in ("hover", "single_goal") else "completed"
    checks, overall, reasons = protocol_checks(experiment, metrics, {"stop_reason": stop}, True)
    assert not overall and not checks["finite_commanded_rpm"]["passed"]
    assert any(reason.startswith("finite_commanded_rpm:") for reason in reasons)


def test_strict_boundaries_fail_and_produce_reasons():
    metrics = passing_metrics("hover"); metrics["final_position_error_m"] = .10; metrics["final_speed_m_s"] = .08
    checks, overall, reasons = protocol_checks(
        "hover", metrics, {"stop_reason": "arrival_and_steady_window_complete"}, True)
    assert not overall and not checks["final_position_error"]["passed"] and not checks["final_speed"]["passed"]
    assert any(reason.startswith("final_position_error:") for reason in reasons)


def test_navigation_strict_threshold_boundaries():
    metrics = passing_metrics("navigation")
    metrics.update({"navigation_tracking_max_error_m": .05,
                    "final_position_error_m": .05, "final_speed_m_s": .03})
    checks, overall, _ = protocol_checks("navigation", metrics, {"stop_reason": "completed"}, True)
    assert checks["minimum_safety_clearance"]["passed"]
    assert not checks["navigation_tracking_max_error"]["passed"]
    assert not checks["final_position_error"]["passed"]
    assert not checks["final_speed"]["passed"] and not overall


def test_yaw_error_wraps_across_pi_boundary():
    metrics = yaw_error_metrics([math.pi - .01], [-math.pi + .01])
    assert metrics["final_error_rad"] == pytest.approx(.02)
    assert wrap_to_pi(-2 * math.pi + .02) == pytest.approx(.02)


def test_legacy_missing_yaw_is_unavailable_not_failure():
    metrics = yaw_error_metrics([0.1], [math.nan])
    assert metrics["status"] == "unavailable"
    assert metrics["final_error_rad"] is None


def test_output_protection_tracks_yaw_figure(tmp_path):
    paths = derived_paths(tmp_path, "hover")
    assert tmp_path / "summary.json" in paths
    assert tmp_path / "yaw_tracking.png" in paths
