import json
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_navigation_speed_sweep import (  # noqa: E402
    acceleration_summary, aggregate, candidate_parameters,
    differentiated_acceleration_samples, event_data, failed_record,
    limiting_reason, select_candidates, sample_metrics, write_table)


def safe_run(candidate, run, mission_time=100.0, clearance=0.20):
    return {
        "candidate": candidate, "route": "full_map", "run": run,
        "success": True, "collision": False, "non_finite_value_count": 0,
        "minimum_safety_clearance_m": clearance, "saturated_at_end": False,
        "final_position_error_m": 0.01,
        "navigation_tracking_max_error_m": 0.04,
        "total_mission_time_s": mission_time,
    }


def test_candidate_parameter_generation():
    assert candidate_parameters("s2") == {
        "nominal_speed": 0.45,
        "max_reference_speed": 0.85,
        "max_reference_acceleration": 0.45,
    }
    assert candidate_parameters("a3") == {
        "nominal_speed": 0.55,
        "max_reference_speed": 1.0,
        "max_reference_acceleration": 0.65,
    }


def acceleration_sample(acceleration, time=1.0, position=None, segment=2, goal=3):
    return {
        "acceleration": acceleration, "mission_time_s": time,
        "position": position or [1.0, 2.0, 3.0],
        "segment_index": segment, "goal_index": goal,
    }


def test_pure_horizontal_acceleration_components():
    result = acceleration_summary([acceleration_sample([0.3, 0.4, 0.0])])
    assert result["max_horizontal"] == 0.5
    assert result["max_vertical"] == 0.0
    assert result["max_total"] == 0.5


def test_pure_vertical_acceleration_components():
    result = acceleration_summary([acceleration_sample([0.0, 0.0, -0.6])])
    assert result["max_horizontal"] == 0.0
    assert result["max_vertical"] == 0.6
    assert result["max_total"] == 0.6


def test_mixed_acceleration_components():
    result = acceleration_summary([acceleration_sample([0.3, 0.4, 1.2])])
    assert result["max_horizontal"] == 0.5
    assert result["max_vertical"] == 1.2
    assert result["max_total"] == 1.3


def test_nonfinite_acceleration_is_excluded():
    result = acceleration_summary([
        acceleration_sample([float("nan"), 0.0, 0.0]),
        acceleration_sample([0.0, float("inf"), 0.0]),
        acceleration_sample([0.1, 0.0, 0.0]),
    ])
    assert result["max_total"] == 0.1


def test_empty_acceleration_samples_are_null():
    assert acceleration_summary([]) == {
        "max_horizontal": None, "max_vertical": None,
        "max_total": None, "peak": None,
    }


def test_acceleration_peak_retains_time_position_and_indices():
    result = acceleration_summary([
        acceleration_sample([0.1, 0.0, 0.0]),
        acceleration_sample([0.0, 0.8, 0.0], time=4.2,
                            position=[4.0, 5.0, 6.0], segment=7, goal=1),
    ])
    assert result["peak"] == {
        "mission_time_s": 4.2, "trajectory_segment_index": 7,
        "goal_index": 1, "position": [4.0, 5.0, 6.0],
        "horizontal_acceleration_m_s2": 0.8,
        "vertical_acceleration_m_s2": 0.0,
        "total_acceleration_m_s2": 0.8,
    }


def test_centered_velocity_difference_filters_tiny_dt():
    samples = [
        {"mission_time_s": 0.0, "velocity": [0.0, 0.0, 0.0]},
        {"mission_time_s": 0.01, "velocity": [0.01, 0.0, 0.0]},
        {"mission_time_s": 0.02, "velocity": [0.02, 0.0, 0.0]},
    ]
    result = differentiated_acceleration_samples(samples)
    assert result[0]["acceleration"] == [1.0, 0.0, 0.0]
    samples[2]["mission_time_s"] = 0.00001
    assert differentiated_acceleration_samples(samples) == []


def test_scaled_horizontal_trajectory_reports_acceleration_limit():
    segment = {
        "duration_scale": 1.2, "reference_max_speed_m_s": 0.6,
        "trajectory_duration_s": 10.0, "refinement_iterations": 0,
        "simplified_point_count": 3,
    }
    summary = acceleration_summary([acceleration_sample([0.48, 0.0, 0.0])])
    parameters = {"max_reference_speed": 0.9,
                  "max_reference_acceleration": 0.5}
    assert limiting_reason(segment, summary, parameters) == (
        "horizontal_acceleration_dominant")


def test_failed_run_is_structured():
    row = failed_record("s1", "full_map", 1, candidate_parameters("s1"),
                        "timeout", "abc", False, 42)
    assert row["success"] is False
    assert row["stop_reason"] == "timeout"
    assert row["repository_commit"] == "abc"


def test_events_before_mission_are_ignored(tmp_path):
    path = tmp_path / "events.csv"
    path.write_text(
        "recording_time_s,mission_time_s,event,details\n"
        '0.0,,recording_started,"{}"\n'
        '1.0,0.5,navigation_complete_changed,"{\"\"value\"\":true}"\n')
    assert event_data(path) == [{
        "event": "navigation_complete_changed", "time": 0.5,
        "details": {"value": True},
    }]


def test_actual_speed_statistics_only_use_navigation_phase(tmp_path):
    path = tmp_path / "samples.csv"
    path.write_text("mission_time_s,speed\n1.0,1.2\n2.0,0.4\n3.0,0.6\n")
    metrics, nonfinite = sample_metrics(path, 2.0)
    assert metrics["actual_max_speed_m_s"] == 0.6
    assert metrics["actual_mean_speed_m_s"] == 0.5
    assert nonfinite == 0


def test_csv_and_three_run_statistics(tmp_path):
    rows = [safe_run("baseline", index, 100.0 + index) for index in (1, 2, 3)]
    stats = aggregate(rows)[0]
    assert stats["runs"] == 3
    assert stats["mission_time_mean_s"] == 102.0
    assert stats["mission_time_min_s"] == 101.0
    assert stats["mission_time_max_s"] == 103.0
    path = tmp_path / "speed_sweep.csv"
    write_table(path, rows)
    assert len(path.read_text().splitlines()) == 4


def test_unsafe_faster_candidate_is_not_selected():
    rows = [safe_run("baseline", index, 120.0) for index in (1, 2, 3)]
    rows += [safe_run("s1", index, 90.0, clearance=-0.01) for index in (1, 2, 3)]
    assert select_candidates(rows) == []


def test_below_benefit_threshold_is_not_selected():
    rows = [safe_run("baseline", index, 100.0) for index in (1, 2, 3)]
    rows += [safe_run("s1", index, 96.0) for index in (1, 2, 3)]
    assert select_candidates(rows) == []


def test_ten_second_benefit_is_selected():
    rows = [safe_run("baseline", index, 150.0) for index in (1, 2, 3)]
    rows += [safe_run("s1", index, 140.0) for index in (1, 2, 3)]
    assert select_candidates(rows) == ["s1"]
