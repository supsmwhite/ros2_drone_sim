import json
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_navigation_speed_sweep import (  # noqa: E402
    aggregate, candidate_parameters, event_data, failed_record, select_candidates,
    sample_metrics, write_table)


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
