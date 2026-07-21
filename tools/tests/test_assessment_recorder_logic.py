import math
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))

from assessment_metrics import (ExperimentStopController, PathHistory,
    mission_relative_time, normalized_target, path_length,
    prepare_output_directory, rotate_body_velocity_to_map, targets_match)


def update_arrival(controller):
    controller.update(0, mission_started=True, goal_error=1, speed=0)
    controller.update(1, mission_started=True, goal_error=.05, speed=.01)
    controller.update(2, mission_started=True, goal_error=.05, speed=.01)


def test_hover_arrival_then_steady_stop():
    c = ExperimentStopController("hover", steady_window=2)
    update_arrival(c); assert not c.stopped
    c.update(4, True, .01, speed=.01); assert c.stopped


def test_single_goal_arrival_then_stop():
    c = ExperimentStopController("single_goal", steady_window=1)
    update_arrival(c); c.update(3, True, .01, speed=.01); assert c.stopped


def test_arrival_hold_resets():
    c = ExperimentStopController("hover")
    c.update(0, True, .05, speed=.01); c.update(.8, True, .2, speed=.01)
    c.update(1, True, .05, speed=.01); c.update(1.9, True, .05, speed=.01)
    assert c.state == "WAITING_FOR_ARRIVAL"


def test_multi_goal_does_not_stop_at_first_arrival():
    c = ExperimentStopController("multi_goal")
    c.update(0, True, .01, speed=.01); c.update(10, True, .01, speed=.01)
    assert c.state == "WAITING_FOR_MISSION_COMPLETE" and not c.stopped


def test_multi_goal_stops_after_complete_and_window():
    c = ExperimentStopController("multi_goal", steady_window=2)
    c.update(0, True); c.update(10, True, mission_complete=True); c.update(12, True)
    assert c.stop_reason == "mission_complete_and_steady_window_complete"


def test_navigation_ignores_intermediate_arrival():
    c = ExperimentStopController("navigation")
    c.update(0, interactive_active=True); c.update(5, goal_error=.01, speed=.01)
    assert not c.stopped


def test_navigation_success_stops():
    c = ExperimentStopController("navigation", steady_window=1)
    c.update(0, interactive_active=True); c.update(5, navigation_complete=True,
        navigation_success=True, interactive_active=False); c.update(6)
    assert c.stop_reason.startswith("navigation_success")


def test_navigation_failure_also_stops():
    c = ExperimentStopController("navigation", steady_window=1)
    c.update(0, interactive_active=True); c.update(2, navigation_complete=True,
        navigation_success=False, interactive_active=False); c.update(3)
    assert "failure" in c.stop_reason


def test_disturbance_without_force_does_not_stop():
    c = ExperimentStopController("disturbance")
    c.update(0, True, .01, .01, .01, force_active=False); c.update(10, True, .01, .01, .01, force_active=False)
    assert c.state == "WAITING_FOR_FORCE"


def test_disturbance_without_release_does_not_stop():
    c = ExperimentStopController("disturbance")
    c.update(0, True, force_active=False); c.update(1, True, force_active=True); c.update(20, True, force_active=True)
    assert c.state == "FORCE_ACTIVE"


def test_disturbance_recovery_hold_and_window():
    c = ExperimentStopController("disturbance", steady_window=1)
    c.update(0, True, force_active=False); c.update(1, True, force_active=True)
    c.update(3, True, horizontal_error=.2, speed=.1, force_active=False)
    c.update(4, True, horizontal_error=.05, speed=.01, force_active=False)
    c.update(5, True, horizontal_error=.05, speed=.01, force_active=False)
    c.update(6, True, horizontal_error=.05, speed=.01, force_active=False)
    assert c.stop_reason == "disturbance_recovery_and_steady_window_complete"


def test_failure_rejection_observation_stops():
    c = ExperimentStopController("failure_case", failure_observation_window=2)
    c.update(0, True, interactive_active=False)
    c.update(1, True, interactive_active=False, failure_reason="REJECTED: no path")
    c.update(3, True, interactive_active=False)
    assert c.stop_reason == "failure_detected_and_observation_complete"


def test_timeout_records_current_state():
    c = ExperimentStopController("disturbance"); c.update(0, True, force_active=False); c.timeout()
    assert c.stop_reason == "timeout_in_state_waiting_for_force"


def test_path_segments_append_and_deduplicate():
    h = PathHistory(); points = [[0, 0, 0], [1, 0, 0]]
    assert h.add("planned", points, 1, .5, 0); assert not h.add("planned", points, 2, 1.5, 0)
    assert len(h.segments["planned"]) == 1


def test_empty_path_records_clear_without_overwrite():
    h = PathHistory(); h.add("reference", [[0, 0, 0], [1, 0, 0]], 1, .5, 0); h.add("reference", [], 2, 1.5, 0)
    assert len(h.segments["reference"]) == 1 and len(h.clear_events) == 1


def test_actual_path_keeps_longest_snapshot():
    h = PathHistory(); h.add("actual", [[0, 0, 0], [1, 0, 0]], 1); h.add("actual", [[0, 0, 0]], 2)
    assert len(h.actual) == 2


def test_three_segment_lengths_accumulate():
    h = PathHistory()
    for i in range(3): h.add("planned", [[i, 0, 0], [i + 1, 0, 0]], i, i, i)
    assert sum(path_length(s["points"]) for s in h.segments["planned"]) == 3


def test_path_schema_records_both_time_domains():
    h = PathHistory(); h.add("reference", [[0, 0, 0]], 4.0, 1.5, 2)
    item = h.as_dict()["reference_segments"][0]
    assert h.as_dict()["schema_version"] == 3
    assert item["recording_time_s"] == 4.0 and item["mission_time_s"] == 1.5


def test_mission_start_event_is_exactly_zero():
    assert mission_relative_time(12.5, 12.5) == 0.0


def test_pre_mission_event_is_null_not_negative():
    assert mission_relative_time(12.0, 12.5) is None


def test_tiny_timestamp_roundoff_clamps_to_zero():
    assert mission_relative_time(12.5 - 5e-10, 12.5) == 0.0


def test_nonempty_output_requires_explicit_overwrite(tmp_path):
    output = tmp_path / "run"; output.mkdir(); (output / "samples.csv").write_text("old")
    with pytest.raises(FileExistsError):
        prepare_output_directory(output)
    assert prepare_output_directory(output, allow_overwrite=True) == output
    assert (output / "samples.csv").read_text() == "old"


def test_final_output_cannot_be_overwritten(tmp_path):
    output = tmp_path / "run"; output.mkdir(); (output / "metadata.json").write_text("{}")
    with pytest.raises(FileExistsError):
        prepare_output_directory(output, allow_overwrite=True, run_status="final")


def test_expected_goal_rejects_stale_hover_and_accepts_wrapped_yaw():
    expected = normalized_target([2, 1, 1.5], 0)
    stale = normalized_target([0, 0, 1.5], 0)
    wrapped = normalized_target([2, 1, 1.5], 2 * math.pi)
    assert not targets_match(stale, expected)
    assert targets_match(wrapped, expected)


def test_body_velocity_is_rotated_to_map_frame():
    half = math.sqrt(.5)
    assert rotate_body_velocity_to_map([0, 0, half, half], [1, 0, 0]) == pytest.approx([0, 1, 0])
