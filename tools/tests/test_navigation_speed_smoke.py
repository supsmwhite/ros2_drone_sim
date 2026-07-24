import math
import yaml

from tools.navigation_speed_smoke import (
    FORMAL_FOUR_GOALS, FORMAL_FOUR_GOAL_SCENARIO, RUN_SCENARIOS, SCENARIOS,
    collision_count, make_open_environment, over_threshold_stats, path_collision_count,
    path_segments, percentile, segment_intersects_box, segments_length)


def test_segment_collision_includes_crossing_and_excludes_clear_segment():
    box = ((1.0, 1.0, 1.0), (2.0, 2.0, 2.0))
    assert segment_intersects_box((0.0, 1.5, 1.5), (3.0, 1.5, 1.5), box)
    assert not segment_intersects_box((0.0, 0.0, 0.0), (0.5, 0.5, 0.5), box)
    assert collision_count(
        [(0.0, 1.5, 1.5), (3.0, 1.5, 1.5), (4.0, 1.5, 1.5)], [box]) == 1


def test_segments_length_accepts_path_history_shape():
    segments = [
        {"points": [[0.0, 0.0, 0.0], [3.0, 4.0, 0.0]]},
        {"points": [[3.0, 4.0, 0.0], [3.0, 4.0, 2.0]]},
    ]
    assert math.isclose(segments_length(segments), 7.0)


def test_open_environment_omits_untyped_empty_obstacle_list(tmp_path):
    path = make_open_environment(tmp_path)
    document = yaml.safe_load(path.read_text())
    parameters = next(iter(document.values()))["ros__parameters"]
    assert "obstacles" not in parameters


def test_path_segments_reads_recorder_schema(tmp_path):
    path = tmp_path / "paths.json"
    path.write_text('{"planned_segments":[{"mission_time_s":3.5,"points":[[0,0,1],[1,0,1]]}]}')
    assert path_segments(path, "planned")[0]["mission_time_s"] == 3.5


def test_path_collision_count_sums_per_segment_without_joining_segments():
    box = ((1.0, 1.0, 1.0), (2.0, 2.0, 2.0))
    segments = [
        {"points": [(0.0, 1.5, 1.5), (3.0, 1.5, 1.5)]},
        {"points": [(4.0, 4.0, 4.0), (5.0, 4.0, 4.0)]},
    ]
    assert path_collision_count(segments, [box]) == 1


def test_percentile_interpolates_between_ranked_samples():
    values = [1.0, 2.0, 3.0, 4.0, 5.0]
    assert math.isclose(percentile(values, 0.5), 3.0)
    assert math.isclose(percentile(values, 0.0), 1.0)
    assert math.isclose(percentile(values, 1.0), 5.0)
    assert percentile([], 0.5) is None


def test_over_threshold_stats_uses_real_timestamp_deltas():
    # Above-threshold from t=0 to t=0.3 (0.3 s), then below, then above again
    # from t=0.5 to t=0.6 (0.1 s); longest continuous run is 0.3 s.
    series = [
        (0.0, 0.09), (0.1, 0.08), (0.2, 0.07), (0.3, 0.02),
        (0.4, 0.01), (0.5, 0.06), (0.6, 0.01),
    ]
    samples_over, fraction, duration, longest = over_threshold_stats(series, 0.05)
    assert samples_over == 4
    assert math.isclose(fraction, 4 / 7)
    assert math.isclose(duration, 0.4)
    assert math.isclose(longest, 0.3)


def test_over_threshold_stats_handles_empty_series():
    assert over_threshold_stats([], 0.05) == (0, None, 0.0, 0.0)


def test_formal_four_goal_trial_is_explicit_and_not_part_of_lightweight_all():
    assert FORMAL_FOUR_GOAL_SCENARIO not in SCENARIOS
    assert RUN_SCENARIOS[FORMAL_FOUR_GOAL_SCENARIO] == FORMAL_FOUR_GOALS
    assert [goal[0] for goal in FORMAL_FOUR_GOALS] == [
        (13.15, 5.80, 3.40),
        (9.70, -1.20, 1.20),
        (6.30, 5.55, 2.35),
        (0.45, 5.70, 1.00),
    ]
