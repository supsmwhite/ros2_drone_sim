import math
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from run_trajectory_curvature_timing_sweep import (  # noqa: E402
    hausdorff,
    parameters,
    parse_timing_log,
    paths_match,
    point_path_distance,
)


def test_candidate_parameters_keep_a2_and_scan_only_maximum_scale():
    baseline = parameters("t0")
    assert baseline == {
        "nominal_speed": 0.50,
        "max_reference_speed": 0.90,
        "max_reference_acceleration": 0.60,
        "corner_timing_enabled": False,
        "corner_timing_start_angle_deg": 25.0,
        "corner_timing_full_angle_deg": 70.0,
        "corner_timing_max_duration_scale": 1.0,
    }
    assert parameters("t10")["corner_timing_max_duration_scale"] == 1.1
    assert parameters("t20")["corner_timing_max_duration_scale"] == 1.2
    assert parameters("t30")["corner_timing_max_duration_scale"] == 1.3


def test_timing_log_parser_preserves_goal_and_per_segment_diagnostics():
    text = """
ordered goal 0 trajectory ready: raw_points=3 simplified_points=3 initial_simplified_points=3 refinements=0 duration=5.000 s velocity_scale=1.00 duration_scale=1.05 max_speed=0.5 m/s max_acceleration=0.4 m/s^2 raw_length=2 m simplified_length=2 m expanded_nodes=3 corner_timing_enabled=true maximum_turn_angle_deg=90.000000 maximum_corner_duration_scale=1.200000 corner_adjusted_segment_count=2 maximum_segment_corner_scale=1.200000 global_duration_scale=1.05
trajectory timing segment: goal=0 segment_index=0 segment_length=1.000000 start_corner_angle=0.000000 end_corner_angle=90.000000 corner_scale=1.200000 base_duration=2.000000 corner_adjusted_duration=2.400000 final_duration=2.520000
"""
    goals, segments = parse_timing_log(text)
    assert goals == [{
        "goal_index": 0, "enabled": True, "maximum_turn_angle_deg": 90.0,
        "maximum_corner_duration_scale": 1.2,
        "corner_adjusted_segment_count": 2,
        "maximum_segment_corner_scale": 1.2,
        "selected_global_duration_scale": 1.05,
    }]
    assert segments[0]["corner_scale"] == 1.2
    assert segments[0]["base_duration"] == 2.0
    assert segments[0]["corner_adjusted_duration"] == 2.4
    assert segments[0]["final_duration"] == 2.52


def test_cross_track_and_hausdorff_use_continuous_polyline_distance():
    horizontal = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]]
    shifted = [[0.0, 1.0, 0.0], [2.0, 1.0, 0.0]]
    assert math.isclose(point_path_distance([1.0, 0.5, 0.0], horizontal), 0.5)
    assert math.isclose(hausdorff(horizontal, shifted), 1.0)


def test_geometry_match_allows_only_millimetric_start_difference():
    baseline = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]
    assert paths_match([[0.005, 0.0, 0.0], *baseline[1:]], baseline)
    assert not paths_match([[0.02, 0.0, 0.0], *baseline[1:]], baseline)
    assert not paths_match(baseline[:-1], baseline)
    changed_middle = [baseline[0], [1.0, 0.001, 0.0], baseline[-1]]
    assert not paths_match(changed_middle, baseline)
