import json
import sys
from pathlib import Path

import pytest


TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
import run_simplifier_clearance_sweep as sweep  # noqa: E402


def test_candidates_keep_a2_speeds_and_map_clearance():
    assert sweep.candidate_parameters("C20") == {
        "nominal_speed": 0.50,
        "max_reference_speed": 0.90,
        "max_reference_acceleration": 0.60,
        "shortcut_preferred_clearance": 0.20,
    }
    with pytest.raises(ValueError):
        sweep.candidate_parameters("c30")


def test_parse_simplifier_diagnostics_is_backward_compatible():
    old = ("ordered goal 0 trajectory ready: raw_points=10 simplified_points=3 "
           "duration=2.0 s velocity_scale=1.0 duration_scale=1.0")
    assert sweep.parse_simplifier_diagnostics(old) == []
    new = old + (" preferred_shortcuts=2 fallback_shortcuts=1 "
                 "collision_only_shortcuts=0 shortcut_preferred_clearance=0.200")
    assert sweep.parse_simplifier_diagnostics(new) == [{
        "goal_index": 0, "preferred_shortcut_count": 2,
        "fallback_shortcut_count": 1, "collision_only_shortcut_count": 0,
        "shortcut_preferred_clearance": 0.2,
    }]


def test_segment_lengths_and_nearest_corner():
    paths = {"simplified_segments": [
        {"sequence": 1, "points": [[1, 1, 0], [1, 4, 0]]},
        {"sequence": 0, "points": [[0, 0, 0], [1, 0, 0]]},
    ]}
    assert sweep.combined_segment_lengths(paths) == [1.0, 3.0]
    corners = [{"position": [3.6, 1.85, 1.5]}]
    assert sweep.nearest_corner(corners, (3.60, 1.85)) == corners[0]
    assert sweep.nearest_corner(corners, (7.60, -1.15)) is None


def test_summarize_applies_hard_screening(tmp_path):
    run = tmp_path / "s0/full_map/run_01"
    run.mkdir(parents=True)
    base = {"candidate": "s0", "route": "full_map", "run": 1,
            "success": True, "mission_time_s": 55.0,
            "simplified_points": 10, "simplified_path_length_m": 20.0,
            "tracking_max_m": 0.02, "minimum_actual_clearance_m": 0.1,
            "collision": False, "nonfinite": 0, "saturated_at_end": False,
            "repository_commit": "abc", "git_dirty": False}
    (run / "simplifier_summary.json").write_text(json.dumps(base))
    rows = sweep.summarize(tmp_path)
    assert rows[0]["selection"] == "pass"
    assert (tmp_path / "candidate_comparison.csv").exists()
    assert (tmp_path / "candidate_comparison.json").exists()
    assert (tmp_path / "simplifier_clearance_summary.md").exists()
