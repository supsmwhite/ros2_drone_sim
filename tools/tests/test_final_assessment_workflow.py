import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).parents[2]
SCRIPT = REPO / "scripts" / "run_final_assessment.sh"
sys.path.insert(0, str(REPO / "tools"))

from final_assessment_manifest import report_eligibility


def run_script(tmp_path, experiment="hover", status="smoke", run_id="run_01", extra=()):
    return subprocess.run(
        [str(SCRIPT), "--experiment", experiment, "--status", status,
         "--run-id", run_id, "--use-rviz", "false",
         "--output-root", str(tmp_path), "--timeout", "10", "--dry-run", *extra],
        cwd=REPO, text=True, capture_output=True)


@pytest.mark.parametrize("experiment,scenario_dir,recorder,target", [
    ("hover", "01_hover", "hover", "--expected-goal 0 0 1.5 0"),
    ("single_goal", "02_single_goal", "single_goal", "--expected-goal 2 1 1.5 0"),
    ("multi_goal", "03_multi_goal", "multi_goal", "1.5707963267948966"),
    ("static_avoidance", "04_static_avoidance", "navigation", "13.2 5.5 1.5"),
    ("narrow_corridor", "05_narrow_corridor", "navigation", "12.1 1.1 1.5"),
])
def test_fixed_scenario_mapping(tmp_path, experiment, scenario_dir, recorder, target):
    result = run_script(tmp_path, experiment)
    assert result.returncode == 0, result.stderr
    assert f"scenario_dir={scenario_dir}" in result.stdout
    assert f"recorder_experiment={recorder}" in result.stdout
    assert target in result.stdout


def test_static_and_narrow_have_distinct_identity_and_path(tmp_path):
    static = run_script(tmp_path, "static_avoidance").stdout
    narrow = run_script(tmp_path, "narrow_corridor").stdout
    assert "scenario_id=static_avoidance" in static and "04_static_avoidance" in static
    assert "scenario_id=narrow_corridor" in narrow and "05_narrow_corridor" in narrow
    assert "recorder_experiment=navigation" in static and "recorder_experiment=navigation" in narrow


@pytest.mark.parametrize("arguments,error", [
    (("--experiment", "unknown", "--status", "smoke", "--run-id", "x"), "invalid --experiment"),
    (("--experiment", "hover", "--status", "candidate", "--run-id", "x"), "invalid --status"),
    (("--experiment", "hover", "--status", "smoke", "--run-id", "../x"), "unsafe --run-id"),
])
def test_invalid_arguments_are_rejected(tmp_path, arguments, error):
    result = subprocess.run(
        [str(SCRIPT), *arguments, "--output-root", str(tmp_path), "--dry-run"],
        cwd=REPO, text=True, capture_output=True)
    assert result.returncode == 2 and error in result.stderr


def test_existing_run_directory_is_rejected_even_for_dry_run(tmp_path):
    (tmp_path / "01_hover" / "trial" / "run_01").mkdir(parents=True)
    result = run_script(tmp_path, status="trial")
    assert result.returncode == 2
    assert "refusing to overwrite" in result.stderr


def test_final_dirty_worktree_is_rejected(tmp_path):
    marker = REPO / ".assessment_dirty_test_marker"
    marker.write_text("test")
    try:
        result = run_script(tmp_path, status="final")
    finally:
        marker.unlink()
    assert result.returncode == 2
    assert "clean worktree" in result.stderr


def test_report_eligibility_requires_every_condition():
    eligible, conditions, failures = report_eligibility(
        "final", "arrival_and_steady_window_complete", True,
        "abc", "abc", True, True, True, True)
    assert eligible and all(conditions.values()) and not failures
    eligible, conditions, failures = report_eligibility(
        "trial", "timeout_in_state_waiting", False,
        "abc", "def", False, False, False, False)
    assert not eligible
    assert set(failures) == {name for name, passed in conditions.items() if not passed}
    assert "manual_acceptance_complete" in failures and "status_is_final" in failures


def test_initial_run_is_not_report_eligible_without_manual_acceptance():
    eligible, _, failures = report_eligibility(
        "final", "navigation_success_and_steady_window_complete", True,
        "abc", "abc", True, True, True, False)
    assert not eligible and failures == ["manual_acceptance_complete"]
