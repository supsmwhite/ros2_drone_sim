import hashlib
import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO = Path(__file__).parents[2]
SCRIPT = REPO / "scripts" / "run_final_assessment.sh"
sys.path.insert(0, str(REPO / "tools"))

from final_assessment_manifest import (REQUIRED_PARAMETERS, finalize_entry,
    recalculate_evidence_checksums, report_eligibility)


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


def test_analyzer_uses_run_parameter_snapshot(tmp_path):
    result = run_script(tmp_path, "hover")
    expected = tmp_path / "01_hover" / "smoke" / "run_01" / "parameters"
    assert f"analyzer_parameters={expected}" in result.stdout


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


def test_recorder_stdout_is_preserved_and_refreshed(tmp_path):
    temporary = tmp_path / "temporary"; run = tmp_path / "run"
    temporary.mkdir(); run.mkdir()
    (temporary / "recorder_stdout.log").write_text("before abnormal exit\n")
    command = (
        f"source '{REPO / 'scripts/final_assessment_lib.sh'}'; "
        f"preserve_assessment_logs '{temporary}' '{run}'; "
        f"printf 'after stop\\n' >>'{temporary / 'recorder_stdout.log'}'; "
        f"preserve_assessment_logs '{temporary}' '{run}'")
    subprocess.run(["bash", "-c", command], check=True)
    assert (run / "recorder_stdout.log").read_text() == "before abnormal exit\nafter stop\n"
    assert not (run / "recorder.log").exists()


@pytest.mark.parametrize("response,accepted", [
    ("response:\naccepted: true\n", True),
    ("ExecuteGoalSequence_Response(accepted=True, message='accepted')\n", True),
    ("response:\naccepted: false\n", False),
    ("ExecuteGoalSequence_Response(accepted=False, message='rejected')\n", False),
])
def test_navigation_acceptance_response_formats(tmp_path, response, accepted):
    response_file = tmp_path / "submission.log"
    response_file.write_text(response)
    command = (
        f"source '{REPO / 'scripts/final_assessment_lib.sh'}'; "
        f"navigation_response_was_accepted '{response_file}'")
    result = subprocess.run(["bash", "-c", command])
    assert (result.returncode == 0) is accepted


def test_domain_daemon_cleanup_uses_the_trial_domain(tmp_path):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    fake_ros2 = fake_bin / "ros2"
    fake_ros2.write_text(
        "#!/usr/bin/env bash\n"
        "printf '%s|%s\\n' \"$ROS_DOMAIN_ID\" \"$*\" >\"$DAEMON_CALL_LOG\"\n")
    fake_ros2.chmod(0o755)
    call_log = tmp_path / "daemon-call.log"
    command = (
        f"source '{REPO / 'scripts/final_assessment_lib.sh'}'; "
        "stop_ros_domain_daemon 177")
    env = {"PATH": f"{fake_bin}:/usr/bin:/bin", "DAEMON_CALL_LOG": str(call_log)}
    subprocess.run(["bash", "-c", command], check=True, env=env)
    assert call_log.read_text() == "177|daemon stop\n"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def make_review_run(tmp_path, status="final", overall_pass=True):
    root = tmp_path / "results"
    relative = f"01_hover/{status}/run_01"
    run = root / relative; run.mkdir(parents=True)
    protected = {
        "metadata.json": json.dumps({"status": status,
            "stop_reason": "arrival_and_steady_window_complete"}),
        "summary.json": json.dumps({"status": status,
            "overall_pass": overall_pass, "failure_reasons": []}),
        "samples.csv": "recording_time_s\n0\n",
        "diagnostics.csv": "mission_time_s,any_saturated\n0,0\n",
        "events.csv": "event\nmission_started\n",
        "paths.json": "{}\n",
    }
    for name, content in protected.items(): (run / name).write_text(content)
    parameter_dir = run / "parameters"; parameter_dir.mkdir()
    for name in REQUIRED_PARAMETERS: (parameter_dir / name).write_text(f"# {name}\n")
    (run / "parameter_sha256.txt").write_text("".join(
        f"{sha(parameter_dir / name)}  {name}\n" for name in REQUIRED_PARAMETERS))
    (run / "manual_acceptance.md").write_text("Status: incomplete\n- [ ] review\nReviewer:\nDate:\n")
    entry = {
        "scenario_id": "hover", "recorder_experiment": "hover",
        "status": status, "run_id": "run_01", "path": relative,
        "git": {"commit_before": "abc", "commit_after": "abc",
                "source_clean_before": True, "source_clean_after": True},
        "parameter_snapshot": {"complete": True},
        "manual_acceptance": {"completed": False,
            "file": f"{relative}/manual_acceptance.md", "screenshots": []},
        "report_eligible": False,
    }
    (run / "manifest_entry.json").write_text(json.dumps(entry))
    manifest = {"schema_version": 4, "generated_at": None, "runs": [entry]}
    manifest_path = root / "manifest.json"; manifest_path.write_text(json.dumps(manifest))
    recalculate_evidence_checksums(run)
    return root, run, manifest_path


def complete_manual(run, create_screenshot=True):
    screenshot = run / "screenshots" / "rviz_overview.png"
    if create_screenshot:
        screenshot.parent.mkdir(); screenshot.write_bytes(b"png evidence")
    (run / "manual_acceptance.md").write_text(
        "Status: complete\n- [x] review\nReviewer: Test Reviewer\n"
        "Date: 2026-07-21\nScreenshot: screenshots/rviz_overview.png\n")
    return "screenshots/rviz_overview.png"


def finalize_args(run, manifest, screenshot):
    return SimpleNamespace(run_dir=run, manifest=manifest, screenshot=[screenshot])


def test_final_manual_review_promotes_and_synchronizes_manifest(tmp_path):
    _, run, manifest = make_review_run(tmp_path)
    protected_before = {name: sha(run / name) for name in (
        "samples.csv", "diagnostics.csv", "events.csv", "paths.json", "summary.json")}
    screenshot = complete_manual(run)
    entry = finalize_entry(finalize_args(run, manifest, screenshot))
    root_entry = json.loads(manifest.read_text())["runs"][0]
    assert entry["report_eligible"] and root_entry == entry
    assert entry["manual_acceptance"]["screenshots"] == [screenshot]
    assert not entry["eligibility_failures"]
    assert {name: sha(run / name) for name in protected_before} == protected_before
    assert screenshot in (run / "evidence_sha256.txt").read_text()


def test_finalize_cli_mode(tmp_path):
    _, run, manifest = make_review_run(tmp_path)
    screenshot = complete_manual(run)
    result = subprocess.run([
        sys.executable, str(REPO / "tools/final_assessment_manifest.py"), "finalize",
        "--manifest", str(manifest), "--run-dir", str(run),
        "--screenshot", screenshot], text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    assert json.loads((run / "manifest_entry.json").read_text())["report_eligible"]


def test_incomplete_manual_review_is_rejected(tmp_path):
    _, run, manifest = make_review_run(tmp_path)
    with pytest.raises(ValueError, match="Status must be complete"):
        finalize_entry(finalize_args(run, manifest, "screenshots/missing.png"))


def test_missing_required_screenshot_is_rejected(tmp_path):
    _, run, manifest = make_review_run(tmp_path)
    screenshot = complete_manual(run, create_screenshot=False)
    with pytest.raises(ValueError, match="screenshot is missing"):
        finalize_entry(finalize_args(run, manifest, screenshot))


def test_failed_summary_cannot_be_finalized(tmp_path):
    _, run, manifest = make_review_run(tmp_path, overall_pass=False)
    screenshot = complete_manual(run)
    with pytest.raises(ValueError, match="overall_pass must be true"):
        finalize_entry(finalize_args(run, manifest, screenshot))


@pytest.mark.parametrize("status", ["smoke", "trial"])
def test_non_final_run_cannot_be_promoted(tmp_path, status):
    _, run, manifest = make_review_run(tmp_path, status=status)
    screenshot = complete_manual(run)
    with pytest.raises(ValueError, match="only an existing final run"):
        finalize_entry(finalize_args(run, manifest, screenshot))


def test_repeated_finalize_is_explicitly_rejected(tmp_path):
    _, run, manifest = make_review_run(tmp_path)
    screenshot = complete_manual(run)
    finalize_entry(finalize_args(run, manifest, screenshot))
    with pytest.raises(ValueError, match="already finalized"):
        finalize_entry(finalize_args(run, manifest, screenshot))


def test_finalize_rejects_changed_raw_evidence(tmp_path):
    _, run, manifest = make_review_run(tmp_path)
    screenshot = complete_manual(run)
    (run / "samples.csv").write_text("changed\n")
    with pytest.raises(ValueError, match="changed after recording"):
        finalize_entry(finalize_args(run, manifest, screenshot))
