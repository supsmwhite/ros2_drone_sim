import csv
import hashlib
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from build_disturbance_report_metrics import RUNS, build_report


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def make_run(root, scenario_id, relative_path, release, entry_delta, confirmed):
    run = root / relative_path
    run.mkdir(parents=True)
    summary = {
        "scenario_id": scenario_id,
        "overall_pass": True,
        "metrics": {
            "force_release_time_s": release,
            "force_duration_s": 2.0,
            "mean_horizontal_force_n": [.3, 0.0],
            "peak_horizontal_deviation_m": .4,
            "peak_force_direction_displacement_m": .4,
            "disturbance_steady_state_error_m": .2,
            "recovery_time_s": entry_delta,
            "reverse_overshoot_m": .1,
            "final_position_error_m": .01,
            "final_speed_m_s": .02,
            "saturation_sample_count": 0,
        },
    }
    (run / "summary.json").write_text(json.dumps(summary))
    with (run / "events.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=("mission_time_s", "event"))
        writer.writeheader()
        writer.writerow({"mission_time_s": confirmed, "event": "recovery_confirmed"})
    (run / "evidence_sha256.txt").write_text(
        f"{sha256(run / 'summary.json')}  summary.json\n"
        f"{sha256(run / 'events.csv')}  events.csv\n")


def test_build_report_uses_verified_summary_and_events(tmp_path):
    values = ((15.6048076, 6.2848704, 22.8945343),
              (23.6803567, 0.0, 24.6853619))
    for (scenario, relative), timing in zip(RUNS, values):
        make_run(tmp_path, scenario, relative, *timing)
    report = build_report(tmp_path)
    short, persistent = report["runs"]
    assert short["recovery_threshold_entry_time_s"] == pytest.approx(6.2848704)
    assert short["recovery_confirmed_time_s"] == pytest.approx(7.2897267)
    assert short["recovery_confirmation_hold_time_s"] == pytest.approx(1.0048563)
    assert persistent["recovery_threshold_entry_time_s"] == pytest.approx(0.0)
    assert persistent["recovery_confirmed_time_s"] == pytest.approx(1.0050052)
    assert persistent["recovery_confirmation_hold_time_s"] == pytest.approx(1.0050052)
    assert short["disturbance_end_mean_error_m"] == .2
    json.dumps(report, allow_nan=False)


def test_build_report_rejects_checksum_change(tmp_path):
    for scenario, relative in RUNS:
        make_run(tmp_path, scenario, relative, 10.0, 1.0, 12.0)
    first = tmp_path / RUNS[0][1] / "summary.json"
    first.write_text("{}")
    with pytest.raises(ValueError, match="checksum mismatch"):
        build_report(tmp_path)
