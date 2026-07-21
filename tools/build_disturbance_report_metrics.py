#!/usr/bin/env python3
"""Build report-only disturbance metrics from checksum-verified immutable evidence."""

import argparse
import csv
import json
import math
from pathlib import Path

from assessment_metrics import disturbance_recovery_times
from final_assessment_manifest import file_sha256, parse_checksum_file


RUNS = (
    ("disturbance_short_gust", "06_disturbance/short_gust/final/run_01"),
    ("disturbance_persistent_release", "06_disturbance/persistent_release/final/run_01"),
)


def verified_source(run_dir, name):
    checksum_path = run_dir / "evidence_sha256.txt"
    if not checksum_path.is_file():
        raise ValueError(f"missing evidence checksums: {checksum_path}")
    checksums = parse_checksum_file(checksum_path, run_dir)
    source = (run_dir / name).resolve()
    if not source.is_file() or source not in checksums:
        raise ValueError(f"source is not protected by evidence checksums: {source}")
    if file_sha256(source) != checksums[source]:
        raise ValueError(f"source checksum mismatch: {source}")
    return source


def event_time(events_path, name):
    with events_path.open(newline="") as handle:
        values = []
        for row in csv.DictReader(handle):
            if row.get("event") != name:
                continue
            try:
                value = float(row["mission_time_s"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(value):
                values.append(value)
    return values[0] if values else None


def build_run(results_root, scenario_id, relative_path):
    run_dir = results_root / relative_path
    summary_path = verified_source(run_dir, "summary.json")
    events_path = verified_source(run_dir, "events.csv")
    summary = json.loads(summary_path.read_text())
    if summary.get("scenario_id") != scenario_id:
        raise ValueError(f"unexpected scenario_id in {summary_path}")
    metrics = summary["metrics"]
    release = metrics.get("force_release_time_s")
    threshold_delta = metrics.get("recovery_time_s")
    threshold_absolute = (None if release is None or threshold_delta is None else
                          release + threshold_delta)
    recovery = disturbance_recovery_times(
        release, threshold_absolute, event_time(events_path, "recovery_confirmed"))
    return {
        "scenario_id": scenario_id,
        "source_summary": str(Path(relative_path) / "summary.json"),
        "source_events": str(Path(relative_path) / "events.csv"),
        "force_duration_s": metrics.get("force_duration_s"),
        "mean_horizontal_force_n": metrics.get("mean_horizontal_force_n"),
        "peak_horizontal_deviation_m": metrics.get("peak_horizontal_deviation_m"),
        "peak_force_direction_displacement_m": metrics.get("peak_force_direction_displacement_m"),
        "disturbance_end_mean_error_m": metrics.get("disturbance_steady_state_error_m"),
        "recovery_threshold_entry_time_s": recovery["recovery_threshold_entry_time_s"],
        "recovery_confirmed_time_s": recovery["recovery_confirmed_time_s"],
        "recovery_confirmation_hold_time_s": recovery["recovery_confirmation_hold_time_s"],
        "reverse_overshoot_m": metrics.get("reverse_overshoot_m"),
        "final_position_error_m": metrics.get("final_position_error_m"),
        "final_speed_m_s": metrics.get("final_speed_m_s"),
        "saturation_sample_count": metrics.get("saturation_sample_count"),
        "overall_pass": summary.get("overall_pass"),
    }


def build_report(results_root):
    return {
        "schema_version": 1,
        "metric_semantics": {
            "recovery_threshold_entry_time_s": "seconds after force release until first entry into the configured recovery position and speed thresholds",
            "recovery_confirmed_time_s": "seconds after force release until the Recorder confirms the recovery hold condition",
            "recovery_confirmation_hold_time_s": "elapsed seconds from threshold entry to Recorder recovery confirmation",
        },
        "runs": [build_run(results_root, scenario, path) for scenario, path in RUNS],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = build_report(args.results_root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_name(args.output.name + ".tmp")
    temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    temporary.replace(args.output)
    print(f"Wrote checksum-verified disturbance report metrics: {args.output}")


if __name__ == "__main__":
    main()
