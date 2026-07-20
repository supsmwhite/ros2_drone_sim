#!/usr/bin/env python3
"""Run and summarize repeatable candidate-only navigation speed experiments."""

import argparse
import csv
import json
import math
import os
import re
import shlex
import signal
import statistics
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = Path("/tmp/ros2_drone_navigation_performance")
CANDIDATES = {
    "baseline": (0.35, 0.70, 0.35),
    "s1": (0.40, 0.80, 0.40),
    "s2": (0.45, 0.85, 0.45),
    "s3": (0.50, 0.90, 0.50),
    "s4": (0.55, 1.00, 0.55),
    "a1": (0.45, 0.85, 0.50),
    "a2": (0.50, 0.90, 0.60),
    "a3": (0.55, 1.00, 0.65),
}
ROUTES = {
    "full_map": [((13.2, 5.5, 1.5), 0.0)],
    "three_goal": [
        ((3.5, 1.0, 2.5), math.pi / 2.0),
        ((5.5, 1.0, 4.0), math.pi),
        ((7.0, 5.0, 4.0), -math.pi / 2.0),
    ],
}
CSV_FIELDS = (
    "candidate route run nominal_speed max_reference_speed "
    "max_reference_acceleration success stop_reason total_mission_time_s "
    "navigation_phase_time_s takeoff_time_s recorder_steady_window_s "
    "actual_max_speed_m_s full_mission_actual_max_speed_m_s actual_speed_rms_m_s "
    "actual_mean_speed_m_s "
    "reference_max_speed_m_s reference_max_acceleration_m_s2 "
    "reference_max_horizontal_acceleration_m_s2 "
    "reference_max_vertical_acceleration_m_s2 "
    "reference_max_total_acceleration_m_s2 reference_peak_acceleration_time_s "
    "reference_peak_acceleration_segment reference_peak_acceleration_goal_index "
    "reference_peak_acceleration_position reference_peak_horizontal_acceleration_m_s2 "
    "reference_peak_vertical_acceleration_m_s2 "
    "actual_max_horizontal_acceleration_m_s2 actual_max_vertical_acceleration_m_s2 "
    "actual_max_total_acceleration_m_s2 actual_peak_acceleration_time_s "
    "limiting_reason trajectory_diagnostics "
    "reference_total_duration_s actual_path_length_m reference_path_length_m "
    "navigation_tracking_max_error_m navigation_tracking_rms_error_m "
    "final_position_error_m minimum_raw_obstacle_distance_m "
    "minimum_safety_clearance_m maximum_absolute_roll_rad "
    "maximum_absolute_pitch_rad maximum_angular_speed_rad_s minimum_motor_rpm "
    "maximum_motor_rpm saturation_sample_count longest_saturation_duration_s "
    "saturated_at_end collision non_finite_value_count duration_scales "
    "velocity_scales refinement_iterations segment_durations_s "
    "mission_start_time_s navigation_phase_start_time_s goal_arrival_time_s "
    "mission_complete_time_s recording_stop_time_s navigation_execution_time_s "
    "goal_activation_times_s per_goal_arrival_times_s per_goal_duration_s "
    "repository_commit git_dirty domain_id"
).split()
TRAJECTORY_RE = re.compile(
    r"ordered goal (?P<goal>\d+) trajectory ready: raw_points=(?P<raw>\d+) "
    r"simplified_points=(?P<simplified>\d+) initial_simplified_points=(?P<initial>\d+) "
    r"refinements=(?P<refine>\d+) "
    r"duration=(?P<duration>[0-9.]+) s velocity_scale=(?P<velocity>[0-9.]+) "
    r"duration_scale=(?P<scale>[0-9.]+) max_speed=(?P<speed>[0-9.]+) m/s "
    r"max_acceleration=(?P<accel>[0-9.]+) m/s\^2"
)

MIN_ACCELERATION_DT_S = 1.0e-4
MAX_ACCELERATION_DT_S = 0.10


def candidate_parameters(name):
    try:
        nominal, speed, acceleration = CANDIDATES[name.lower()]
    except KeyError as error:
        raise ValueError(f"unknown candidate: {name}") from error
    return {
        "nominal_speed": nominal,
        "max_reference_speed": speed,
        "max_reference_acceleration": acceleration,
    }


def git_state():
    commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    dirty = bool(subprocess.check_output(
        ["git", "status", "--porcelain"], cwd=ROOT, text=True).strip())
    return commit, dirty


def ros_command(arguments):
    command = "source /opt/ros/humble/setup.bash"
    command += f" && source {shlex.quote(str(ROOT / 'install/setup.bash'))}"
    command += " && exec " + " ".join(shlex.quote(str(item)) for item in arguments)
    return ["bash", "-lc", command]


def request_yaml(route):
    poses = []
    for (x, y, z), yaw in ROUTES[route]:
        poses.append({
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)},
        })
    return json.dumps({
        "goals": {"header": {"frame_id": "map"}, "poses": poses},
        "draft_revision": 1,
    }, separators=(",", ":"))


def terminate(process, grace=8.0):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=grace)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def event_data(path):
    result = []
    if not path.exists():
        return result
    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            try:
                mission_time = float(row["mission_time_s"])
            except (TypeError, ValueError):
                continue
            try:
                details = json.loads(row["details"])
            except (json.JSONDecodeError, TypeError):
                details = {}
            result.append({"event": row["event"],
                           "time": mission_time,
                           "details": details})
    return result


def first_event(events, name, predicate=lambda _: True):
    return next((item["time"] for item in events
                 if item["event"] == name and predicate(item["details"])), None)


def parse_trajectory_log(text):
    segments = []
    for match in TRAJECTORY_RE.finditer(text):
        segments.append({
            "goal_index": int(match.group("goal")),
            "raw_point_count": int(match.group("raw")),
            "simplified_point_count": int(match.group("simplified")),
            "initial_simplified_point_count": int(match.group("initial")),
            "refinement_iterations": int(match.group("refine")),
            "trajectory_duration_s": float(match.group("duration")),
            "velocity_scale": float(match.group("velocity")),
            "duration_scale": float(match.group("scale")),
            "reference_max_speed_m_s": float(match.group("speed")),
            "reference_max_acceleration_m_s2": float(match.group("accel")),
        })
    return segments


def acceleration_components(ax, ay, az):
    values = (ax, ay, az)
    if not all(math.isfinite(value) for value in values):
        return None
    horizontal = math.hypot(ax, ay)
    vertical = abs(az)
    return {
        "horizontal": horizontal,
        "vertical": vertical,
        "total": math.sqrt(horizontal * horizontal + vertical * vertical),
    }


def acceleration_summary(samples):
    valid = []
    for sample in samples:
        components = acceleration_components(
            sample["acceleration"][0], sample["acceleration"][1],
            sample["acceleration"][2])
        if components is not None:
            valid.append((sample, components))
    if not valid:
        return {
            "max_horizontal": None, "max_vertical": None, "max_total": None,
            "peak": None,
        }
    peak_sample, peak_components = max(valid, key=lambda item: item[1]["total"])
    return {
        "max_horizontal": max(item[1]["horizontal"] for item in valid),
        "max_vertical": max(item[1]["vertical"] for item in valid),
        "max_total": peak_components["total"],
        "peak": {
            "mission_time_s": peak_sample.get("mission_time_s"),
            "trajectory_segment_index": peak_sample.get("segment_index"),
            "goal_index": peak_sample.get("goal_index"),
            "position": peak_sample.get("position"),
            "horizontal_acceleration_m_s2": peak_components["horizontal"],
            "vertical_acceleration_m_s2": peak_components["vertical"],
            "total_acceleration_m_s2": peak_components["total"],
        },
    }


def differentiated_acceleration_samples(samples, minimum_dt=MIN_ACCELERATION_DT_S,
                                         maximum_dt=MAX_ACCELERATION_DT_S):
    result = []
    for previous, current, following in zip(samples, samples[1:], samples[2:]):
        dt = following["mission_time_s"] - previous["mission_time_s"]
        if not math.isfinite(dt) or dt < 2.0 * minimum_dt or dt > 2.0 * maximum_dt:
            continue
        velocities = previous["velocity"] + current["velocity"] + following["velocity"]
        if not all(math.isfinite(value) for value in velocities):
            continue
        acceleration = [
            (following["velocity"][axis] - previous["velocity"][axis]) / dt
            for axis in range(3)
        ]
        result.append({**current, "acceleration": acceleration})
    return result


def limiting_reason(segment, reference_summary, parameters):
    scale = segment["duration_scale"]
    if scale > 1.0 + 1.0e-9:
        speed_exceedance = (
            segment["reference_max_speed_m_s"] * scale /
            parameters["max_reference_speed"])
        acceleration_exceedance = (
            reference_summary["max_total"] * scale * scale /
            parameters["max_reference_acceleration"]
            if reference_summary["max_total"] is not None else 0.0)
        if speed_exceedance > 1.0 and speed_exceedance > acceleration_exceedance:
            return "speed_limit"
        peak = reference_summary.get("peak")
        if peak and peak["total_acceleration_m_s2"] > 0.0:
            horizontal_ratio = (
                peak["horizontal_acceleration_m_s2"] /
                peak["total_acceleration_m_s2"])
            vertical_ratio = (
                peak["vertical_acceleration_m_s2"] /
                peak["total_acceleration_m_s2"])
            if horizontal_ratio >= 0.80:
                return "horizontal_acceleration_dominant"
            if vertical_ratio >= 0.80:
                return "vertical_acceleration_dominant"
            return "mixed_acceleration"
        return "unknown"
    if segment["refinement_iterations"] > 0:
        return "collision_refinement"
    segment_count = max(0, segment["simplified_point_count"] - 1)
    base_duration = segment["trajectory_duration_s"] / scale
    if segment_count and abs(base_duration - 2.0 * segment_count) < 1.0e-3:
        return "minimum_segment_duration"
    return "none"


def sample_metrics(path, navigation_start):
    speeds = []
    navigation_samples = []
    reference_samples = []
    nonfinite = 0
    if not path.exists():
        return {}, 0
    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            mission = row.get("mission_time_s", "")
            if not mission:
                continue
            try:
                mission = float(mission)
            except ValueError:
                nonfinite += 1
                continue
            for value in row.values():
                if value:
                    try:
                        nonfinite += int(not math.isfinite(float(value)))
                    except ValueError:
                        pass
            if navigation_start is not None and mission >= navigation_start:
                try:
                    speed = float(row["speed"])
                    if math.isfinite(speed):
                        speeds.append(speed)
                except (KeyError, ValueError):
                    nonfinite += 1
                required_sample_fields = [
                    *(f"velocity_{axis}" for axis in "xyz"),
                    *(f"actual_{axis}" for axis in "xyz"),
                ]
                if all(row.get(field, "") != "" for field in required_sample_fields):
                    try:
                        sample = {
                            "mission_time_s": mission,
                            "velocity": [
                                float(row[f"velocity_{axis}"]) for axis in "xyz"],
                            "position": [
                                float(row[f"actual_{axis}"]) for axis in "xyz"],
                            "goal_index": (int(float(row["navigation_goal_index"]))
                                           if row.get("navigation_goal_index") else None),
                            "segment_index": (int(float(row["navigation_segment_index"]))
                                              if row.get(
                                                  "navigation_segment_index") else None),
                        }
                        navigation_samples.append(sample)
                        reference_values = [
                            row.get(f"reference_acceleration_{axis}", "")
                            for axis in "xyz"]
                        if all(value != "" for value in reference_values):
                            reference_samples.append({
                                **sample,
                                "acceleration": [
                                    float(value) for value in reference_values],
                                "position": [
                                    float(row[f"reference_{axis}"]) for axis in "xyz"],
                            })
                    except (KeyError, TypeError, ValueError):
                        nonfinite += 1
    reference = acceleration_summary(reference_samples)
    actual = acceleration_summary(
        differentiated_acceleration_samples(navigation_samples))
    reference_by_goal = {}
    for goal_index in sorted({sample["goal_index"] for sample in reference_samples
                              if sample["goal_index"] is not None}):
        reference_by_goal[goal_index] = acceleration_summary([
            sample for sample in reference_samples
            if sample["goal_index"] == goal_index])
    return {
        "actual_max_speed_m_s": max(speeds) if speeds else None,
        "actual_speed_rms_m_s": (
            math.sqrt(sum(value * value for value in speeds) / len(speeds))
            if speeds else None),
        "actual_mean_speed_m_s": statistics.fmean(speeds) if speeds else None,
        "reference_max_horizontal_acceleration_m_s2": reference["max_horizontal"],
        "reference_max_vertical_acceleration_m_s2": reference["max_vertical"],
        "reference_max_total_acceleration_m_s2": reference["max_total"],
        "reference_acceleration_summary": reference,
        "reference_acceleration_by_goal": reference_by_goal,
        "actual_max_horizontal_acceleration_m_s2": actual["max_horizontal"],
        "actual_max_vertical_acceleration_m_s2": actual["max_vertical"],
        "actual_max_total_acceleration_m_s2": actual["max_total"],
        "actual_acceleration_summary": actual,
    }, nonfinite


def failed_record(candidate, route, run, parameters, reason, commit, dirty, domain):
    row = {key: None for key in CSV_FIELDS}
    row.update({"candidate": candidate, "route": route, "run": run,
                **parameters, "success": False, "stop_reason": reason,
                "collision": False, "non_finite_value_count": 0,
                "repository_commit": commit, "git_dirty": dirty,
                "domain_id": domain})
    return row


def build_record(run_dir, candidate, route, run, parameters, commit, dirty, domain):
    summary_path = run_dir / "summary.json"
    metadata_path = run_dir / "metadata.json"
    if not summary_path.exists() or not metadata_path.exists():
        return failed_record(candidate, route, run, parameters,
                             "missing analyzer output", commit, dirty, domain)
    summary = json.loads(summary_path.read_text())
    metadata = json.loads(metadata_path.read_text())
    metrics = summary["metrics"]
    events = event_data(run_dir / "events.csv")
    complete = first_event(events, "navigation_complete_changed",
                           lambda value: value.get("value") is True)
    stopped = first_event(events, "recording_stopped")
    navigation_start = summary.get("navigation_phase_start_time_s")
    speed_metrics, nonfinite = sample_metrics(
        run_dir / "samples.csv", navigation_start)
    reference_summary = speed_metrics.pop("reference_acceleration_summary")
    reference_by_goal = speed_metrics.pop("reference_acceleration_by_goal")
    actual_summary = speed_metrics.pop("actual_acceleration_summary")
    launch_text = (run_dir / "launch.log").read_text(errors="replace")
    segments = parse_trajectory_log(launch_text)
    trajectory_diagnostics = []
    for segment in segments:
        acceleration = reference_by_goal.get(
            segment["goal_index"], reference_summary)
        reason = limiting_reason(segment, acceleration, parameters)
        trajectory_diagnostics.append({
            "goal_index": segment["goal_index"],
            "base_duration_s": (
                segment["trajectory_duration_s"] / segment["duration_scale"]),
            "selected_duration_scale": segment["duration_scale"],
            "selected_velocity_scale": segment["velocity_scale"],
            "selected_duration_s": segment["trajectory_duration_s"],
            "max_reference_speed_m_s": segment["reference_max_speed_m_s"],
            "max_reference_horizontal_acceleration_m_s2": acceleration["max_horizontal"],
            "max_reference_vertical_acceleration_m_s2": acceleration["max_vertical"],
            "max_reference_total_acceleration_m_s2": acceleration["max_total"],
            "limiting_reason": reason,
            "peak": acceleration["peak"],
        })
    reasons = list(dict.fromkeys(
        item["limiting_reason"] for item in trajectory_diagnostics))
    reference_peak = reference_summary["peak"] or {}
    actual_peak = actual_summary["peak"] or {}
    row = {key: None for key in CSV_FIELDS}
    row.update({
        "candidate": candidate, "route": route, "run": run, **parameters,
        "success": bool(metrics.get("navigation_complete") and
                        metrics.get("navigation_success") and
                        not metrics.get("collision_observed")),
        "stop_reason": metadata.get("stop_reason"),
        "total_mission_time_s": complete,
        "navigation_phase_time_s": (complete - navigation_start
                                     if complete is not None and
                                     navigation_start is not None else None),
        "takeoff_time_s": navigation_start,
        "recorder_steady_window_s": (stopped - complete
                                      if stopped is not None and
                                      complete is not None else None),
        "full_mission_actual_max_speed_m_s": metadata.get("safety_observations", {}).get(
            "maximum_speed_m_s"),
        **speed_metrics,
        "reference_peak_acceleration_time_s": reference_peak.get("mission_time_s"),
        "reference_peak_acceleration_segment": reference_peak.get(
            "trajectory_segment_index"),
        "reference_peak_acceleration_goal_index": reference_peak.get("goal_index"),
        "reference_peak_acceleration_position": reference_peak.get("position"),
        "reference_peak_horizontal_acceleration_m_s2": reference_peak.get(
            "horizontal_acceleration_m_s2"),
        "reference_peak_vertical_acceleration_m_s2": reference_peak.get(
            "vertical_acceleration_m_s2"),
        "actual_peak_acceleration_time_s": actual_peak.get("mission_time_s"),
        "limiting_reason": ";".join(reasons) if reasons else "unknown",
        "trajectory_diagnostics": trajectory_diagnostics,
        "reference_max_speed_m_s": max(
            (item["reference_max_speed_m_s"] for item in segments), default=None),
        "reference_max_acceleration_m_s2": max(
            (item["reference_max_acceleration_m_s2"] for item in segments),
            default=None),
        "reference_total_duration_s": sum(
            item["trajectory_duration_s"] for item in segments) if segments else None,
        "actual_path_length_m": metrics.get("actual_path_length_m"),
        "reference_path_length_m": metrics.get("reference_path_length_m"),
        "navigation_tracking_max_error_m": metrics.get(
            "navigation_tracking_max_error_m"),
        "navigation_tracking_rms_error_m": metrics.get(
            "navigation_tracking_rms_error_m"),
        "final_position_error_m": metrics.get("final_position_error_m"),
        "minimum_raw_obstacle_distance_m": metrics.get(
            "minimum_raw_obstacle_distance_m"),
        "minimum_safety_clearance_m": metrics.get("minimum_safety_clearance_m"),
        "maximum_absolute_roll_rad": metrics.get("maximum_absolute_roll_rad"),
        "maximum_absolute_pitch_rad": metrics.get("maximum_absolute_pitch_rad"),
        "maximum_angular_speed_rad_s": metrics.get("maximum_angular_speed_rad_s"),
        "minimum_motor_rpm": metrics.get("minimum_motor_rpm"),
        "maximum_motor_rpm": metrics.get("maximum_motor_rpm"),
        "saturation_sample_count": metrics.get("saturation_sample_count"),
        "longest_saturation_duration_s": metrics.get(
            "longest_saturation_duration_s"),
        "saturated_at_end": metrics.get("saturated_at_end"),
        "collision": metrics.get("collision_observed"),
        "non_finite_value_count": nonfinite + metrics.get(
            "non_finite_attitude_count", 0) + metrics.get("non_finite_rpm_count", 0),
        "duration_scales": [item["duration_scale"] for item in segments],
        "velocity_scales": [item["velocity_scale"] for item in segments],
        "refinement_iterations": [item["refinement_iterations"] for item in segments],
        "segment_durations_s": [item["trajectory_duration_s"] for item in segments],
        "mission_start_time_s": 0.0 if complete is not None else None,
        "navigation_phase_start_time_s": navigation_start,
        "goal_arrival_time_s": metrics.get("arrival_time_s"),
        "mission_complete_time_s": complete,
        "recording_stop_time_s": stopped,
        "navigation_execution_time_s": sum(
            item["trajectory_duration_s"] for item in segments) if segments else None,
        "goal_activation_times_s": metrics.get("goal_activation_times_s"),
        "per_goal_arrival_times_s": metrics.get("per_goal_arrival_times_s"),
        "per_goal_duration_s": metrics.get("per_goal_duration_s"),
        "repository_commit": commit, "git_dirty": dirty, "domain_id": domain,
    })
    if not row["success"]:
        row["stop_reason"] = metadata.get("failure_reason") or row["stop_reason"]
    return row


def safety_pass(row, baseline_clearance=None):
    required = (
        row.get("success") is True and row.get("collision") is False and
        row.get("non_finite_value_count") == 0 and
        row.get("minimum_safety_clearance_m") is not None and
        row["minimum_safety_clearance_m"] > 0 and
        row.get("saturated_at_end") is False and
        row.get("final_position_error_m") is not None and
        row["final_position_error_m"] < 0.10 and
        row.get("navigation_tracking_max_error_m") is not None and
        row["navigation_tracking_max_error_m"] < 0.10)
    if not required:
        return False
    clearance = row["minimum_safety_clearance_m"]
    return (clearance >= 0.15 if baseline_clearance is None or baseline_clearance >= 0.15
            else clearance >= baseline_clearance - 0.03)


def aggregate(rows):
    groups = {}
    for row in rows:
        groups.setdefault((row["candidate"], row["route"]), []).append(row)
    result = []
    for (candidate, route), items in sorted(groups.items()):
        times = [item["total_mission_time_s"] for item in items
                 if item.get("total_mission_time_s") is not None]
        result.append({
            "candidate": candidate, "route": route, "runs": len(items),
            "successes": sum(item.get("success") is True for item in items),
            "mission_time_mean_s": statistics.fmean(times) if times else None,
            "mission_time_min_s": min(times) if times else None,
            "mission_time_max_s": max(times) if times else None,
            "mission_time_stddev_s": statistics.pstdev(times) if times else None,
        })
    return result


def select_candidates(rows):
    baseline = [row for row in rows if row["candidate"] == "baseline" and
                row["route"] == "full_map" and row.get("success")]
    if not baseline:
        return []
    baseline_time = statistics.fmean(row["total_mission_time_s"] for row in baseline)
    baseline_clearance = statistics.fmean(
        row["minimum_safety_clearance_m"] for row in baseline)
    selected = []
    candidates = sorted({row["candidate"] for row in rows
                         if row["candidate"] != "baseline"})
    for candidate in candidates:
        full = [row for row in rows if row["candidate"] == candidate and
                row["route"] == "full_map"]
        if len(full) < 3 or not all(safety_pass(row, baseline_clearance) for row in full):
            continue
        mean_time = statistics.fmean(row["total_mission_time_s"] for row in full)
        reduction = baseline_time - mean_time
        if reduction >= 10.0 or reduction / baseline_time >= 0.10:
            selected.append((mean_time, candidate))
    return [candidate for _, candidate in sorted(selected)]


def write_table(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: (json.dumps(row[key], separators=(",", ":"))
                                   if isinstance(row.get(key), list) else row.get(key))
                             for key in CSV_FIELDS})


def read_records(output, round2=False):
    records = []
    root = output / "round2" if round2 else output
    pattern = "*/**/run_*/run_record.json"
    for path in sorted(root.glob(pattern)):
        records.append(json.loads(path.read_text()))
    return records


def summarize(output):
    rows = read_records(output)
    write_table(output / "speed_sweep.csv", rows)
    aggregates = aggregate(rows)
    selected = select_candidates(rows)
    payload = {"generated_at": datetime.now(timezone.utc).isoformat(),
               "runs": rows, "aggregates": aggregates,
               "eligible_candidates": selected}
    (output / "speed_sweep.json").write_text(
        json.dumps(payload, indent=2, allow_nan=False) + "\n")
    lines = ["# Navigation speed sweep", "",
             "All runs are candidate evidence; recorder steady time is excluded from mission time.", "",
             "| candidate | route | success/runs | mean mission time (s) |", "|---|---|---:|---:|"]
    for item in aggregates:
        mean = item["mission_time_mean_s"]
        lines.append(f"| {item['candidate']} | {item['route']} | "
                     f"{item['successes']}/{item['runs']} | "
                     f"{mean:.3f} |" if mean is not None else
                     f"| {item['candidate']} | {item['route']} | "
                     f"{item['successes']}/{item['runs']} | n/a |")
    lines += ["", "Eligible candidates: " + (", ".join(selected) if selected else "none"), ""]
    (output / "README.md").write_text("\n".join(lines))
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        full = [row for row in rows if row["route"] == "full_map"]
        names = [f"{row['candidate']}-{row['run']}" for row in full]
        fig, axes = plt.subplots(2, 2, figsize=(11, 7))
        fields = (("total_mission_time_s", "mission time (s)"),
                  ("actual_max_speed_m_s", "actual max speed (m/s)"),
                  ("navigation_tracking_rms_error_m", "tracking RMS (m)"),
                  ("minimum_safety_clearance_m", "minimum clearance (m)"))
        for axis, (field, label) in zip(axes.flat, fields):
            values = [row.get(field) if row.get(field) is not None else 0 for row in full]
            colors = ["tab:blue" if row.get("success") else "tab:red" for row in full]
            axis.bar(names, values, color=colors); axis.set_ylabel(label)
            axis.tick_params(axis="x", rotation=45)
        fig.tight_layout(); fig.savefig(output / "speed_sweep.png", dpi=150)
        plt.close(fig)
    except ImportError:
        pass
    return rows


def summarize_round2(output):
    rows = read_records(output, round2=True)
    write_table(output / "speed_sweep_round2.csv", rows)
    aggregates = aggregate(rows)
    selected = select_candidates(rows)
    payload = {"generated_at": datetime.now(timezone.utc).isoformat(),
               "runs": rows, "aggregates": aggregates,
               "eligible_candidates": selected}
    (output / "speed_sweep_round2.json").write_text(
        json.dumps(payload, indent=2, allow_nan=False) + "\n")
    lines = ["# Navigation acceleration-margin sweep", "",
             "Recorder steady time is excluded from mission-complete time.", "",
             "| candidate | route | success/runs | mean mission time (s) |",
             "|---|---|---:|---:|"]
    for item in aggregates:
        mean = item["mission_time_mean_s"]
        value = f"{mean:.3f}" if mean is not None else "n/a"
        lines.append(
            f"| {item['candidate']} | {item['route']} | "
            f"{item['successes']}/{item['runs']} | {value} |")
    lines += ["", "Eligible candidates: " + (
        ", ".join(selected) if selected else "none"), ""]
    (output / "round2_summary.md").write_text("\n".join(lines))
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        full = [row for row in rows if row["route"] == "full_map"]
        names = [f"{row['candidate']}-{row['run']}" for row in full]
        fig, axes = plt.subplots(3, 2, figsize=(12, 11))
        fields = (
            ("mission_complete_time_s", "mission complete (s)"),
            ("actual_max_speed_m_s", "actual max speed (m/s)"),
            ("duration_scales", "selected duration scale"),
            ("navigation_tracking_rms_error_m", "tracking RMS (m)"),
            ("navigation_tracking_max_error_m", "tracking max (m)"),
            ("minimum_safety_clearance_m", "minimum clearance (m)"),
        )
        for axis, (field, label) in zip(axes.flat, fields):
            values = []
            for row in full:
                value = row.get(field)
                if isinstance(value, list):
                    value = max(value, default=0)
                values.append(value if value is not None else 0)
            colors = ["tab:blue" if row.get("success") else "tab:red" for row in full]
            axis.bar(names, values, color=colors); axis.set_ylabel(label)
            axis.tick_params(axis="x", rotation=45)
        fig.tight_layout(); fig.savefig(output / "speed_sweep_round2.png", dpi=150)
        plt.close(fig)
        fig, axis = plt.subplots(figsize=(10, 5))
        width = 0.25; positions = list(range(len(full)))
        acceleration_fields = (
            ("reference_max_horizontal_acceleration_m_s2", "horizontal"),
            ("reference_max_vertical_acceleration_m_s2", "vertical"),
            ("reference_max_total_acceleration_m_s2", "total"),
        )
        for offset, (field, label) in zip((-width, 0, width), acceleration_fields):
            axis.bar([position + offset for position in positions],
                     [row.get(field) or 0 for row in full], width, label=label)
        axis.set_xticks(positions, names, rotation=45)
        axis.set_ylabel("reference acceleration (m/s²)"); axis.legend()
        fig.tight_layout(); fig.savefig(
            output / "speed_sweep_round2_acceleration.png", dpi=150)
        plt.close(fig)
    except ImportError:
        pass
    return rows


def run_once(args):
    parameters = candidate_parameters(args.candidate)
    commit, dirty = git_state()
    if dirty and not args.allow_dirty:
        raise SystemExit("refusing candidate run from a dirty worktree")
    domain = args.domain_id if args.domain_id is not None else (
        20 + (int(time.time() * 1000) + args.run) % 180)
    evidence_root = args.output / "round2" if args.round2 else args.output
    run_dir = evidence_root / args.candidate / args.route / f"run_{args.run:02d}"
    run_dir.mkdir(parents=True, exist_ok=True)
    manifest = {"candidate": args.candidate, "route": args.route, "run": args.run,
                "parameters": parameters, "repository_commit": commit,
                "git_dirty": dirty, "ros_domain_id": domain,
                "run_status": "candidate",
                "generated_at": datetime.now(timezone.utc).isoformat()}
    (run_dir / "run_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n")
    config_dir = evidence_root / "configs"
    config_dir.mkdir(parents=True, exist_ok=True)
    (config_dir / f"{args.candidate}.json").write_text(
        json.dumps(parameters, indent=2) + "\n")
    env = os.environ.copy(); env["ROS_DOMAIN_ID"] = str(domain)
    launch_handle = (run_dir / "launch.log").open("w")
    recorder_handle = (run_dir / "recorder_process.log").open("w")
    launch = recorder = None
    try:
        launch = subprocess.Popen(ros_command([
            "ros2", "launch", "drone_bringup", "assessment_navigation_sim.launch.py",
            "use_rviz:=false", "yaw_mode:=path_tangent",
            f"nominal_speed:={parameters['nominal_speed']}",
            f"max_reference_speed:={parameters['max_reference_speed']}",
            f"max_reference_acceleration:={parameters['max_reference_acceleration']}",
        ]), cwd=ROOT, env=env, stdout=launch_handle, stderr=subprocess.STDOUT,
            start_new_session=True)
        recorder = subprocess.Popen(ros_command([
            "python3", str(ROOT / "tools/assessment_recorder.py"),
            "--experiment", "navigation", "--run-status", "candidate",
            "--output", str(run_dir), "--timeout", str(args.timeout),
        ]), cwd=ROOT, env=env, stdout=recorder_handle, stderr=subprocess.STDOUT,
            start_new_session=True)
        time.sleep(args.startup_wait)
        service = subprocess.run(ros_command([
            "ros2", "service", "call", "/drone/interactive_goals/execute",
            "drone_msgs/srv/ExecuteGoalSequence", request_yaml(args.route),
        ]), cwd=ROOT, env=env, text=True, capture_output=True, timeout=45)
        (run_dir / "service.log").write_text(service.stdout + service.stderr)
        normalized_service_output = service.stdout.replace(" ", "").lower()
        accepted = service.returncode == 0 and (
            "accepted:true" in normalized_service_output or
            "accepted=true" in normalized_service_output)
        if not accepted:
            terminate(recorder)
        else:
            recorder.wait(timeout=args.timeout + 20)
    except (subprocess.TimeoutExpired, KeyboardInterrupt) as error:
        (run_dir / "orchestrator_error.log").write_text(str(error) + "\n")
    finally:
        terminate(recorder); terminate(launch)
        launch_handle.close(); recorder_handle.close()
    if (run_dir / "metadata.json").exists():
        subprocess.run(ros_command([
            "python3", str(ROOT / "tools/analyze_assessment_run.py"), str(run_dir),
            "--parameters", str(ROOT / "results/parameters"),
        ]), cwd=ROOT, env=env, text=True,
            stdout=(run_dir / "analyzer.log").open("w"), stderr=subprocess.STDOUT,
            timeout=90, check=False)
    row = build_record(run_dir, args.candidate, args.route, args.run,
                       parameters, commit, dirty, domain)
    (run_dir / "run_record.json").write_text(
        json.dumps(row, indent=2, allow_nan=False) + "\n")
    if args.round2:
        summarize_round2(args.output)
    else:
        summarize(args.output)
    return 0 if row["success"] else 1


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", choices=CANDIDATES)
    parser.add_argument("--route", choices=ROUTES)
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--run-start", type=int, default=1)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--startup-wait", type=float, default=5.0)
    parser.add_argument("--domain-id", type=int)
    parser.add_argument("--allow-dirty", action="store_true",
                        help="Only for infrastructure smoke checks, never evidence runs.")
    parser.add_argument("--summarize", action="store_true")
    parser.add_argument("--round2", action="store_true")
    args = parser.parse_args(argv)
    if args.summarize:
        return args
    if not args.candidate or not args.route:
        parser.error("--candidate and --route are required unless --summarize is used")
    if args.runs <= 0 or args.run_start <= 0:
        parser.error("--runs and --run-start must be positive")
    return args


def main(argv=None):
    args = arguments(argv)
    if args.summarize:
        if args.round2:
            summarize_round2(args.output)
        else:
            summarize(args.output)
        return 0
    status = 0
    for run in range(args.run_start, args.run_start + args.runs):
        args.run = run
        status = max(status, run_once(args))
    return status


if __name__ == "__main__":
    sys.exit(main())
