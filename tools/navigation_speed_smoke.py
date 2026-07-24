#!/usr/bin/env python3
"""Run temporary navigation-speed scenarios and emit comparable JSON metrics."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import statistics
import subprocess
import sys
import time

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = Path("/tmp/ros2_drone_assessment_smoke/navigation_speed")
SCENARIOS = {
    "open": [((8.0, 0.0, 1.5), 0.0)],
    "obstacle": [((13.2, 5.5, 1.5), 0.0)],
    "turning": [
        ((3.5, 1.0, 2.5), math.pi / 2.0),
        ((5.5, 1.0, 4.0), math.pi),
        ((7.0, 5.0, 4.0), -math.pi / 2.0),
    ],
}
TRAJECTORY_RE = re.compile(
    r"ordered goal (?P<goal>\d+) trajectory ready:.*?"
    r"duration=(?P<duration>[0-9.]+) s velocity_scale=(?P<velocity>[0-9.]+) "
    r"duration_scale=(?P<scale>[0-9.]+) max_speed=(?P<speed>[0-9.]+) m/s "
    r"max_acceleration=(?P<accel>[0-9.]+) m/s\^2"
)
# Frozen navigation-tracking acceptance policy (see docs/navigation_speed_validation.md).
# tracking_max_m alone rejected candidates on brief 6-8 cm turn deviations even when the
# rest of the trajectory tracked tightly. The combined distribution/duration judgement below
# replaces the single hard ceiling with maximum, RMS, p95, and how long/how often the error
# actually stays above 5 cm, computed from real per-sample mission_time_s deltas (50 Hz odom
# callbacks), not an assumed fixed period.
TRACKING_OVER_THRESHOLD_M = 0.05
THRESHOLDS = {
    "tracking_max_m": 0.08,
    "tracking_rms_m": 0.025,
    "tracking_p95_m": 0.05,
    "tracking_over_005_fraction": 0.05,
    "tracking_over_005_longest_continuous_s": 0.50,
    "final_position_error_m": 0.10,
    "final_speed_mps": 0.05,
    "minimum_clearance_m": 0.085,
    "maximum_motor_rpm_ratio": 0.85,
    "maximum_tilt_ratio": 0.85,
}


def command(arguments):
    setup = "source /opt/ros/humble/setup.bash"
    setup += f" && source {shlex.quote(str(ROOT / 'install/setup.bash'))}"
    setup += " && exec " + " ".join(shlex.quote(str(item)) for item in arguments)
    return ["bash", "-lc", setup]


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


def git_state():
    commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    branch = subprocess.check_output(
        ["git", "branch", "--show-current"], cwd=ROOT, text=True).strip()
    dirty = bool(subprocess.check_output(
        ["git", "status", "--porcelain"], cwd=ROOT, text=True).strip())
    return commit, branch, dirty


def node_parameters(path):
    return next(iter(yaml.safe_load(path.read_text()).values()))["ros__parameters"]


def request_yaml(goals):
    poses = []
    for (x, y, z), yaw in goals:
        poses.append({
            "position": {"x": x, "y": y, "z": z},
            "orientation": {
                "z": math.sin(0.5 * yaw),
                "w": math.cos(0.5 * yaw),
            },
        })
    return json.dumps({
        "goals": {"header": {"frame_id": "map"}, "poses": poses},
        "draft_revision": 1,
    }, separators=(",", ":"))


def make_open_environment(output):
    source = yaml.safe_load(
        (ROOT / "src/drone_bringup/config/environment.yaml").read_text())
    parameters = next(iter(source.values()))["ros__parameters"]
    # ROS 2 cannot infer the element type of an explicitly empty YAML list.
    # Omitting the override lets each node use its declared vector<double>{}
    # default, which represents the intended obstacle-free environment.
    parameters.pop("obstacles", None)
    path = output / "open_environment.yaml"
    path.write_text(yaml.safe_dump(source, sort_keys=False))
    return path


def finite(row, key):
    value = row.get(key, "")
    if value in ("", None):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def truth(row, key):
    value = finite(row, key)
    return None if value is None else bool(int(value))


def load_rows(path):
    if not path.exists():
        return []
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def path_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def path_segments(path, name):
    if not path.exists():
        return []
    data = json.loads(path.read_text())
    value = data.get(f"{name}_segments", data.get(name, []))
    if isinstance(value, dict):
        value = value.get("segments", [])
    return value if isinstance(value, list) else []


def segment_points(segment):
    if isinstance(segment, dict):
        return segment.get("points", [])
    return segment if isinstance(segment, list) else []


def segments_length(segments):
    return sum(path_length(segment_points(segment)) for segment in segments)


def inflated_boxes(environment_path, inflation):
    values = node_parameters(environment_path).get("obstacles", [])
    boxes = []
    for offset in range(0, len(values), 6):
        x, y, z, sx, sy, sz = map(float, values[offset:offset + 6])
        boxes.append((
            (x - sx / 2.0 - inflation, y - sy / 2.0 - inflation,
             z - sz / 2.0 - inflation),
            (x + sx / 2.0 + inflation, y + sy / 2.0 + inflation,
             z + sz / 2.0 + inflation),
        ))
    return boxes


def segment_intersects_box(start, end, box):
    lower, upper = box
    low, high = 0.0, 1.0
    for axis in range(3):
        delta = end[axis] - start[axis]
        if abs(delta) < 1.0e-12:
            if start[axis] < lower[axis] or start[axis] > upper[axis]:
                return False
            continue
        first = (lower[axis] - start[axis]) / delta
        second = (upper[axis] - start[axis]) / delta
        if first > second:
            first, second = second, first
        low, high = max(low, first), min(high, second)
        if low > high:
            return False
    return True


def collision_count(points, boxes):
    return sum(
        any(segment_intersects_box(a, b, box) for box in boxes)
        for a, b in zip(points, points[1:])
    )


def path_collision_count(segments, boxes):
    """Sum collisions per segment so unrelated segments are never joined."""
    return sum(collision_count(segment_points(segment), boxes) for segment in segments)


def percentile(sorted_values, fraction):
    if not sorted_values:
        return None
    if len(sorted_values) == 1:
        return sorted_values[0]
    position = (len(sorted_values) - 1) * fraction
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return sorted_values[int(position)]
    return (sorted_values[lower] * (upper - position) +
            sorted_values[upper] * (position - lower))


def over_threshold_stats(series, threshold):
    """series: ordered [(mission_time_s, tracking_error_m), ...] from real samples.

    Duration uses a zero-order hold between consecutive real timestamps: the
    interval [t_i, t_{i+1}) is counted as over-threshold when tracking[i] exceeds
    the limit. This avoids assuming a fixed sample period.
    """
    samples_over = sum(1 for _, value in series if value > threshold)
    fraction = samples_over / len(series) if series else None
    duration = 0.0
    longest = 0.0
    current = 0.0
    for (time_a, value_a), (time_b, _) in zip(series, series[1:]):
        delta = max(0.0, time_b - time_a)
        if value_a > threshold:
            duration += delta
            current += delta
            longest = max(longest, current)
        else:
            current = 0.0
    return samples_over, fraction, duration, longest


def first_time(rows, predicate):
    for row in rows:
        if predicate(row):
            value = finite(row, "mission_time_s")
            if value is not None:
                return value
    return None


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else None


def parse_trajectory_log(path):
    text = path.read_text(errors="replace") if path.exists() else ""
    values = []
    for match in TRAJECTORY_RE.finditer(text):
        values.append({
            "goal_index": int(match.group("goal")),
            "duration_s": float(match.group("duration")),
            "selected_velocity_scale": float(match.group("velocity")),
            "selected_duration_scale": float(match.group("scale")),
            "max_reference_speed_mps": float(match.group("speed")),
            "max_reference_acceleration_mps2": float(match.group("accel")),
        })
    return values


def goal_metrics(rows, goals):
    result = ([], [], [], [])
    last_visited = 0
    for row in rows:
        visited = finite(row, "navigation_visited_goals")
        if visited is None or int(visited) <= last_visited:
            continue
        index = int(visited) - 1
        if index >= len(goals):
            continue
        position = [finite(row, f"actual_{axis}") for axis in "xyz"]
        speed = finite(row, "speed")
        yaw = finite(row, "yaw")
        angular = [finite(row, f"angular_speed_{axis}") for axis in "xyz"]
        if all(value is not None for value in position):
            result[0].append(math.dist(position, goals[index][0]))
        else:
            result[0].append(None)
        result[1].append(speed)
        result[2].append(
            None if yaw is None else abs(math.remainder(yaw - goals[index][1], 2 * math.pi)))
        result[3].append(
            None if any(value is None for value in angular) else
            math.sqrt(sum(value * value for value in angular)))
        last_visited = int(visited)
    return result


def consecutive_saturation(diagnostics):
    counts = {name: 0 for name in (
        "horizontal_saturation_samples", "altitude_saturation_samples",
        "attitude_saturation_samples", "mixer_saturation_samples")}
    current = maximum = 0
    ending = False
    for row in diagnostics:
        flags = {
            "horizontal_saturation_samples": truth(row, "horizontal_saturated"),
            "altitude_saturation_samples": truth(row, "altitude_saturated"),
            "attitude_saturation_samples": truth(row, "attitude_saturated"),
            "mixer_saturation_samples": truth(row, "mixer_saturated"),
        }
        ending = any(value is True for value in flags.values())
        for key, value in flags.items():
            counts[key] += int(value is True)
        current = current + 1 if ending else 0
        maximum = max(maximum, current)
    return counts, maximum, ending


def analyze(run_dir, scenario, candidate, parameters, environment_path, commit, branch):
    rows = load_rows(run_dir / "samples.csv")
    diagnostics = load_rows(run_dir / "diagnostics.csv")
    goals = SCENARIOS[scenario]
    trajectories = parse_trajectory_log(run_dir / "launch.log")
    planned = path_segments(run_dir / "paths.json", "planned")
    simplified = path_segments(run_dir / "paths.json", "simplified")
    reference = path_segments(run_dir / "paths.json", "reference")
    navigation_start = min(
        (segment.get("mission_time_s") for segment in planned
         if isinstance(segment, dict) and segment.get("points") and
         segment.get("mission_time_s") is not None),
        default=None)
    if navigation_start is None:
        navigation_start = first_time(
            rows, lambda row: finite(row, "tracking_error") is not None and
            (row.get("navigation_goal_index", "") != ""))
    navigation_rows = [
        row for row in rows
        if finite(row, "mission_time_s") is not None and
        navigation_start is not None and finite(row, "mission_time_s") >= navigation_start
    ]
    actual_points = [
        [finite(row, f"actual_{axis}") for axis in "xyz"] for row in navigation_rows]
    actual_points = [
        point for point in actual_points if all(value is not None for value in point)]
    mission_complete = first_time(rows, lambda row: truth(row, "navigation_complete") is True)
    metadata = json.loads((run_dir / "metadata.json").read_text()) \
        if (run_dir / "metadata.json").exists() else {}
    events = load_rows(run_dir / "events.csv")
    stable_confirmed = None
    for event in events:
        if event.get("event") == "recording_stopped":
            stable_confirmed = finite(event, "mission_time_s")
    speeds = [finite(row, "speed") for row in navigation_rows]
    speeds = [value for value in speeds if value is not None]
    reference_speeds = []
    reference_accelerations = []
    commanded_accelerations = []
    tracking = []
    clearances = []
    rpms = []
    rolls, pitches = [], []
    non_finite_count = 0
    core_fields = (
        "actual_x", "actual_y", "actual_z", "speed", "roll", "pitch", "yaw",
        "angular_speed_x", "angular_speed_y", "angular_speed_z")
    tracking_series = []
    tracking_peak = None
    for row in navigation_rows:
        non_finite_count += sum(
            row.get(key, "") != "" and finite(row, key) is None for key in core_fields)
        velocity = [finite(row, f"reference_velocity_{axis}") for axis in "xyz"]
        acceleration = [finite(row, f"reference_acceleration_{axis}") for axis in "xyz"]
        commanded = [
            finite(row, "commanded_horizontal_acceleration_x"),
            finite(row, "commanded_horizontal_acceleration_y")]
        reference_speed = math.sqrt(sum(value * value for value in velocity)) \
            if all(value is not None for value in velocity) else None
        reference_acceleration = math.sqrt(sum(value * value for value in acceleration)) \
            if all(value is not None for value in acceleration) else None
        commanded_acceleration = math.hypot(*commanded) \
            if all(value is not None for value in commanded) else None
        if reference_speed is not None:
            reference_speeds.append(reference_speed)
        if reference_acceleration is not None:
            reference_accelerations.append(reference_acceleration)
        if commanded_acceleration is not None:
            commanded_accelerations.append(commanded_acceleration)
        value = finite(row, "tracking_error")
        row_time = finite(row, "mission_time_s")
        if value is not None:
            tracking.append(value)
            if row_time is not None:
                tracking_series.append((row_time, value))
            if tracking_peak is None or value > tracking_peak["tracking_max_m"]:
                tracking_peak = {
                    "tracking_max_m": value,
                    "tracking_max_time_s": row_time,
                    "tracking_max_actual_position": [
                        finite(row, f"actual_{axis}") for axis in "xyz"],
                    "tracking_max_reference_position": [
                        finite(row, f"reference_{axis}") for axis in "xyz"],
                    "tracking_max_goal_index": finite(row, "navigation_goal_index"),
                    "tracking_max_actual_speed_mps": finite(row, "speed"),
                    "tracking_max_reference_speed_mps": reference_speed,
                    "tracking_max_reference_acceleration_mps2": reference_acceleration,
                    "tracking_max_commanded_horizontal_acceleration_mps2": commanded_acceleration,
                    "tracking_max_safety_clearance_m": finite(row, "safety_clearance"),
                    "tracking_max_roll_rad": finite(row, "roll"),
                    "tracking_max_pitch_rad": finite(row, "pitch"),
                    "tracking_max_saturation_state": {
                        "horizontal": truth(row, "horizontal_saturated"),
                        "altitude": truth(row, "altitude_saturated"),
                        "attitude": truth(row, "attitude_saturated"),
                        "mixer": truth(row, "mixer_saturated"),
                    },
                }
        value = finite(row, "safety_clearance")
        if value is not None:
            clearances.append(value)
        for motor in range(1, 5):
            value = finite(row, f"commanded_motor_rpm_m{motor}")
            if value is not None:
                rpms.append(value)
        value = finite(row, "roll")
        if value is not None:
            rolls.append(abs(value))
        value = finite(row, "pitch")
        if value is not None:
            pitches.append(abs(value))
    saturation, max_consecutive, ending_saturated = consecutive_saturation(diagnostics)
    final = rows[-1] if rows else {}
    final_position_error = finite(final, "goal_position_error")
    final_speed = finite(final, "speed")
    goal_position_errors, goal_speeds, goal_yaw_errors, goal_angular_speeds = \
        goal_metrics(rows, goals)
    environment = node_parameters(environment_path)
    safety_boxes = inflated_boxes(environment_path, float(environment["safety_radius"]))
    planned_collisions = path_collision_count(planned, safety_boxes)
    simplified_collisions = path_collision_count(simplified, safety_boxes)
    reference_collisions = path_collision_count(reference, safety_boxes)
    actual_collisions = collision_count(actual_points, safety_boxes)
    actual_collisions += int(any(truth(row, "collision_state") is True for row in rows))
    collisions = (
        planned_collisions + simplified_collisions + reference_collisions + actual_collisions)
    sorted_tracking = sorted(tracking)
    tracking_p90 = percentile(sorted_tracking, 0.90)
    tracking_p95 = percentile(sorted_tracking, 0.95)
    tracking_p99 = percentile(sorted_tracking, 0.99)
    tracking_rms = rms(tracking)
    over_samples, over_fraction, over_duration, over_longest = over_threshold_stats(
        sorted(tracking_series, key=lambda item: item[0]), TRACKING_OVER_THRESHOLD_M)
    max_rpm = max(rpms, default=None)
    max_roll = max(rolls, default=None)
    max_pitch = max(pitches, default=None)
    max_tilt = max([value for value in (max_roll, max_pitch) if value is not None],
                   default=None)
    failure_reasons = []
    visited = max(
        (int(value) for row in rows
         if (value := finite(row, "navigation_visited_goals")) is not None),
        default=0)
    if mission_complete is None or visited != len(goals):
        failure_reasons.append("mission did not complete every ordered goal")
    if collisions:
        failure_reasons.append(
            f"collision_count={collisions} "
            f"(planned={planned_collisions} simplified={simplified_collisions} "
            f"reference={reference_collisions} actual={actual_collisions})")
    if non_finite_count:
        failure_reasons.append(f"non_finite_count={non_finite_count}")
    if tracking and max(tracking) >= THRESHOLDS["tracking_max_m"]:
        failure_reasons.append(
            f"tracking_max_m={max(tracking):.6f} >= {THRESHOLDS['tracking_max_m']}")
    if tracking_rms is not None and tracking_rms >= THRESHOLDS["tracking_rms_m"]:
        failure_reasons.append(
            f"tracking_rms_m={tracking_rms:.6f} >= {THRESHOLDS['tracking_rms_m']}")
    if tracking_p95 is not None and tracking_p95 >= THRESHOLDS["tracking_p95_m"]:
        failure_reasons.append(
            f"tracking_p95_m={tracking_p95:.6f} >= {THRESHOLDS['tracking_p95_m']}")
    if over_fraction is not None and over_fraction >= THRESHOLDS["tracking_over_005_fraction"]:
        failure_reasons.append(
            f"tracking_over_005_fraction={over_fraction:.4f} >= "
            f"{THRESHOLDS['tracking_over_005_fraction']}")
    if over_longest >= THRESHOLDS["tracking_over_005_longest_continuous_s"]:
        failure_reasons.append(
            f"tracking_over_005_longest_continuous_s={over_longest:.3f} >= "
            f"{THRESHOLDS['tracking_over_005_longest_continuous_s']}")
    if final_position_error is None or \
            final_position_error >= THRESHOLDS["final_position_error_m"]:
        failure_reasons.append("final position error failed")
    if final_speed is None or final_speed >= THRESHOLDS["final_speed_mps"]:
        failure_reasons.append("final speed failed")
    if scenario != "open" and (
            not clearances or min(clearances) < THRESHOLDS["minimum_clearance_m"]):
        failure_reasons.append("minimum clearance failed")
    if any(saturation.values()):
        failure_reasons.append("navigation saturation samples must remain zero")
    if ending_saturated:
        failure_reasons.append("controller ended saturated")
    if max_rpm is None or max_rpm > parameters["max_rpm"]:
        failure_reasons.append("RPM hard limit failed")
    if max_rpm is not None and max_rpm / parameters["max_rpm"] > \
            THRESHOLDS["maximum_motor_rpm_ratio"]:
        failure_reasons.append("RPM margin failed")
    if max_tilt is not None and max_tilt / parameters["max_tilt_angle"] > \
            THRESHOLDS["maximum_tilt_ratio"]:
        failure_reasons.append("tilt margin failed")
    result = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "git_commit": commit,
        "branch": branch,
        "scenario": scenario,
        "candidate_name": candidate,
        "nominal_speed": parameters["nominal_speed"],
        "max_reference_speed": parameters["max_reference_speed"],
        "max_reference_acceleration": parameters["max_reference_acceleration"],
        "min_segment_duration": parameters["min_segment_duration"],
        "max_horizontal_acceleration": parameters["max_horizontal_acceleration"],
        "max_tilt_angle": parameters["max_tilt_angle"],
        "mass": parameters["mass"],
        "max_rpm": parameters["max_rpm"],
        "task_accepted_time": 0.0 if rows else None,
        "navigation_start_time": navigation_start,
        "mission_complete_time": mission_complete,
        "stable_confirmed_time": stable_confirmed,
        "task_time_s": mission_complete,
        "navigation_time_s": (
            mission_complete - navigation_start
            if mission_complete is not None and navigation_start is not None else None),
        "planned_path_length_m": segments_length(planned),
        "simplified_path_length_m": segments_length(simplified),
        "reference_path_length_m": segments_length(reference),
        "actual_path_length_m": path_length(actual_points),
        "actual_reference_path_ratio": (
            path_length(actual_points) / segments_length(reference)
            if segments_length(reference) > 0.0 else None),
        "max_reference_speed_mps": max(reference_speeds, default=None),
        "max_actual_speed_mps": max(speeds, default=None),
        "mean_navigation_speed_mps": statistics.fmean(speeds) if speeds else None,
        "max_reference_acceleration_mps2": max(reference_accelerations, default=None),
        "max_commanded_horizontal_acceleration_mps2": max(
            commanded_accelerations, default=None),
        "tracking_max_m": max(tracking, default=None),
        "tracking_rms_m": tracking_rms,
        "tracking_p90_m": tracking_p90,
        "tracking_p95_m": tracking_p95,
        "tracking_p99_m": tracking_p99,
        "tracking_over_005_samples": over_samples,
        "tracking_over_005_fraction": over_fraction,
        "tracking_over_005_duration_s": over_duration,
        "tracking_over_005_longest_continuous_s": over_longest,
        **(tracking_peak or {
            "tracking_max_time_s": None, "tracking_max_actual_position": None,
            "tracking_max_reference_position": None, "tracking_max_goal_index": None,
            "tracking_max_actual_speed_mps": None, "tracking_max_reference_speed_mps": None,
            "tracking_max_reference_acceleration_mps2": None,
            "tracking_max_commanded_horizontal_acceleration_mps2": None,
            "tracking_max_safety_clearance_m": None, "tracking_max_roll_rad": None,
            "tracking_max_pitch_rad": None, "tracking_max_saturation_state": None,
        }),
        "final_position_error_m": final_position_error,
        "final_speed_mps": final_speed,
        "minimum_clearance_m": min(clearances, default=None),
        "planned_path_collision_count": planned_collisions,
        "simplified_path_collision_count": simplified_collisions,
        "reference_path_collision_count": reference_collisions,
        "actual_path_collision_count": actual_collisions,
        "collision_count": collisions,
        "non_finite_count": non_finite_count,
        "maximum_motor_rpm": max_rpm,
        "maximum_motor_rpm_ratio": (
            max_rpm / parameters["max_rpm"] if max_rpm is not None else None),
        "maximum_roll_rad": max_roll,
        "maximum_pitch_rad": max_pitch,
        "maximum_tilt_ratio": (
            max_tilt / parameters["max_tilt_angle"] if max_tilt is not None else None),
        **saturation,
        "maximum_consecutive_saturation_samples": max_consecutive,
        "ending_saturated": ending_saturated,
        "selected_duration_scales": [
            item["selected_duration_scale"] for item in trajectories],
        "selected_velocity_scales": [
            item["selected_velocity_scale"] for item in trajectories],
        "goal_position_errors": goal_position_errors,
        "goal_speeds": goal_speeds,
        "goal_yaw_errors": goal_yaw_errors,
        "goal_angular_speeds": goal_angular_speeds,
        "pass": not failure_reasons,
        "failure_reasons": failure_reasons,
        "recorder_stop_reason": metadata.get("stop_reason"),
        "trajectory_diagnostics": trajectories,
        "acceptance_thresholds": THRESHOLDS,
    }
    return result


def format_value(value, digits=3):
    return "n/a" if value is None else f"{value:.{digits}f}"


def update_comparison(output):
    records = []
    for path in output.glob("*/*/*/result.json"):
        try:
            records.append(json.loads(path.read_text()))
        except (json.JSONDecodeError, OSError):
            pass
    records.sort(key=lambda row: (row["candidate_name"], row["scenario"], row["timestamp"]))
    (output / "comparison.json").write_text(json.dumps(
        {"generated_at": datetime.now(timezone.utc).isoformat(), "runs": records},
        indent=2, allow_nan=False) + "\n")
    lines = [
        "# Navigation speed smoke comparison", "",
        "| candidate | scenario | pass | task s | max speed | tracking max | "
        "tracking p95 | tracking RMS | over 5cm frac | longest over 5cm | "
        "clearance | RPM ratio | tilt ratio | saturation |",
        "|---|---|:---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in records:
        saturation = sum(row[key] for key in (
            "horizontal_saturation_samples", "altitude_saturation_samples",
            "attitude_saturation_samples", "mixer_saturation_samples"))
        lines.append(
            f"| {row['candidate_name']} | {row['scenario']} | "
            f"{'PASS' if row['pass'] else 'FAIL'} | "
            f"{format_value(row['task_time_s'])} | "
            f"{format_value(row['max_actual_speed_mps'])} | "
            f"{format_value(row['tracking_max_m'], 4)} | "
            f"{format_value(row.get('tracking_p95_m'), 4)} | "
            f"{format_value(row['tracking_rms_m'], 4)} | "
            f"{format_value(row.get('tracking_over_005_fraction'), 4)} | "
            f"{format_value(row.get('tracking_over_005_longest_continuous_s'), 3)} | "
            f"{format_value(row['minimum_clearance_m'], 4)} | "
            f"{format_value(row['maximum_motor_rpm_ratio'], 3)} | "
            f"{format_value(row['maximum_tilt_ratio'], 3)} | {saturation} |")
    (output / "comparison.md").write_text("\n".join(lines) + "\n")


def run_once(args, scenario):
    commit, branch, dirty = git_state()
    if dirty and not args.allow_dirty:
        raise SystemExit("refusing a smoke run from a dirty worktree; commit infrastructure first")
    dynamics = node_parameters(ROOT / "src/drone_bringup/config/dynamics.yaml")
    controller = node_parameters(ROOT / "src/drone_bringup/config/controller.yaml")
    trajectory = node_parameters(ROOT / "src/drone_bringup/config/planned_trajectory.yaml")
    parameters = {
        "nominal_speed": args.nominal_speed or float(trajectory["nominal_speed"]),
        "max_reference_speed": args.max_reference_speed or
        float(trajectory["max_reference_speed"]),
        "max_reference_acceleration": args.max_reference_acceleration or
        float(trajectory["max_reference_acceleration"]),
        "min_segment_duration": args.min_segment_duration or
        float(trajectory["min_segment_duration"]),
        "max_horizontal_acceleration": args.max_horizontal_acceleration or
        float(controller["max_horizontal_acceleration"]),
        "max_tilt_angle": args.max_tilt_angle or float(controller["max_tilt_angle"]),
        "mass": float(dynamics["mass"]),
        "max_rpm": float(dynamics["max_rpm"]),
    }
    if parameters["max_reference_acceleration"] >= \
            parameters["max_horizontal_acceleration"]:
        raise SystemExit(
            "max_reference_acceleration must remain below max_horizontal_acceleration")
    physical_acceleration = float(dynamics["gravity"]) * math.tan(
        parameters["max_tilt_angle"])
    if parameters["max_horizontal_acceleration"] > physical_acceleration:
        raise SystemExit(
            "max_horizontal_acceleration exceeds g*tan(max_tilt_angle)")
    stamp = datetime.now().strftime("%Y%m%dT%H%M%S")
    run_dir = args.output / args.candidate / scenario / stamp
    run_dir.mkdir(parents=True)
    environment_path = (
        make_open_environment(run_dir) if scenario == "open" else
        ROOT / "src/drone_bringup/config/environment.yaml")
    domain = args.domain_id or (20 + int(time.time() * 1000) % 180)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain)
    launch_log = (run_dir / "launch.log").open("w")
    recorder_log = (run_dir / "recorder_process.log").open("w")
    launch = recorder = None
    try:
        launch_file = (
            "interactive_goal_navigation_sim.launch.py" if scenario == "open"
            else "assessment_navigation_sim.launch.py")
        launch_arguments = [
            "ros2", "launch", "drone_bringup", launch_file,
            "use_rviz:=false", "yaw_mode:=path_tangent",
            f"nominal_speed:={parameters['nominal_speed']}",
            f"min_segment_duration:={parameters['min_segment_duration']}",
            f"max_reference_speed:={parameters['max_reference_speed']}",
            f"max_reference_acceleration:={parameters['max_reference_acceleration']}",
            f"max_horizontal_acceleration:={parameters['max_horizontal_acceleration']}",
            f"max_tilt_angle:={parameters['max_tilt_angle']}",
        ]
        if scenario == "open":
            launch_arguments.append(f"environment_config:={environment_path}")
        launch = subprocess.Popen(
            command(launch_arguments), cwd=ROOT, env=env, stdout=launch_log,
            stderr=subprocess.STDOUT, start_new_session=True)
        recorder_arguments = [
            "python3", str(ROOT / "tools/assessment_recorder.py"),
            "--experiment", "navigation", "--run-status", "smoke",
            "--output", str(run_dir), "--overwrite-existing",
            "--environment-config", str(environment_path),
            "--timeout", str(args.timeout), "--steady-window", "2.0",
        ]
        for position, yaw in SCENARIOS[scenario]:
            recorder_arguments += [
                "--expected-goal", *map(str, (*position, yaw))]
        recorder = subprocess.Popen(
            command(recorder_arguments), cwd=ROOT, env=env, stdout=recorder_log,
            stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(args.startup_wait)
        service = subprocess.run(command([
            "ros2", "service", "call", "/drone/interactive_goals/execute",
            "drone_msgs/srv/ExecuteGoalSequence", request_yaml(SCENARIOS[scenario]),
        ]), cwd=ROOT, env=env, text=True, capture_output=True, timeout=45)
        (run_dir / "service.log").write_text(service.stdout + service.stderr)
        normalized = service.stdout.replace(" ", "").lower()
        if service.returncode != 0 or (
                "accepted:true" not in normalized and "accepted=true" not in normalized):
            terminate(recorder)
        else:
            recorder.wait(timeout=args.timeout + 20)
    except (subprocess.TimeoutExpired, KeyboardInterrupt) as error:
        (run_dir / "orchestrator_error.log").write_text(str(error) + "\n")
    finally:
        terminate(recorder)
        terminate(launch)
        launch_log.close()
        recorder_log.close()
    result = analyze(
        run_dir, scenario, args.candidate, parameters, environment_path, commit, branch)
    result["ros_domain_id"] = domain
    result["run_directory"] = str(run_dir)
    (run_dir / "result.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n")
    update_comparison(args.output)
    print(json.dumps(result, indent=2, allow_nan=False))
    return result["pass"]


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=(*SCENARIOS, "all"))
    parser.add_argument("--candidate", default="baseline")
    parser.add_argument("--nominal-speed", type=float)
    parser.add_argument("--max-reference-speed", type=float)
    parser.add_argument("--max-reference-acceleration", type=float)
    parser.add_argument("--min-segment-duration", type=float)
    parser.add_argument("--max-horizontal-acceleration", type=float)
    parser.add_argument("--max-tilt-angle", type=float)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--startup-wait", type=float, default=5.0)
    parser.add_argument("--domain-id", type=int)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--allow-dirty", action="store_true")
    args = parser.parse_args(argv)
    numeric = [
        value for name, value in vars(args).items()
        if isinstance(value, float) and name != "max_tilt_angle"]
    if any(not math.isfinite(value) or value <= 0.0 for value in numeric):
        parser.error("numeric options must be finite and positive")
    if args.max_tilt_angle is not None and (
            not math.isfinite(args.max_tilt_angle) or args.max_tilt_angle <= 0.0):
        parser.error("--max-tilt-angle must be finite and positive")
    if args.domain_id is not None and not 0 <= args.domain_id <= 232:
        parser.error("--domain-id must be in 0..232")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*", args.candidate):
        parser.error("unsafe candidate name")
    return args


def main(argv=None):
    args = arguments(argv)
    scenarios = SCENARIOS if args.scenario == "all" else (args.scenario,)
    passed = True
    for scenario in scenarios:
        passed = run_once(args, scenario) and passed
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
