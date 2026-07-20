#!/usr/bin/env python3
"""Run and summarize candidate-only corner-aware trajectory timing experiments."""

import argparse
import csv
import json
import math
import os
import re
import statistics
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import yaml

import run_navigation_speed_sweep as navigation_sweep


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = Path("/tmp/ros2_drone_trajectory_curvature_timing")
BASELINE_TIME_S = 67.42
LOCAL_RADIUS_M = 0.60
CORNERS = ((3.60, 1.85), (7.60, -1.15), (9.10, 0.35),
           (9.85, 0.60), (10.60, 1.35))
CANDIDATES = {
    "t0": {"enabled": False, "max_scale": 1.00},
    "t10": {"enabled": True, "max_scale": 1.10},
    "t20": {"enabled": True, "max_scale": 1.20},
    "t30": {"enabled": True, "max_scale": 1.30},
}
ROUTES = navigation_sweep.ROUTES
MAIN_TIMING_RE = re.compile(
    r"ordered goal (?P<goal>\d+) trajectory ready:.*?"
    r"corner_timing_enabled=(?P<enabled>true|false) "
    r"maximum_turn_angle_deg=(?P<angle>[0-9.]+) "
    r"maximum_corner_duration_scale=(?P<corner>[0-9.]+) "
    r"corner_adjusted_segment_count=(?P<count>\d+) "
    r"maximum_segment_corner_scale=(?P<segment>[0-9.]+) "
    r"global_duration_scale=(?P<global>[0-9.]+)")
SEGMENT_TIMING_RE = re.compile(
    r"trajectory timing segment: goal=(?P<goal>\d+) segment_index=(?P<segment>\d+) "
    r"segment_length=(?P<length>[0-9.]+) start_corner_angle=(?P<start>[0-9.]+) "
    r"end_corner_angle=(?P<end>[0-9.]+) corner_scale=(?P<scale>[0-9.]+) "
    r"base_duration=(?P<base>[0-9.]+) "
    r"corner_adjusted_duration=(?P<adjusted>[0-9.]+) "
    r"final_duration=(?P<final>[0-9.]+)")

CSV_FIELDS = (
    "candidate route run max_corner_duration_scale success_runs mission_time_mean "
    "success mission_time_s baseline_improvement_percent reference_total_duration "
    "maximum_turn_angle corner_adjusted_segment_count maximum_applied_corner_scale "
    "selected_global_duration_scale reference_jerk_max worst_corner_reference_jerk "
    "actual_jerk_max tracking_max tracking_rms cross_track_max cross_track_rms "
    "reference_minimum_clearance actual_minimum_clearance reference_hausdorff_to_t0 "
    "maximum_roll maximum_pitch maximum_yaw_rate "
    "maximum_actual_horizontal_acceleration collision saturation nonfinite "
    "planned_point_count simplified_point_count simplified_path_length "
    "turning_angles corner_scales segment_corner_scales base_durations "
    "corner_adjusted_durations final_durations local_corners selection "
    "repository_commit git_dirty domain_id"
).split()


def parameters(candidate):
    item = CANDIDATES[candidate]
    return {
        "nominal_speed": 0.50,
        "max_reference_speed": 0.90,
        "max_reference_acceleration": 0.60,
        "corner_timing_enabled": item["enabled"],
        "corner_timing_start_angle_deg": 25.0,
        "corner_timing_full_angle_deg": 70.0,
        "corner_timing_max_duration_scale": item["max_scale"],
    }


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def stats(values):
    values = [value for value in values if finite(value)]
    return {
        "max": max(values) if values else None,
        "rms": math.sqrt(sum(value * value for value in values) / len(values))
        if values else None,
    }


def vector_norm(values):
    return math.sqrt(sum(value * value for value in values))


def differentiate(samples, field):
    output = []
    for before, after in zip(samples, samples[1:]):
        dt = after["time"] - before["time"]
        if not finite(dt) or dt <= 1.0e-4 or dt > 0.10:
            continue
        values = [(after[field][axis] - before[field][axis]) / dt for axis in range(3)]
        if all(finite(value) for value in values):
            output.append({"time": 0.5 * (before["time"] + after["time"]),
                           "value": values, "position": after["position"]})
    return output


def point_segment_distance(point, start, end):
    delta = [end[index] - start[index] for index in range(3)]
    denominator = sum(value * value for value in delta)
    if denominator <= 1.0e-24:
        return math.dist(point, start)
    fraction = max(0.0, min(1.0, sum(
        (point[index] - start[index]) * delta[index] for index in range(3)) /
        denominator))
    projection = [start[index] + fraction * delta[index] for index in range(3)]
    return math.dist(point, projection)


def point_path_distance(point, path):
    return min((point_segment_distance(point, start, end)
                for start, end in zip(path, path[1:])), default=None)


def hausdorff(first, second):
    if len(first) < 2 or len(second) < 2:
        return None
    return max(max(point_path_distance(point, second) for point in first),
               max(point_path_distance(point, first) for point in second))


def parse_timing_log(text):
    goals = [{"goal_index": int(match.group("goal")),
              "enabled": match.group("enabled") == "true",
              "maximum_turn_angle_deg": float(match.group("angle")),
              "maximum_corner_duration_scale": float(match.group("corner")),
              "corner_adjusted_segment_count": int(match.group("count")),
              "maximum_segment_corner_scale": float(match.group("segment")),
              "selected_global_duration_scale": float(match.group("global"))}
             for match in MAIN_TIMING_RE.finditer(text)]
    segments = [{"goal_index": int(match.group("goal")),
                 "segment_index": int(match.group("segment")),
                 "segment_length": float(match.group("length")),
                 "start_corner_angle": float(match.group("start")),
                 "end_corner_angle": float(match.group("end")),
                 "corner_scale": float(match.group("scale")),
                 "base_duration": float(match.group("base")),
                 "corner_adjusted_duration": float(match.group("adjusted")),
                 "final_duration": float(match.group("final"))}
                for match in SEGMENT_TIMING_RE.finditer(text)]
    return goals, segments


def path_segments(payload, name):
    return payload.get(name + "_segments", [])


def combined_path(payload, name):
    output = []
    for segment in path_segments(payload, name):
        points = segment.get("points", [])
        output.extend(points if not output else points[1:])
    return output


def environment_boxes():
    payload = yaml.safe_load(
        (ROOT / "results/parameters/environment.yaml").read_text())
    values = next(iter(payload.values()))["ros__parameters"]["obstacles"]
    boxes = []
    for index in range(0, len(values), 6):
        x, y, z, sx, sy, sz = map(float, values[index:index + 6])
        boxes.append(((x - sx / 2, y - sy / 2, z - sz / 2),
                      (x + sx / 2, y + sy / 2, z + sz / 2)))
    return boxes


def point_box_distance(point, box):
    lower, upper = box
    return math.sqrt(sum(max(lower[index] - point[index], 0.0,
                             point[index] - upper[index]) ** 2
                         for index in range(3)))


def reference_clearance(points):
    boxes = environment_boxes()
    return min((min(point_box_distance(point, box) for box in boxes) - 0.25
                for point in points), default=None)


def read_samples(path, navigation_start):
    samples = []
    nonfinite = 0
    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            try:
                mission_time = float(row["mission_time_s"])
            except (KeyError, TypeError, ValueError):
                continue
            if navigation_start is None or mission_time < navigation_start:
                continue
            try:
                sample = {
                    "time": mission_time,
                    "reference_position": [float(row[f"reference_{axis}"]) for axis in "xyz"],
                    "position": [float(row[f"actual_{axis}"]) for axis in "xyz"],
                    "reference_velocity": [float(row[f"reference_velocity_{axis}"])
                                           for axis in "xyz"],
                    "velocity": [float(row[f"velocity_{axis}"]) for axis in "xyz"],
                    "reference_acceleration": [
                        float(row[f"reference_acceleration_{axis}"]) for axis in "xyz"],
                    "tracking": float(row["tracking_error"]),
                    "roll": float(row["roll"]), "pitch": float(row["pitch"]),
                    "yaw_rate": float(row["angular_speed_z"]),
                    "saturated": any(float(row[field]) == 1.0 for field in (
                        "horizontal_saturated", "altitude_saturated",
                        "attitude_saturated", "mixer_saturated")),
                }
            except (KeyError, TypeError, ValueError):
                nonfinite += 1
                continue
            if not all(finite(value) for key, value in sample.items()
                       if key not in {"saturated", "reference_position", "position",
                                      "reference_velocity", "velocity",
                                      "reference_acceleration"}) or not all(
                    finite(value) for field in ("reference_position", "position",
                                                "reference_velocity", "velocity",
                                                "reference_acceleration")
                    for value in sample[field]):
                nonfinite += 1
                continue
            samples.append(sample)
    return samples, nonfinite


def local_corner_metrics(samples, reference_jerk, actual_acceleration, actual_jerk,
                         reference_path, simplified_path, timing_segments):
    result = []
    boxes = environment_boxes()
    for x, y in CORNERS:
        local = [sample for sample in samples if math.hypot(
            sample["reference_position"][0] - x,
            sample["reference_position"][1] - y) <= LOCAL_RADIUS_M]
        local_reference_jerk = [item for item in reference_jerk if math.hypot(
            item["position"][0] - x, item["position"][1] - y) <= LOCAL_RADIUS_M]
        local_actual_jerk = [item for item in actual_jerk if math.hypot(
            item["position"][0] - x, item["position"][1] - y) <= LOCAL_RADIUS_M]
        local_actual_acceleration = [item for item in actual_acceleration if math.hypot(
            item["position"][0] - x, item["position"][1] - y) <= LOCAL_RADIUS_M]
        nearest_waypoint = min(range(len(simplified_path)), key=lambda index: math.hypot(
            simplified_path[index][0] - x, simplified_path[index][1] - y)
            ) if simplified_path else None
        adjacent = [item for item in timing_segments if nearest_waypoint is not None and
                    item["segment_index"] in {nearest_waypoint - 1, nearest_waypoint}]
        turn_angle = None
        if nearest_waypoint is not None and timing_segments:
            if nearest_waypoint == 0:
                turn_angle = timing_segments[0]["start_corner_angle"]
            elif nearest_waypoint - 1 < len(timing_segments):
                turn_angle = timing_segments[nearest_waypoint - 1]["end_corner_angle"]
        local_cross_track = [point_path_distance(item["position"], reference_path)
                             for item in local] if len(reference_path) >= 2 else []
        result.append({
            "center": [x, y], "sample_count": len(local),
            "turn_angle": turn_angle,
            "segment_corner_scale": max((item["corner_scale"] for item in adjacent),
                                        default=None),
            "reference_speed_min": min((vector_norm(item["reference_velocity"])
                                        for item in local), default=None),
            "reference_speed_max": max((vector_norm(item["reference_velocity"])
                                        for item in local), default=None),
            "actual_speed_min": min((vector_norm(item["velocity"])
                                     for item in local), default=None),
            "actual_speed_max": max((vector_norm(item["velocity"])
                                     for item in local), default=None),
            "reference_acceleration_max": max((vector_norm(
                item["reference_acceleration"]) for item in local), default=None),
            "actual_acceleration_max": max((vector_norm(item["value"])
                                            for item in local_actual_acceleration),
                                           default=None),
            "reference_jerk_max": max((vector_norm(item["value"])
                                       for item in local_reference_jerk), default=None),
            "actual_jerk_max": max((vector_norm(item["value"])
                                    for item in local_actual_jerk), default=None),
            "tracking_max": stats([item["tracking"] for item in local])["max"],
            "tracking_rms": stats([item["tracking"] for item in local])["rms"],
            "cross_track_max": stats(local_cross_track)["max"],
            "cross_track_rms": stats(local_cross_track)["rms"],
            "reference_clearance": min((min(point_box_distance(
                item["reference_position"], box) for box in boxes) - 0.25
                for item in local), default=None),
            "actual_clearance": min((min(point_box_distance(
                item["position"], box) for box in boxes) - 0.25
                for item in local), default=None),
        })
    return result


def analyze_run(run_dir, candidate, route, run, params, commit, dirty, domain):
    base = navigation_sweep.build_record(
        run_dir, candidate, route, run, params, commit, dirty, domain)
    summary_path = run_dir / "summary.json"
    paths_path = run_dir / "paths.json"
    if not summary_path.exists() or not paths_path.exists():
        return {**base, "candidate": candidate, "route": route, "run": run}
    summary = json.loads(summary_path.read_text())
    paths = json.loads(paths_path.read_text())
    samples, extra_nonfinite = read_samples(
        run_dir / "samples.csv", summary.get("navigation_phase_start_time_s"))
    reference_jerk = differentiate([
        {"time": sample["time"], "reference_acceleration":
         sample["reference_acceleration"], "position": sample["reference_position"]}
        for sample in samples], "reference_acceleration")
    actual_acceleration = differentiate(samples, "velocity")
    actual_jerk = differentiate([
        {"time": item["time"], "acceleration": item["value"],
         "position": item["position"]} for item in actual_acceleration], "acceleration")
    reference = combined_path(paths, "reference")
    simplified = combined_path(paths, "simplified")
    planned = combined_path(paths, "planned")
    cross_track = [point_path_distance(sample["position"], reference)
                   for sample in samples] if len(reference) >= 2 else []
    launch_text = (run_dir / "launch.log").read_text(errors="replace")
    timing_goals, timing_segments = parse_timing_log(launch_text)
    tracking = stats([sample["tracking"] for sample in samples])
    cross = stats(cross_track)
    jerk = stats([vector_norm(item["value"]) for item in reference_jerk])
    actual_jerk_stats = stats([vector_norm(item["value"]) for item in actual_jerk])
    actual_horizontal_acceleration = stats([
        math.hypot(item["value"][0], item["value"][1])
        for item in actual_acceleration])
    metrics = summary["metrics"]
    local = local_corner_metrics(
        samples, reference_jerk, actual_acceleration, actual_jerk,
        reference, simplified, timing_segments)
    row = {
        **base, "candidate": candidate, "route": route, "run": run,
        "max_corner_duration_scale": params["corner_timing_max_duration_scale"],
        "mission_time_s": base.get("total_mission_time_s"),
        "baseline_improvement_percent": (100.0 * (BASELINE_TIME_S -
            base["total_mission_time_s"]) / BASELINE_TIME_S
            if finite(base.get("total_mission_time_s")) else None),
        "reference_total_duration": base.get("reference_total_duration_s"),
        "maximum_turn_angle": max((item["maximum_turn_angle_deg"]
                                   for item in timing_goals), default=None),
        "corner_adjusted_segment_count": sum(item["corner_adjusted_segment_count"]
                                             for item in timing_goals),
        "maximum_applied_corner_scale": max((item["maximum_segment_corner_scale"]
                                             for item in timing_goals), default=None),
        "selected_global_duration_scale": [item["selected_global_duration_scale"]
                                           for item in timing_goals],
        "reference_jerk_max": jerk["max"],
        "actual_jerk_max": actual_jerk_stats["max"],
        "tracking_max": tracking["max"], "tracking_rms": tracking["rms"],
        "cross_track_max": cross["max"], "cross_track_rms": cross["rms"],
        "reference_minimum_clearance": reference_clearance(reference),
        "actual_minimum_clearance": metrics.get("minimum_safety_clearance_m"),
        "maximum_roll": metrics.get("maximum_absolute_roll_rad"),
        "maximum_pitch": metrics.get("maximum_absolute_pitch_rad"),
        "maximum_yaw_rate": max((abs(sample["yaw_rate"]) for sample in samples),
                                default=None),
        "maximum_actual_horizontal_acceleration": actual_horizontal_acceleration["max"],
        "collision": metrics.get("collision_observed"),
        "saturation": metrics.get("saturation_sample_count", 0),
        "nonfinite": base.get("non_finite_value_count", 0) + extra_nonfinite,
        "planned_point_count": len(planned), "simplified_point_count": len(simplified),
        "simplified_path_length": sum(math.dist(a, b) for a, b in
                                      zip(simplified, simplified[1:])),
        "turning_angles": [[item["start_corner_angle"], item["end_corner_angle"]]
                           for item in timing_segments],
        "corner_scales": [item["corner_scale"] for item in timing_segments],
        "segment_corner_scales": [item["corner_scale"] for item in timing_segments],
        "base_durations": [item["base_duration"] for item in timing_segments],
        "corner_adjusted_durations": [item["corner_adjusted_duration"]
                                      for item in timing_segments],
        "final_durations": [item["final_duration"] for item in timing_segments],
        "local_corners": local, "reference_path": reference,
        "planned_path": planned, "simplified_path": simplified,
        "series": {"time": [item["time"] for item in samples],
                   "reference_position": [item["reference_position"] for item in samples],
                   "actual_position": [item["position"] for item in samples],
                   "reference_speed": [vector_norm(item["reference_velocity"])
                                       for item in samples],
                   "actual_speed": [vector_norm(item["velocity"]) for item in samples],
                   "reference_acceleration": [vector_norm(
                       item["reference_acceleration"]) for item in samples],
                   "tracking": [item["tracking"] for item in samples],
                   "cross_track": cross_track,
                   "jerk_time": [item["time"] for item in reference_jerk],
                   "reference_jerk": [vector_norm(item["value"])
                                      for item in reference_jerk]},
    }
    worst = sorted((item["reference_jerk_max"] for item in local
                    if finite(item["reference_jerk_max"])), reverse=True)[:3]
    row["worst_corner_reference_jerk"] = statistics.fmean(worst) if worst else None
    return row


def safety_pass(row):
    return bool(row.get("success") and not row.get("collision") and
                row.get("nonfinite") == 0 and row.get("saturation") == 0 and
                finite(row.get("tracking_max")) and row["tracking_max"] < 0.10 and
                finite(row.get("actual_minimum_clearance")) and
                row["actual_minimum_clearance"] > 0 and
                finite(row.get("mission_time_s")) and row["mission_time_s"] <= 60.68)


def add_comparisons(rows):
    baselines = {(row["route"], row["run"]): row for row in rows
                 if row["candidate"] == "t0"}
    fallback = {route: next((row for row in rows if row["candidate"] == "t0" and
                            row["route"] == route), None) for route in ROUTES}
    for row in rows:
        baseline = baselines.get((row["route"], row["run"])) or fallback[row["route"]]
        row["reference_hausdorff_to_t0"] = (hausdorff(
            row.get("reference_path", []), baseline.get("reference_path", []))
            if baseline else None)
        row["planned_matches_t0"] = bool(baseline and
            row.get("planned_path") == baseline.get("planned_path"))
        row["simplified_matches_t0"] = bool(baseline and
            row.get("simplified_path") == baseline.get("simplified_path"))


def aggregate(rows):
    output = []
    for candidate in CANDIDATES:
        for route in ROUTES:
            group = [row for row in rows if row["candidate"] == candidate and
                     row["route"] == route]
            if not group:
                continue
            times = [row["mission_time_s"] for row in group
                     if finite(row.get("mission_time_s"))]
            output.append({
                "candidate": candidate, "route": route, "runs": len(group),
                "success_runs": sum(row.get("success") is True for row in group),
                "mission_time_mean": statistics.fmean(times) if times else None,
                "mission_time_min": min(times) if times else None,
                "mission_time_max": max(times) if times else None,
                "mission_time_population_stddev": statistics.pstdev(times)
                if times else None,
            })
    return output


def choose(rows):
    baseline = next((row for row in rows if row["candidate"] == "t0" and
                     row["route"] == "full_map"), None)
    if not baseline:
        return [], "C"
    eligible = []
    for candidate in ("t10", "t20", "t30"):
        row = next((item for item in rows if item["candidate"] == candidate and
                   item["route"] == "full_map"), None)
        if not row or not safety_pass(row):
            continue
        improvements = 0
        for field, threshold in (("reference_jerk_max", 0.15),
                                 ("worst_corner_reference_jerk", 0.20),
                                 ("cross_track_max", 0.10),
                                 ("cross_track_rms", 0.15)):
            if finite(row.get(field)) and finite(baseline.get(field)) and baseline[field] > 0:
                improvements += (baseline[field] - row[field]) / baseline[field] >= threshold
        improvements += (finite(row.get("maximum_actual_horizontal_acceleration")) and
                         finite(baseline.get("maximum_actual_horizontal_acceleration")) and
                         row["maximum_actual_horizontal_acceleration"] <
                         baseline["maximum_actual_horizontal_acceleration"])
        if improvements >= 2:
            eligible.append(candidate)
    return eligible[:2], ("A" if eligible else "C")


def plot_summary(output, rows):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    plots = output / "plots"
    plots.mkdir(parents=True, exist_ok=True)
    full = [row for row in rows if row["route"] == "full_map" and row["run"] == 1]
    fig, axis = plt.subplots(figsize=(8, 6))
    for row in full:
        path = row.get("reference_path", [])
        if path:
            axis.plot([point[0] for point in path], [point[1] for point in path],
                      label=row["candidate"])
    axis.set(xlabel="x (m)", ylabel="y (m)"); axis.grid(alpha=.3); axis.legend()
    fig.tight_layout(); fig.savefig(plots / "reference_xy_overlay.png", dpi=150); plt.close(fig)
    for field, name, ylabel in (("reference_speed", "reference_actual_speed", "speed (m/s)"),
                                ("reference_acceleration", "reference_acceleration", "m/s²"),
                                ("reference_jerk", "reference_jerk", "m/s³"),
                                ("tracking", "tracking_cross_track", "error (m)")):
        fig, axis = plt.subplots(figsize=(9, 5))
        for row in full:
            series = row.get("series", {})
            x = series.get("jerk_time" if field == "reference_jerk" else "time", [])
            axis.plot(x, series.get(field, []), label=f"{row['candidate']} {field}")
            if field == "reference_speed":
                axis.plot(series.get("time", []), series.get("actual_speed", []),
                          ls="--", alpha=.7, label=f"{row['candidate']} actual")
            if field == "tracking":
                axis.plot(series.get("time", []), series.get("cross_track", []),
                          ls="--", alpha=.7, label=f"{row['candidate']} cross-track")
        axis.set(xlabel="mission time (s)", ylabel=ylabel); axis.grid(alpha=.3)
        axis.legend(ncol=2, fontsize=8); fig.tight_layout()
        fig.savefig(plots / f"{name}.png", dpi=150); plt.close(fig)
    fig, axis = plt.subplots(figsize=(10, 5))
    for row in full:
        axis.plot(row.get("turning_angles", []), label=f"{row['candidate']} angles")
        axis.plot(row.get("final_durations", []), ls="--",
                  label=f"{row['candidate']} durations")
    axis.set(xlabel="segment index"); axis.grid(alpha=.3); axis.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(plots / "segment_duration_turning_angle.png", dpi=150)
    plt.close(fig)
    for corner_index, (x, y) in enumerate(CORNERS, start=1):
        fig, axis = plt.subplots(figsize=(6, 6))
        for row in full:
            series = row.get("series", {})
            reference = [point for point in series.get("reference_position", [])
                         if math.hypot(point[0] - x, point[1] - y) <= 1.0]
            actual = [point for point in series.get("actual_position", [])
                      if math.hypot(point[0] - x, point[1] - y) <= 1.0]
            if reference:
                axis.plot([point[0] for point in reference],
                          [point[1] for point in reference],
                          label=f"{row['candidate']} reference")
            if actual:
                axis.plot([point[0] for point in actual], [point[1] for point in actual],
                          ls="--", alpha=.7, label=f"{row['candidate']} actual")
        axis.scatter([x], [y], marker="x", color="black", label="diagnostic center")
        axis.set(xlabel="x (m)", ylabel="y (m)"); axis.grid(alpha=.3)
        axis.legend(fontsize=7); fig.tight_layout()
        fig.savefig(plots / f"corner_{corner_index:02d}.png", dpi=150); plt.close(fig)


def summarize(output):
    rows = [json.loads(path.read_text()) for path in sorted(
        output.glob("*/**/run_*/run_record.json"))]
    add_comparisons(rows)
    aggregates = aggregate(rows)
    selected, classification = choose(rows)
    for row in rows:
        row["selection"] = "selected" if row["candidate"] in selected else ""
    payload = {"generated_at": datetime.now(timezone.utc).isoformat(),
               "classification": classification, "selected_candidates": selected,
               "aggregates": aggregates, "runs": rows}
    (output / "timing_sweep.json").write_text(
        json.dumps(payload, indent=2, allow_nan=False) + "\n")
    with (output / "timing_sweep.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            aggregate_row = next((item for item in aggregates
                                  if item["candidate"] == row["candidate"] and
                                  item["route"] == row["route"]), {})
            merged = {**row, **aggregate_row}
            writer.writerow({field: json.dumps(merged[field], separators=(",", ":"))
                             if isinstance(merged.get(field), (list, dict))
                             else merged.get(field) for field in CSV_FIELDS})
    lines = ["# Corner-aware trajectory timing sweep", "",
             f"Automated classification: **{classification}**", "",
             "| candidate | route | success/runs | mean/min/max/stddev mission time (s) |",
             "|---|---|---:|---:|"]
    for item in aggregates:
        values = (item["mission_time_mean"], item["mission_time_min"],
                  item["mission_time_max"], item["mission_time_population_stddev"])
        formatted = "/".join(f"{value:.3f}" for value in values) if all(
            finite(value) for value in values) else "n/a"
        lines.append(f"| {item['candidate']} | {item['route']} | "
                     f"{item['success_runs']}/{item['runs']} | {formatted} |")
    lines += ["", "Selected candidates: " + (", ".join(selected) if selected else "none"),
              "", "Human RViz comparison: pending explicit human observation.", ""]
    (output / "timing_summary.md").write_text("\n".join(lines))
    plot_summary(output, rows)
    return rows


def run_once(args, run):
    params = parameters(args.candidate)
    commit, dirty = navigation_sweep.git_state()
    if dirty and not args.allow_dirty:
        raise SystemExit("refusing candidate evidence run from a dirty worktree")
    domain = args.domain_id if args.domain_id is not None else (
        20 + (int(time.time() * 1000) + run) % 180)
    run_dir = args.output / args.candidate / args.route / f"run_{run:02d}"
    run_dir.mkdir(parents=True, exist_ok=True)
    manifest = {"candidate": args.candidate, "route": args.route, "run": run,
                "parameters": params, "repository_commit": commit, "git_dirty": dirty,
                "ros_domain_id": domain, "run_status": "candidate",
                "generated_at": datetime.now(timezone.utc).isoformat()}
    (run_dir / "run_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    env = os.environ.copy(); env["ROS_DOMAIN_ID"] = str(domain)
    launch_handle = (run_dir / "launch.log").open("w")
    recorder_handle = (run_dir / "recorder_process.log").open("w")
    launch = recorder = None
    try:
        launch = subprocess.Popen(navigation_sweep.ros_command([
            "ros2", "launch", "drone_bringup", "assessment_navigation_sim.launch.py",
            "use_rviz:=false", "yaw_mode:=path_tangent", "nominal_speed:=0.50",
            "max_reference_speed:=0.90", "max_reference_acceleration:=0.60",
            f"corner_timing_enabled:={'true' if params['corner_timing_enabled'] else 'false'}",
            "corner_timing_start_angle_deg:=25.0", "corner_timing_full_angle_deg:=70.0",
            f"corner_timing_max_duration_scale:={params['corner_timing_max_duration_scale']}",
        ]), cwd=ROOT, env=env, stdout=launch_handle, stderr=subprocess.STDOUT,
            start_new_session=True)
        recorder = subprocess.Popen(navigation_sweep.ros_command([
            "python3", str(ROOT / "tools/assessment_recorder.py"),
            "--experiment", "navigation", "--run-status", "candidate",
            "--output", str(run_dir), "--timeout", str(args.timeout),
        ]), cwd=ROOT, env=env, stdout=recorder_handle, stderr=subprocess.STDOUT,
            start_new_session=True)
        time.sleep(args.startup_wait)
        service = subprocess.run(navigation_sweep.ros_command([
            "ros2", "service", "call", "/drone/interactive_goals/execute",
            "drone_msgs/srv/ExecuteGoalSequence",
            navigation_sweep.request_yaml(args.route),
        ]), cwd=ROOT, env=env, text=True, capture_output=True, timeout=45)
        (run_dir / "service.log").write_text(service.stdout + service.stderr)
        normalized = service.stdout.replace(" ", "").lower()
        if service.returncode != 0 or not ("accepted:true" in normalized or
                                           "accepted=true" in normalized):
            navigation_sweep.terminate(recorder)
        else:
            recorder.wait(timeout=args.timeout + 20)
    except (subprocess.TimeoutExpired, KeyboardInterrupt) as error:
        (run_dir / "orchestrator_error.log").write_text(str(error) + "\n")
    finally:
        navigation_sweep.terminate(recorder); navigation_sweep.terminate(launch)
        launch_handle.close(); recorder_handle.close()
    if (run_dir / "metadata.json").exists():
        with (run_dir / "analyzer.log").open("w") as analyzer:
            subprocess.run(navigation_sweep.ros_command([
                "python3", str(ROOT / "tools/analyze_assessment_run.py"), str(run_dir),
                "--parameters", str(ROOT / "results/parameters"),
            ]), cwd=ROOT, env=env, stdout=analyzer, stderr=subprocess.STDOUT,
                timeout=90, check=False)
    row = analyze_run(run_dir, args.candidate, args.route, run,
                      params, commit, dirty, domain)
    (run_dir / "run_record.json").write_text(
        json.dumps(row, indent=2, allow_nan=False) + "\n")
    summarize(args.output)
    return 0 if row.get("success") else 1


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
    parser.add_argument("--allow-dirty", action="store_true")
    parser.add_argument("--summarize", action="store_true")
    args = parser.parse_args(argv)
    if not args.summarize and (not args.candidate or not args.route):
        parser.error("--candidate and --route are required unless --summarize is used")
    if args.runs <= 0 or args.run_start <= 0:
        parser.error("--runs and --run-start must be positive")
    return args


def main(argv=None):
    args = arguments(argv)
    if args.summarize:
        summarize(args.output)
        return 0
    status = 0
    for run in range(args.run_start, args.run_start + args.runs):
        status = max(status, run_once(args, run))
    return status


if __name__ == "__main__":
    sys.exit(main())
