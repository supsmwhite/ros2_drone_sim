#!/usr/bin/env python3
"""Compute fixed assessment metrics and plots from one recorder output directory."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else math.nan
    except (TypeError, ValueError):
        return math.nan


def finite(values):
    return [value for value in values if math.isfinite(value)]


def mean(values):
    values = finite(values)
    return sum(values) / len(values) if values else None


def rms(values):
    values = finite(values)
    return math.sqrt(sum(value*value for value in values) / len(values)) if values else None


def max_or_none(values):
    values = finite(values)
    return max(values) if values else None


def min_or_none(values):
    values = finite(values)
    return min(values) if values else None


def path_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:])) if len(points) >= 2 else 0.0


def load_rows(path):
    with path.open(newline="", encoding="utf-8") as handle:
        raw = list(csv.DictReader(handle))
    return [{key: number(value) for key, value in row.items()} for row in raw]


def yaml_parameters(directory):
    def params(name):
        data = yaml.safe_load((directory / name).read_text(encoding="utf-8"))
        return next(iter(data.values()))["ros__parameters"]
    return params("dynamics.yaml"), params("controller.yaml"), params("environment.yaml")


def window(rows, duration):
    end = rows[-1]["mission_time_s"]
    return [row for row in rows if row["mission_time_s"] >= end-duration]


def arrival_time(rows, position_threshold, speed_threshold, hold):
    candidate = None
    for row in rows:
        ok = row["position_error"] < position_threshold and row["speed"] < speed_threshold
        if ok and candidate is None:
            candidate = row["mission_time_s"]
        elif not ok:
            candidate = None
        if candidate is not None and row["mission_time_s"]-candidate >= hold:
            return candidate
    return None


def consecutive_duration(rows, predicate):
    longest = 0.0
    start = None
    previous = None
    for row in rows:
        t = row["mission_time_s"]
        if predicate(row):
            if start is None:
                start = t
            previous = t
            longest = max(longest, previous-start)
        else:
            start = previous = None
    return longest


def overshoot(rows, experiment):
    target = [rows[-1][key] for key in ("target_x", "target_y", "target_z")]
    if experiment == "hover" or math.dist(
            [rows[0][key] for key in ("actual_x", "actual_y", "actual_z")], target) < 1.0e-9:
        value = max(0.0, max(row["actual_z"]-row["target_z"] for row in rows))
        return value, None, []
    segment_metrics = []
    indices = []
    for row in rows:
        value = row.get("current_goal_index", math.nan)
        indices.append(0 if not math.isfinite(value) else int(value))
    for index in sorted(set(indices)):
        segment = [row for row, item in zip(rows, indices) if item == index]
        start = [segment[0][key] for key in ("actual_x", "actual_y", "actual_z")]
        goal = [segment[-1][key] for key in ("target_x", "target_y", "target_z")]
        distance = math.dist(start, goal)
        if distance < 1.0e-9:
            value = max(0.0, max(row["actual_z"]-row["target_z"] for row in segment))
            percent = None
        else:
            unit = [(goal[i]-start[i])/distance for i in range(3)]
            projections = [sum((row[key]-goal[i])*unit[i]
                               for i, key in enumerate(("actual_x", "actual_y", "actual_z")))
                           for row in segment]
            value = max(0.0, max(projections))
            percent = 100.0*value/distance
        segment_metrics.append({"goal_index": index, "maximum_overshoot_m": value,
                                "maximum_overshoot_percent": percent})
    return max(item["maximum_overshoot_m"] for item in segment_metrics), max(
        (item["maximum_overshoot_percent"] for item in segment_metrics
         if item["maximum_overshoot_percent"] is not None), default=None), segment_metrics


def save_plot(path, draw, xlabel="mission time (s)", ylabel=None):
    fig = plt.figure(figsize=(8, 4.8))
    ax = fig.add_subplot(111)
    draw(ax)
    ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    handles, labels = ax.get_legend_handles_labels()
    if labels:
        ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plots(run, rows, paths, experiment):
    t = [row["mission_time_s"] for row in rows]
    save_plot(run / "trajectory_xy.png", lambda ax: (
        ax.plot([r["actual_x"] for r in rows], [r["actual_y"] for r in rows], label="actual"),
        ax.scatter([rows[-1]["target_x"]], [rows[-1]["target_y"]], marker="x", label="target")),
        "x (m)", "y (m)")
    fig = plt.figure(figsize=(8, 5.5)); ax = fig.add_subplot(111, projection="3d")
    ax.plot([r["actual_x"] for r in rows], [r["actual_y"] for r in rows],
            [r["actual_z"] for r in rows], label="actual")
    ax.scatter([rows[-1]["target_x"]], [rows[-1]["target_y"]], [rows[-1]["target_z"]], marker="x", label="target")
    ax.set(xlabel="x (m)", ylabel="y (m)", zlabel="z (m)"); ax.legend(); fig.tight_layout()
    fig.savefig(run / "trajectory_3d.png", dpi=150); plt.close(fig)
    save_plot(run / "position_xyz.png", lambda ax: [
        ax.plot(t, [r["actual_"+axis] for r in rows], label=axis) for axis in "xyz"], ylabel="position (m)")
    save_plot(run / "position_error.png", lambda ax: ax.plot(t, [r["position_error"] for r in rows]), ylabel="3-D error (m)")
    save_plot(run / "attitude.png", lambda ax: [
        ax.plot(t, [r[axis] for r in rows], label=axis) for axis in ("roll", "pitch", "yaw")], ylabel="angle (rad)")
    save_plot(run / "motor_rpm.png", lambda ax: [
        ax.plot(t, [r[f"m{i}_rpm"] for r in rows], label=f"M{i}") for i in range(1, 5)], ylabel="RPM")
    if experiment == "navigation":
        def route(ax):
            for name in ("planned", "reference"):
                points = paths.get(name, [])
                if points: ax.plot([p[0] for p in points], [p[1] for p in points], label=name)
            ax.plot([r["actual_x"] for r in rows], [r["actual_y"] for r in rows], label="actual")
        save_plot(run / "planned_reference_actual.png", route, "x (m)", "y (m)")
        save_plot(run / "obstacle_clearance.png", lambda ax: ax.plot(t, [r["safety_clearance"] for r in rows]), ylabel="safety clearance (m)")
        save_plot(run / "tracking_error.png", lambda ax: ax.plot(t, [r["position_error"] for r in rows]), ylabel="tracking error (m)")
    if experiment == "disturbance":
        horizontal = [math.hypot(r["error_x"], r["error_y"]) for r in rows]
        force = [math.hypot(r.get("external_force_x", math.nan), r.get("external_force_y", math.nan)) for r in rows]
        save_plot(run / "horizontal_error.png", lambda ax: ax.plot(t, horizontal), ylabel="horizontal error (m)")
        save_plot(run / "external_force.png", lambda ax: ax.plot(t, force), ylabel="horizontal force (N)")
        save_plot(run / "integral_compensation.png", lambda ax: [ax.plot(t, [r.get("integral_compensation_"+a, math.nan) for r in rows], label=a) for a in "xy"], ylabel="integral acceleration (m/s²)")
        save_plot(run / "recovery.png", lambda ax: (ax.plot(t, horizontal, label="error"), ax.plot(t, force, label="force")), ylabel="m / N")
    if experiment == "failure_case":
        save_plot(run / "failure_timeline.png", lambda ax: ax.step(t, [r.get("mission_success", math.nan) for r in rows]), ylabel="mission success")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--parameters", type=Path, default=Path("results/parameters"))
    args = parser.parse_args()
    metadata = json.loads((args.run / "metadata.json").read_text(encoding="utf-8"))
    all_rows = load_rows(args.run / "samples.csv")
    rows = [row for row in all_rows if math.isfinite(row["mission_time_s"]) and
            math.isfinite(row["position_error"])]
    if len(rows) < 2:
        raise SystemExit("run has fewer than two mission samples")
    if any(b["time_s"] <= a["time_s"] for a, b in zip(all_rows, all_rows[1:])):
        raise SystemExit("samples.csv time_s is not strictly increasing")
    dynamics, controller, environment = yaml_parameters(args.parameters)
    paths = json.loads((args.run / "paths.json").read_text(encoding="utf-8"))
    thresholds = metadata["thresholds"]
    final_window = window(rows, 1.0)
    steady_duration = min(3.0, rows[-1]["mission_time_s"]-rows[0]["mission_time_s"])
    steady = window(rows, steady_duration)
    maximum_overshoot, overshoot_percent, segment_overshoots = overshoot(rows, metadata["experiment"])
    rpm_values = [row[f"m{i}_rpm"] for row in rows for i in range(1, 5)]
    non_finite_rpm = sum(not math.isfinite(value) for value in rpm_values)
    saturation = lambda row: any(row[key] == 1.0 for key in (
        "horizontal_saturated", "altitude_saturated", "attitude_saturated", "mixer_saturated"))
    attitude_bad = lambda row: abs(row["roll"]) > max(0.5, 2*float(controller["max_tilt_angle"])) or abs(row["pitch"]) > max(0.5, 2*float(controller["max_tilt_angle"]))
    actual_points = [[row[key] for key in ("actual_x", "actual_y", "actual_z")] for row in rows]
    final_target = [rows[-1][key] for key in ("target_x", "target_y", "target_z")]
    reference_length = path_length(paths["reference"]) if paths.get("reference") else None
    planned_length = path_length(paths["planned"]) if paths.get("planned") else None
    raw_clearance = min_or_none([row["raw_obstacle_distance"] for row in rows])
    safety_radius = float(environment["safety_radius"])
    metrics = {
        "final_position_error_m": rows[-1]["position_error"],
        "final_window_mean_error_m": mean([r["position_error"] for r in final_window]),
        "arrival_time_s": arrival_time(rows, thresholds["arrival_position_threshold_m"], thresholds["arrival_speed_threshold_m_s"], thresholds["arrival_hold_time_s"]),
        "maximum_overshoot_m": maximum_overshoot,
        "maximum_overshoot_percent": overshoot_percent,
        "segment_overshoots": segment_overshoots,
        "steady_state_mean_error_m": mean([r["position_error"] for r in steady]),
        "steady_state_rms_error_m": rms([r["position_error"] for r in steady]),
        "steady_state_max_error_m": max_or_none([r["position_error"] for r in steady]),
        "steady_state_window_s": steady_duration,
        "minimum_raw_obstacle_distance_m": raw_clearance,
        "safety_radius_m": safety_radius,
        "minimum_safety_clearance_m": None if raw_clearance is None else raw_clearance-safety_radius,
        "straight_line_distance_m": math.dist(actual_points[0], final_target),
        "planned_path_length_m": planned_length,
        "reference_path_length_m": reference_length,
        "actual_path_length_m": path_length(actual_points),
        "flight_time_s": rows[-1]["mission_time_s"]-rows[0]["mission_time_s"],
        "path_efficiency": None if not reference_length else path_length(actual_points)/reference_length,
        "maximum_absolute_roll_rad": max_or_none([abs(r["roll"]) for r in rows]),
        "maximum_absolute_pitch_rad": max_or_none([abs(r["pitch"]) for r in rows]),
        "maximum_angular_speed_rad_s": max_or_none([math.sqrt(r["angular_speed_x"]**2+r["angular_speed_y"]**2+r["angular_speed_z"]**2) for r in rows]),
        "non_finite_attitude_count": sum(not all(math.isfinite(r[key]) for key in ("roll", "pitch", "yaw")) for r in rows),
        "attitude_divergence_detected": consecutive_duration(rows, attitude_bad) >= 1.0,
        "minimum_motor_rpm": min_or_none(rpm_values), "maximum_motor_rpm": max_or_none(rpm_values),
        "saturation_sample_count": sum(saturation(row) for row in rows),
        "longest_saturation_duration_s": consecutive_duration(rows, saturation),
        "saturated_at_end": saturation(rows[-1]), "non_finite_rpm_count": non_finite_rpm}
    if metadata["experiment"] == "disturbance":
        horizontal = [math.hypot(r["error_x"], r["error_y"]) for r in rows]
        forces = [math.hypot(r.get("external_force_x", math.nan), r.get("external_force_y", math.nan)) for r in rows]
        active = [i for i, force in enumerate(forces) if math.isfinite(force) and force > 1.0e-6]
        release = max(active)+1 if active and max(active)+1 < len(rows) else None
        baseline = mean(horizontal[:max(1, (active[0] if active else len(rows))//2)])
        recovery = None
        if release is not None and baseline is not None:
            limit = baseline + thresholds["arrival_position_threshold_m"]
            for row, value in zip(rows[release:], horizontal[release:]):
                if value < limit:
                    recovery = row["mission_time_s"]-rows[release]["mission_time_s"]; break
        metrics.update({"peak_horizontal_deviation_m": max(horizontal),
                        "disturbance_steady_state_error_m": mean(horizontal[-len(steady):]),
                        "recovery_time_s": recovery,
                        "reverse_overshoot_m": None if release is None else max(0.0, max(horizontal[release:])-(baseline or 0.0))})
    summary = {
        "schema_version": 1, "experiment": metadata["experiment"], "status": metadata["status"],
        "repository_commit": metadata["repository_commit"], "sample_count": len(all_rows),
        "mission_sample_count": len(rows), "metrics": metrics,
        "assignment_thresholds": {"hover_final_position_error_max_m": 0.30},
        "project_thresholds": {"final_position_error_max_m": 0.10,
            "minimum_safety_clearance_strictly_greater_than_m": 0.0,
            "non_finite_value_count": 0, "attitude_divergence_detected": False,
            "saturated_at_end": False},
        "checks": {"assignment_hover_error_pass": metrics["final_position_error_m"] < 0.30,
            "project_final_error_pass": metrics["final_position_error_m"] < 0.10,
            "obstacle_clearance_pass": metrics["minimum_safety_clearance_m"] is None or metrics["minimum_safety_clearance_m"] > 0.0,
            "finite_attitude_pass": metrics["non_finite_attitude_count"] == 0,
            "finite_rpm_pass": metrics["non_finite_rpm_count"] == 0,
            "attitude_stability_pass": not metrics["attitude_divergence_detected"],
            "rpm_end_saturation_pass": not metrics["saturated_at_end"]}}
    (args.run / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    plots(args.run, rows, paths, metadata["experiment"])
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
