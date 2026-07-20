#!/usr/bin/env python3
"""Run and summarize candidate-only simplifier clearance experiments."""

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

import run_navigation_speed_sweep as speed_sweep


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = Path("/tmp/ros2_drone_simplifier_clearance")
CANDIDATES = {"s0": 0.0, "c18": 0.18, "c20": 0.20, "c25": 0.25}
A2_PARAMETERS = {
    "nominal_speed": 0.50,
    "max_reference_speed": 0.90,
    "max_reference_acceleration": 0.60,
}
FOCUS_POINTS = {
    "3p6_1p85": (3.60, 1.85),
    "7p6_m1p15": (7.60, -1.15),
    "10p6_1p35": (10.60, 1.35),
    "9p1_0p35": (9.10, 0.35),
    "9p85_0p6": (9.85, 0.60),
}
DIAGNOSTIC_RE = re.compile(
    r"ordered goal (?P<goal>\d+) trajectory ready:.*?"
    r"preferred_shortcuts=(?P<preferred>\d+) "
    r"fallback_shortcuts=(?P<fallback>\d+) "
    r"collision_only_shortcuts=(?P<collision_only>\d+) "
    r"shortcut_preferred_clearance=(?P<clearance>[0-9.]+)")
COMPARISON_FIELDS = (
    "candidate preferred_clearance route run success mission_time_s "
    "planned_points simplified_points simplified_path_length_m "
    "minimum_segment_length_m short_segment_count corner_count "
    "maximum_turn_angle_deg preferred_shortcut_count fallback_shortcut_count "
    "minimum_simplified_clearance_m minimum_reference_clearance_m "
    "minimum_actual_clearance_m clearance_at_3p6_1p85 "
    "clearance_at_7p6_m1p15 clearance_at_10p6_1p35 clearance_at_9p1_0p35 "
    "clearance_at_9p85_0p6 reference_jerk_max_m_s3 actual_jerk_max_m_s3 "
    "cross_track_max_m cross_track_rms_m tracking_max_m tracking_rms_m "
    "collision saturation nonfinite repository_commit git_dirty selection"
).split()


def candidate_parameters(name):
    try:
        clearance = CANDIDATES[name.lower()]
    except KeyError as error:
        raise ValueError(f"unknown candidate: {name}") from error
    return {**A2_PARAMETERS, "shortcut_preferred_clearance": clearance}


def parse_simplifier_diagnostics(text):
    return [{"goal_index": int(match.group("goal")),
             "preferred_shortcut_count": int(match.group("preferred")),
             "fallback_shortcut_count": int(match.group("fallback")),
             "collision_only_shortcut_count": int(match.group("collision_only")),
             "shortcut_preferred_clearance": float(match.group("clearance"))}
            for match in DIAGNOSTIC_RE.finditer(text)]


def service_accepted(text, returncode):
    normalized = text.replace(" ", "").lower()
    return returncode == 0 and (
        "accepted:true" in normalized or "accepted=true" in normalized)


def combined_segment_lengths(paths):
    lengths = []
    for segment in sorted(paths.get("simplified_segments", []),
                          key=lambda item: item["sequence"]):
        points = segment.get("points", [])
        lengths.extend(math.dist(first[:3], second[:3])
                       for first, second in zip(points, points[1:]))
    return lengths


def nearest_corner(corners, point, tolerance=0.05):
    if not corners:
        return None
    corner = min(corners, key=lambda item: math.dist(item["position"][:2], point))
    return corner if math.dist(corner["position"][:2], point) <= tolerance else None


def extract_run_summary(run_dir, record):
    geometry_path = run_dir / "geometry" / "geometry_summary.json"
    if not geometry_path.exists():
        return {"candidate": record["candidate"], "route": record["route"],
                "run": record["run"], "success": False,
                "repository_commit": record["repository_commit"],
                "git_dirty": record["git_dirty"]}
    geometry = json.loads(geometry_path.read_text())
    paths = json.loads((run_dir / "paths.json").read_text())
    lengths = combined_segment_lengths(paths)
    diagnostics = parse_simplifier_diagnostics(
        (run_dir / "launch.log").read_text(errors="replace"))
    corners = geometry["corners"]
    focus = {}
    for name, point in FOCUS_POINTS.items():
        corner = nearest_corner(corners, point)
        focus[name] = ({layer: corner.get(f"{layer}_clearance_m")
                        for layer in ("planned", "simplified", "reference", "actual")}
                       if corner else None)
    layers = geometry["layers"]
    return {
        "candidate": record["candidate"], "route": record["route"],
        "run": record["run"], "success": record["success"],
        "preferred_clearance": candidate_parameters(record["candidate"])[
            "shortcut_preferred_clearance"],
        "mission_time_s": record.get("mission_complete_time_s"),
        "planned_points": layers["planned"]["original_point_count"],
        "simplified_points": layers["simplified"]["original_point_count"],
        "simplified_path_length_m": layers["simplified"]["path_length_m"],
        "segment_lengths_m": lengths,
        "minimum_segment_length_m": min(lengths, default=None),
        "short_segment_count": sum(length < 0.20 for length in lengths),
        "corner_count": geometry["corner_count"],
        "maximum_turn_angle_deg": max(
            (corner["angle_deg"] for corner in corners), default=None),
        "preferred_shortcut_count": sum(
            item["preferred_shortcut_count"] for item in diagnostics),
        "fallback_shortcut_count": sum(
            item["fallback_shortcut_count"] for item in diagnostics),
        "collision_only_shortcut_count": sum(
            item["collision_only_shortcut_count"] for item in diagnostics),
        "minimum_simplified_clearance_m": layers["simplified"][
            "minimum_safety_clearance_m"],
        "minimum_reference_clearance_m": layers["reference"][
            "minimum_safety_clearance_m"],
        "minimum_actual_clearance_m": layers["actual"][
            "minimum_safety_clearance_m"],
        "focus_clearances": focus,
        "reference_jerk_max_m_s3": max(
            (corner.get("reference_max_jerk_m_s3") or 0.0 for corner in corners),
            default=None),
        "actual_jerk_max_m_s3": max(
            (corner.get("actual_max_jerk_m_s3") or 0.0 for corner in corners),
            default=None),
        "cross_track_max_m": geometry["global_dynamics"]["spatial_cross_track"]["max"],
        "cross_track_rms_m": geometry["global_dynamics"]["spatial_cross_track"]["rms"],
        "tracking_max_m": geometry["global_dynamics"]["temporal_tracking"]["max"],
        "tracking_rms_m": geometry["global_dynamics"]["temporal_tracking"]["rms"],
        "actual_max_speed_m_s": record.get("actual_max_speed_m_s"),
        "reference_max_acceleration_m_s2": record.get(
            "reference_max_total_acceleration_m_s2"),
        "actual_max_acceleration_m_s2": record.get(
            "actual_max_total_acceleration_m_s2"),
        "maximum_absolute_roll_rad": record.get("maximum_absolute_roll_rad"),
        "maximum_absolute_pitch_rad": record.get("maximum_absolute_pitch_rad"),
        "maximum_yaw_rate_rad_s": record.get("maximum_angular_speed_rad_s"),
        "minimum_motor_rpm": record.get("minimum_motor_rpm"),
        "maximum_motor_rpm": record.get("maximum_motor_rpm"),
        "collision": record.get("collision"),
        "saturation": bool(record.get("saturation_sample_count")),
        "saturated_at_end": record.get("saturated_at_end"),
        "nonfinite": record.get("non_finite_value_count"),
        "repository_commit": record["repository_commit"],
        "git_dirty": record["git_dirty"],
    }


def run_geometry(run_dir, env):
    output = run_dir / "geometry"
    command = speed_sweep.ros_command([
        "python3", str(ROOT / "tools/analyze_navigation_geometry.py"), str(run_dir),
        "--environment", str(ROOT / "src/drone_bringup/config/environment.yaml"),
        "--astar", str(ROOT / "src/drone_bringup/config/astar.yaml"),
        "--trajectory", str(ROOT / "src/drone_bringup/config/planned_trajectory.yaml"),
        "--output", str(output),
    ])
    with (run_dir / "geometry_analyzer.log").open("w") as handle:
        return subprocess.run(command, cwd=ROOT, env=env, stdout=handle,
                              stderr=subprocess.STDOUT, timeout=120).returncode


def run_once(args):
    parameters = candidate_parameters(args.candidate)
    commit, dirty = speed_sweep.git_state()
    if dirty and not args.allow_dirty:
        raise SystemExit("refusing candidate run from a dirty worktree")
    domain = args.domain_id if args.domain_id is not None else (
        20 + (int(time.time() * 1000) + args.run) % 180)
    run_dir = args.output / args.candidate / args.route / f"run_{args.run:02d}"
    run_dir.mkdir(parents=True, exist_ok=True)
    manifest = {"candidate": args.candidate, "route": args.route, "run": args.run,
                "parameters": parameters, "repository_commit": commit,
                "git_dirty": dirty, "ros_domain_id": domain,
                "run_status": "candidate",
                "generated_at": datetime.now(timezone.utc).isoformat()}
    (run_dir / "run_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain)
    launch_handle = (run_dir / "launch.log").open("w")
    recorder_handle = (run_dir / "recorder_process.log").open("w")
    launch = recorder = None
    try:
        launch = subprocess.Popen(speed_sweep.ros_command([
            "ros2", "launch", "drone_bringup", "assessment_navigation_sim.launch.py",
            "use_rviz:=false", "yaw_mode:=path_tangent",
            f"nominal_speed:={parameters['nominal_speed']}",
            f"max_reference_speed:={parameters['max_reference_speed']}",
            f"max_reference_acceleration:={parameters['max_reference_acceleration']}",
            f"shortcut_preferred_clearance:={parameters['shortcut_preferred_clearance']}",
        ]), cwd=ROOT, env=env, stdout=launch_handle, stderr=subprocess.STDOUT,
            start_new_session=True)
        recorder = subprocess.Popen(speed_sweep.ros_command([
            "python3", str(ROOT / "tools/assessment_recorder.py"),
            "--experiment", "navigation", "--run-status", "candidate",
            "--output", str(run_dir), "--timeout", str(args.timeout),
        ]), cwd=ROOT, env=env, stdout=recorder_handle, stderr=subprocess.STDOUT,
            start_new_session=True)
        time.sleep(args.startup_wait)
        service = subprocess.run(speed_sweep.ros_command([
            "ros2", "service", "call", "/drone/interactive_goals/execute",
            "drone_msgs/srv/ExecuteGoalSequence",
            speed_sweep.request_yaml(args.route),
        ]), cwd=ROOT, env=env, text=True, capture_output=True, timeout=45)
        (run_dir / "service.log").write_text(service.stdout + service.stderr)
        accepted = service_accepted(service.stdout, service.returncode)
        if not accepted:
            speed_sweep.terminate(recorder)
        else:
            recorder.wait(timeout=args.timeout + 20)
    except (subprocess.TimeoutExpired, KeyboardInterrupt) as error:
        (run_dir / "orchestrator_error.log").write_text(str(error) + "\n")
    finally:
        speed_sweep.terminate(recorder)
        speed_sweep.terminate(launch)
        launch_handle.close()
        recorder_handle.close()
    if (run_dir / "metadata.json").exists():
        with (run_dir / "analyzer.log").open("w") as handle:
            subprocess.run(speed_sweep.ros_command([
                "python3", str(ROOT / "tools/analyze_assessment_run.py"), str(run_dir),
                "--parameters", str(ROOT / "results/parameters"),
            ]), cwd=ROOT, env=env, stdout=handle, stderr=subprocess.STDOUT,
                timeout=90, check=False)
    record = speed_sweep.build_record(
        run_dir, args.candidate, args.route, args.run, A2_PARAMETERS,
        commit, dirty, domain)
    record["shortcut_preferred_clearance"] = parameters[
        "shortcut_preferred_clearance"]
    (run_dir / "run_record.json").write_text(
        json.dumps(record, indent=2, allow_nan=False) + "\n")
    if record["success"]:
        run_geometry(run_dir, env)
    summary = extract_run_summary(run_dir, record)
    (run_dir / "simplifier_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n")
    summarize(args.output)
    return 0 if record["success"] else 1


def read_summaries(output):
    return [json.loads(path.read_text()) for path in
            sorted(output.glob("*/**/run_*/simplifier_summary.json"))]


def flatten(row):
    result = {key: row.get(key) for key in COMPARISON_FIELDS}
    result["mission_time_s"] = row.get("mission_time_s")
    for name in FOCUS_POINTS:
        result[f"clearance_at_{name}"] = json.dumps(
            row.get("focus_clearances", {}).get(name), separators=(",", ":"))
    return result


def summarize(output):
    output.mkdir(parents=True, exist_ok=True)
    rows = read_summaries(output)
    baseline = next((row for row in rows if row["candidate"] == "s0" and
                     row["route"] == "full_map" and row["run"] == 1 and
                     row.get("simplified_points") is not None), None)
    for row in rows:
        row["selection"] = ""
        if baseline and row["route"] == "full_map":
            point_limit = 2.0 * baseline["simplified_points"]
            length_limit = 1.15 * baseline["simplified_path_length_m"]
            safe = (row["success"] and not row.get("collision") and
                    not row.get("nonfinite") and not row.get("saturated_at_end") and
                    row.get("tracking_max_m", math.inf) < 0.10 and
                    row.get("minimum_actual_clearance_m", -math.inf) > 0)
            row["selection"] = "pass" if (safe and
                row["simplified_points"] <= point_limit and
                row["simplified_path_length_m"] <= length_limit and
                row.get("mission_time_s", math.inf) <= 60.68) else "fail"
    payload = {"generated_at": datetime.now(timezone.utc).isoformat(), "runs": rows}
    (output / "candidate_comparison.json").write_text(
        json.dumps(payload, indent=2, allow_nan=False) + "\n")
    with (output / "candidate_comparison.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=COMPARISON_FIELDS)
        writer.writeheader()
        writer.writerows(flatten(row) for row in rows)
    aggregates = []
    for candidate in CANDIDATES:
        items = [row for row in rows if row["candidate"] == candidate and
                 row["route"] == "full_map"]
        times = [row["mission_time_s"] for row in items
                 if row.get("mission_time_s") is not None]
        aggregates.append({"candidate": candidate, "runs": len(items),
                           "successes": sum(row.get("success") is True for row in items),
                           "mean": statistics.fmean(times) if times else None,
                           "min": min(times) if times else None,
                           "max": max(times) if times else None,
                           "population_stddev": statistics.pstdev(times) if times else None})
    lines = ["# Simplifier clearance experiment", "",
             "All runs are candidate evidence from a clean commit.", "",
             "| candidate | full-map success/runs | mission mean (s) |",
             "|---|---:|---:|"]
    for item in aggregates:
        mean = "n/a" if item["mean"] is None else f"{item['mean']:.3f}"
        lines.append(f"| {item['candidate']} | {item['successes']}/{item['runs']} | {mean} |")
    (output / "simplifier_clearance_summary.md").write_text("\n".join(lines) + "\n")
    return rows


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", choices=CANDIDATES)
    parser.add_argument("--route", choices=speed_sweep.ROUTES)
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
        args.run = run
        status = max(status, run_once(args))
    return status


if __name__ == "__main__":
    sys.exit(main())
