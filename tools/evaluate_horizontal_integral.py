#!/usr/bin/env python3
"""Evaluate horizontal integral control in isolated ROS 2 simulations."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import yaml

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "results/horizontal_integral_upgrade"
CONTROLLER = ROOT / "src/drone_bringup/config/controller.yaml"
DYNAMICS = ROOT / "src/drone_bringup/config/dynamics.yaml"
TARGET = (0.0, 0.0, 1.5)


def load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def write_configs(directory: Path, *, enabled: bool, ki: float, kaw: float,
                  unload_gain: float, limit: float, mass: float = 1.0) -> tuple[Path, Path]:
    controller = load_yaml(CONTROLLER)
    dynamics = load_yaml(DYNAMICS)
    cp = controller["position_controller_node"]["ros__parameters"]
    dp = dynamics["quadrotor_dynamics_node"]["ros__parameters"]
    cp.update({
        "enable_horizontal_integral": enabled,
        "horizontal_position_ki_x": ki,
        "horizontal_position_ki_y": ki,
        "horizontal_integral_acceleration_limit": limit,
        "horizontal_anti_windup_gain": kaw,
        "horizontal_integrator_unload_gain": unload_gain,
        "horizontal_integral_capture_radius": 0.50,
        "horizontal_integral_reset_distance": 1.0,
    })
    dp["mass"] = mass
    controller_path = directory / "controller.yaml"
    dynamics_path = directory / "dynamics.yaml"
    controller_path.write_text(yaml.safe_dump(controller, sort_keys=False), encoding="utf-8")
    dynamics_path.write_text(yaml.safe_dump(dynamics, sort_keys=False), encoding="utf-8")
    return controller_path, dynamics_path


def stop_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=3)


def rotate(q, vector):
    x, y, z, w = q
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    vx, vy, vz = vector
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return (vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx)


def euler(q):
    x, y, z, w = q
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return (
        math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
        math.asin(max(-1.0, min(1.0, 2*(w*y-z*x)))),
        math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)),
    )


def save_csv(samples: list[dict], path: Path) -> None:
    if not samples:
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(samples[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(samples)


def save_plot(samples: list[dict], path: Path, title: str) -> None:
    times = [row["time_s"] for row in samples]
    figure, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    axes[0].plot(times, [row["horizontal_error_m"] for row in samples], label="horizontal error")
    axes[0].plot(times, [row["x_m"] for row in samples], label="x")
    axes[0].set_ylabel("Position [m]")
    axes[1].plot(times, [row["speed_m_s"] for row in samples], label="speed")
    axes[1].plot(times, [row["ix_m_s2"] for row in samples], label="I x")
    axes[1].plot(times, [row["iy_m_s2"] for row in samples], label="I y")
    axes[1].set_ylabel("Speed / accel")
    axes[2].plot(times, [row["force_x_n"] for row in samples], label="force x")
    axes[2].plot(times, [row["raw_acceleration_m_s2"] for row in samples], label="raw accel")
    axes[2].plot(times, [row["acceleration_m_s2"] for row in samples], label="limited accel")
    axes[2].set(xlabel="Time [s]", ylabel="Force / accel")
    for axis in axes:
        axis.grid(True)
        axis.legend(fontsize=8, ncol=3)
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def worker(spec_path: Path) -> int:
    spec = json.loads(spec_path.read_text(encoding="utf-8"))
    import rclpy
    from drone_msgs.msg import ControllerDiagnostics, MotorRPM
    from geometry_msgs.msg import PoseStamped, WrenchStamped
    from nav_msgs.msg import Odometry

    output = Path(spec["output"])
    output.mkdir(parents=True, exist_ok=True)
    launch_log = (output / "launch.log").open("w", encoding="utf-8")
    process = subprocess.Popen([
        "ros2", "launch", "drone_bringup", "disturbance_hover_sim.launch.py",
        "use_rviz:=false", f"controller_config:={spec['controller']}",
        f"dynamics_config:={spec['dynamics']}",
    ], stdout=launch_log, stderr=subprocess.STDOUT, text=True,
       start_new_session=True, env=os.environ.copy())
    rclpy.init()
    node = rclpy.create_node("horizontal_integral_evaluator")
    goal_pub = node.create_publisher(PoseStamped, "/drone/goal", 10)
    force_pub = node.create_publisher(WrenchStamped, "/drone/external_wrench", 10)
    latest: dict = {}
    samples: list[dict] = []
    rpm = (0.0, 0.0, 0.0, 0.0)
    diag = None
    phase = "TAKEOFF"
    start = time.monotonic()
    commanded_force = 0.0

    def on_rpm(message):
        nonlocal rpm
        rpm = (message.m1_front_left_ccw_rpm, message.m2_rear_left_cw_rpm,
               message.m3_rear_right_ccw_rpm, message.m4_front_right_cw_rpm)

    def on_diag(message):
        nonlocal diag
        diag = message

    def on_odom(message):
        pose = message.pose.pose
        q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        velocity = rotate(q, (message.twist.twist.linear.x, message.twist.twist.linear.y,
                              message.twist.twist.linear.z))
        angles = euler(q)
        position = (pose.position.x, pose.position.y, pose.position.z)
        latest.update(position=position, velocity=velocity, angles=angles)
        if phase == "TAKEOFF" or diag is None:
            return
        horizontal_error = math.hypot(position[0], position[1])
        samples.append({
            "time_s": time.monotonic()-start, "phase": phase,
            "x_m": position[0], "y_m": position[1], "z_m": position[2],
            "vx_m_s": velocity[0], "vy_m_s": velocity[1], "vz_m_s": velocity[2],
            "horizontal_error_m": horizontal_error,
            "position_error_m": math.dist(position, TARGET),
            "speed_m_s": math.sqrt(sum(value*value for value in velocity)),
            "roll_rad": angles[0], "pitch_rad": angles[1],
            "ix_m_s2": diag.horizontal_i_acceleration_x,
            "iy_m_s2": diag.horizontal_i_acceleration_y,
            "integral_norm_m_s2": math.hypot(diag.horizontal_i_acceleration_x,
                                                diag.horizontal_i_acceleration_y),
            "raw_acceleration_m_s2": math.hypot(diag.horizontal_raw_acceleration_x,
                                                   diag.horizontal_raw_acceleration_y),
            "acceleration_m_s2": math.hypot(diag.horizontal_acceleration_x,
                                               diag.horizontal_acceleration_y),
            "force_x_n": commanded_force, "max_rpm": max(rpm),
            "horizontal_saturated": int(diag.horizontal_saturated),
            "altitude_saturated": int(diag.altitude_saturated),
            "attitude_saturated": int(diag.attitude_saturated),
            "mixer_saturated": int(diag.mixer_saturated),
            "integral_enabled": int(diag.horizontal_integral_enabled),
            "integral_frozen": int(diag.horizontal_integral_frozen),
            "integral_reset": int(diag.horizontal_integral_reset),
            "saturation_backcalc_active": int(
                diag.horizontal_saturation_backcalc_active),
            "integrator_unloading_active": int(
                diag.horizontal_integrator_unloading_active),
        })

    subscriptions = [
        node.create_subscription(Odometry, "/drone/odom", on_odom, 20),
        node.create_subscription(MotorRPM, "/drone/motor_rpm_cmd", on_rpm, 20),
        node.create_subscription(ControllerDiagnostics, "/drone/controller/diagnostics", on_diag, 20),
    ]

    def publish_goal():
        message = PoseStamped()
        message.header.frame_id = "map"
        message.pose.position.z = TARGET[2]
        message.pose.orientation.w = 1.0
        goal_pub.publish(message)

    def publish_force(value: float):
        nonlocal commanded_force
        commanded_force = value
        message = WrenchStamped()
        message.header.frame_id = "map"
        message.wrench.force.x = value
        force_pub.publish(message)

    def spin_once():
        publish_goal()
        rclpy.spin_once(node, timeout_sec=0.01)
        if process.poll() is not None:
            raise RuntimeError(f"launch exited with {process.returncode}")

    def run_phase(name: str, duration: float, force: float):
        nonlocal phase
        phase = name
        deadline = time.monotonic() + duration
        next_force = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_force:
                publish_force(force)
                next_force = time.monotonic() + 0.04
            spin_once()

    error = None
    try:
        scenario = spec["scenario"]
        deadline = time.monotonic() + 30.0
        takeoff_start = time.monotonic()
        stable_since = None
        while time.monotonic() < deadline:
            spin_once()
            if "position" not in latest:
                continue
            position_error = math.dist(latest["position"], TARGET)
            speed = math.sqrt(sum(value*value for value in latest["velocity"]))
            stable_since = stable_since or time.monotonic() if position_error < 0.03 and speed < 0.03 else None
            if stable_since and time.monotonic()-stable_since >= 1.0:
                break
            if scenario == "mass" and time.monotonic()-takeoff_start >= 12.0:
                break
        if stable_since is None and scenario != "mass":
            raise RuntimeError("takeoff failed to stabilize")
        start = time.monotonic()
        run_phase("BASELINE", 1.0, 0.0)
        if scenario == "hover":
            run_phase("HOLD", 15.0, 0.0)
        elif scenario == "short":
            run_phase("DISTURBANCE", 2.0, float(spec["force"]))
            run_phase("RECOVERY", 10.0, 0.0)
        elif scenario in ("persistent", "bias"):
            run_phase("DISTURBANCE", 15.0, float(spec["force"]))
        elif scenario == "release":
            run_phase("DISTURBANCE", 10.0, float(spec["force"]))
            run_phase("RECOVERY", 10.0, 0.0)
        elif scenario == "mass":
            run_phase("HOLD", 15.0, 0.0)
        else:
            raise ValueError(f"unknown scenario {scenario}")
    except Exception as exception:
        error = str(exception)
    finally:
        publish_force(0.0)
        for _ in range(3):
            rclpy.spin_once(node, timeout_sec=0.02)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        stop_process(process)
        launch_log.close()

    if not samples:
        (output / "metrics.json").write_text(json.dumps({"error": error or "no samples"}, indent=2)+"\n")
        return 1
    final = samples[-1]
    disturbance = [row for row in samples if row["phase"] == "DISTURBANCE"]
    recovery = [row for row in samples if row["phase"] == "RECOVERY"]
    last3_start = samples[-1]["time_s"] - 3.0
    last3 = [row for row in samples if row["time_s"] >= last3_start]
    recovery_start = recovery[0]["time_s"] if recovery else None
    recovery_time = None
    if recovery:
        for index, row in enumerate(recovery):
            if row["horizontal_error_m"] < 0.05 and row["speed_m_s"] < 0.03:
                window_start = row["time_s"]
                window = [item for item in recovery[index:] if item["time_s"] <= window_start+1.0]
                if window and window[-1]["time_s"] >= window_start+0.95 and all(
                    item["horizontal_error_m"] < 0.05 and item["speed_m_s"] < 0.03 for item in window):
                    recovery_time = window_start-recovery_start
                    break
    reverse_overshoot = max([0.0] + [-row["x_m"] for row in recovery])
    disturbance_tail = []
    if disturbance:
        disturbance_tail_start = disturbance[-1]["time_s"] - 3.0
        disturbance_tail = [
            row for row in disturbance if row["time_s"] >= disturbance_tail_start]
    unloading_recovery = [
        row for row in recovery if row["integrator_unloading_active"]]
    metrics = {
        "scenario": spec["scenario"], "force_n": spec.get("force", 0.0),
        "parameters": spec["parameters"], "dynamics_mass_kg": spec.get("mass", 1.0),
        "maximum_horizontal_offset_m": max(row["horizontal_error_m"] for row in samples),
        "recovery_time_s": recovery_time,
        "reverse_overshoot_m": reverse_overshoot,
        "final_position_error_m": final["position_error_m"],
        "final_horizontal_error_m": final["horizontal_error_m"],
        "final_speed_m_s": final["speed_m_s"],
        "final_altitude_m": final["z_m"],
        "last_3s_average_error_m": sum(row["horizontal_error_m"] for row in last3)/len(last3),
        "last_3s_maximum_error_m": max(row["horizontal_error_m"] for row in last3),
        "last_3s_average_speed_m_s": sum(row["speed_m_s"] for row in last3)/len(last3),
        "last_3s_average_integral_x_m_s2": sum(row["ix_m_s2"] for row in last3)/len(last3),
        "peak_integral_acceleration_m_s2": max(row["integral_norm_m_s2"] for row in samples),
        "disturbance_last_3s_average_error_m": (
            sum(row["horizontal_error_m"] for row in disturbance_tail) / len(disturbance_tail)
            if disturbance_tail else None),
        "integrator_unloading_start_s": (
            unloading_recovery[0]["time_s"] - recovery_start
            if unloading_recovery and recovery_start is not None else None),
        "maximum_tilt_rad": max(math.hypot(row["roll_rad"], row["pitch_rad"]) for row in samples),
        "maximum_rpm": max(row["max_rpm"] for row in samples),
        "saturation_counts": {name: sum(row[f"{name}_saturated"] for row in samples)
                              for name in ("horizontal", "altitude", "attitude", "mixer")},
        "saturation_backcalc_samples": sum(
            row["saturation_backcalc_active"] for row in samples),
        "integrator_unloading_samples": sum(
            row["integrator_unloading_active"] for row in samples),
        "integral_reset_samples": sum(row["integral_reset"] for row in samples),
        "sample_count": len(samples), "error": error,
    }
    save_csv(samples, output / "samples.csv")
    save_plot(samples, output / "summary.png", spec["name"])
    (output / "metrics.json").write_text(json.dumps(metrics, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(metrics, indent=2), flush=True)
    return 0 if error is None else 1


def run_case(temporary: Path, output: Path, domain: int, *, name: str, scenario: str,
             enabled: bool, ki: float, kaw: float, unload_gain: float, limit: float,
             force: float = 0.0, mass: float = 1.0) -> dict:
    case_temp = temporary / name
    case_temp.mkdir(parents=True)
    controller, dynamics = write_configs(
        case_temp, enabled=enabled, ki=ki, kaw=kaw,
        unload_gain=unload_gain, limit=limit, mass=mass)
    output.mkdir(parents=True, exist_ok=True)
    parameters = {"enabled": enabled, "ki": ki, "kaw": kaw,
                  "unload_gain": unload_gain, "limit_m_s2": limit,
                  "capture_radius_m": 0.5, "reset_distance_m": 1.0}
    spec = {"name": name, "scenario": scenario, "force": force, "mass": mass,
            "controller": str(controller), "dynamics": str(dynamics),
            "output": str(output), "parameters": parameters}
    spec_path = case_temp / "spec.json"
    spec_path.write_text(json.dumps(spec), encoding="utf-8")
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain)
    completed = subprocess.run([sys.executable, __file__, "--worker", str(spec_path)],
                               env=env, check=False)
    metrics = json.loads((output / "metrics.json").read_text(encoding="utf-8"))
    metrics["ros_domain_id"] = domain
    metrics["returncode"] = completed.returncode
    (output / "metrics.json").write_text(
        json.dumps(metrics, indent=2)+"\n", encoding="utf-8")
    return metrics


def write_selected_artifacts() -> None:
    selected = BASE / "selected"
    selected.mkdir(parents=True, exist_ok=True)

    def metrics(directory: Path) -> dict:
        path = directory / "metrics.json"
        return json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}

    pd = {name: metrics(BASE/"baseline_pd"/name) for name in
          ("hover", "short_0p3", "short_0p8", "persistent", "release")}
    pid = {name: metrics(selected/name) for name in
           ("hover", "short_0p3", "short_0p8", "persistent", "release")}
    parameters = {
        "enable_horizontal_integral": True,
        "horizontal_position_ki_x": 0.15,
        "horizontal_position_ki_y": 0.15,
        "horizontal_integral_acceleration_limit": 0.35,
        "horizontal_anti_windup_gain": 2.0,
        "horizontal_integrator_unload_gain": 2.0,
        "horizontal_integral_capture_radius": 0.5,
        "horizontal_integral_reset_distance": 1.0,
        "max_horizontal_acceleration": 0.8,
        "max_tilt_angle": 0.15,
        "horizontal_position_kp": 0.4,
        "horizontal_velocity_kd": 1.2,
    }
    (selected/"selected_parameters.json").write_text(
        json.dumps(parameters, indent=2)+"\n", encoding="utf-8")
    comparison_rows = []
    multi_goal = metrics(selected/"multi_goal")
    for controller, source in (("PD baseline", pd), ("selected PID-like", pid)):
        comparison_rows.append({
            "controller": controller,
            "persistent_last_3s_error_m": source["persistent"].get("last_3s_average_error_m"),
            "short_0p3_max_offset_m": source["short_0p3"].get("maximum_horizontal_offset_m"),
            "short_0p3_recovery_s": source["short_0p3"].get("recovery_time_s"),
            "short_0p8_max_offset_m": source["short_0p8"].get("maximum_horizontal_offset_m"),
            "short_0p8_recovery_s": source["short_0p8"].get("recovery_time_s"),
            "release_reverse_overshoot_m": source["release"].get("reverse_overshoot_m"),
            "peak_integral_m_s2": max((item.get("peak_integral_acceleration_m_s2", 0.0)
                                        for item in source.values()), default=0.0),
            "horizontal_saturation_samples": source["short_0p8"].get(
                "saturation_counts", {}).get("horizontal"),
            "multi_goal_max_tracking_error_m": (
                None if controller == "PD baseline" else multi_goal.get("maximum_tracking_error_m")),
            "multi_goal_minimum_clearance_m": (
                None if controller == "PD baseline" else multi_goal.get("minimum_clearance_m")),
        })
    with (selected/"comparison.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(comparison_rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(comparison_rows)
    test_xml_files = sorted(
        ROOT.glob("build/*/test_results/**/*.xml"),
        key=lambda path: str(path))
    ctest_xml_files = []
    for package_testing in sorted(ROOT.glob("build/*/Testing")):
        candidates = list(package_testing.glob("*/Test.xml"))
        if candidates:
            ctest_xml_files.append(max(candidates, key=lambda path: path.stat().st_mtime_ns))

    def relative(path: Path) -> str:
        return str(path.relative_to(ROOT))

    def full_test_summary(result_paths: list[Path], ctest_paths: list[Path]) -> dict | None:
        if not result_paths and not ctest_paths:
            return None
        totals = {"tests": 0, "errors": 0, "failures": 0, "skipped": 0}
        parsed = 0
        for path in result_paths:
            try:
                root = ET.parse(path).getroot()
            except (ET.ParseError, OSError):
                continue
            suites = [root] if root.tag == "testsuite" else list(root.findall("testsuite"))
            for suite in suites:
                totals["tests"] += int(suite.attrib.get("tests", 0))
                totals["errors"] += int(suite.attrib.get("errors", 0))
                totals["failures"] += int(suite.attrib.get("failures", 0))
                totals["skipped"] += int(
                    suite.attrib.get("skipped", suite.attrib.get("disabled", 0)))
                parsed += 1
        for path in ctest_paths:
            try:
                tests = ET.parse(path).getroot().findall("./Testing/Test")
            except (ET.ParseError, OSError):
                continue
            for test in tests:
                status = test.attrib.get("Status", "").lower()
                totals["tests"] += 1
                if status == "notrun":
                    totals["skipped"] += 1
                elif status != "passed":
                    totals["failures"] += 1
                parsed += 1
        return totals if parsed else None

    def named_test_result(fragment: str) -> tuple[dict | None, str | None]:
        matches = [path for path in test_xml_files if fragment in path.name]
        if not matches:
            return None, None
        path = max(matches, key=lambda item: item.stat().st_mtime_ns)
        try:
            root = ET.parse(path).getroot()
        except (ET.ParseError, OSError):
            return None, relative(path)
        suites = [root] if root.tag == "testsuite" else list(root.findall("testsuite"))
        tests = sum(int(suite.attrib.get("tests", 0)) for suite in suites)
        errors = sum(int(suite.attrib.get("errors", 0)) for suite in suites)
        failures = sum(int(suite.attrib.get("failures", 0)) for suite in suites)
        skipped = sum(int(suite.attrib.get("skipped", 0)) for suite in suites)
        return {
            "status": "passed" if tests > 0 and errors == 0 and failures == 0 else "failed",
            "tests": tests, "errors": errors, "failures": failures, "skipped": skipped,
        }, relative(path)

    interactive_navigation, interactive_source = named_test_result(
        "test_interactive_goal_navigation_e2e")
    preflight_failure, preflight_source = named_test_result(
        "test_interactive_preflight_failure")
    full_test = full_test_summary(test_xml_files, ctest_xml_files)
    multi_goal_path = selected / "multi_goal" / "metrics.json"
    repeat_results_path = BASE / "repeat_results.json"
    repeat_results = (
        json.loads(repeat_results_path.read_text(encoding="utf-8")).get("results", [])
        if repeat_results_path.exists() else [])
    git_commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, check=True,
        text=True, stdout=subprocess.PIPE).stdout.strip()
    regression = {
        "git_commit": git_commit,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "generated_from": {
            "full_test": [
                relative(path) for path in test_xml_files + ctest_xml_files] or None,
            "multi_goal": relative(multi_goal_path) if multi_goal_path.exists() else None,
            "interactive_navigation": interactive_source,
            "preflight_failure": preflight_source,
            "release_repetitions": (
                relative(repeat_results_path) if repeat_results_path.exists() else None),
        },
        "pd_baseline": pd,
        "selected": pid,
        "mass_mismatch": {name: metrics(selected/name) for name in ("mass_1p1", "mass_1p2")},
        "constant_bias": {name: metrics(selected/name) for name in ("bias_0p15", "bias_0p3")},
        "multi_goal": multi_goal,
        "release_repetitions": {
            f"run_{index}": result for index, result in enumerate(repeat_results, start=1)
        } if repeat_results else "not_run",
        "interactive_navigation": interactive_navigation or "not_run",
        "preflight_failure": preflight_failure or "not_run",
        "full_test": full_test or "not_run",
    }
    (selected/"regression_summary.json").write_text(
        json.dumps(regression, indent=2)+"\n", encoding="utf-8")
    copies = (
        (selected/"hover"/"summary.png", "hover_no_disturbance.png"),
        (selected/"short_0p3"/"summary.png", "short_disturbance_0p3.png"),
        (selected/"short_0p8"/"summary.png", "short_disturbance_0p8.png"),
        (selected/"persistent"/"summary.png", "persistent_disturbance.png"),
        (selected/"release"/"summary.png", "disturbance_release.png"),
        (selected/"persistent"/"summary.png", "integral_state.png"),
        (selected/"release"/"summary.png", "anti_windup_state.png"),
        (selected/"multi_goal"/"tracking_error.png", "multi_goal_tracking.png"),
    )
    for source, destination in copies:
        if source.exists():
            shutil.copyfile(source, selected/destination)


def parent(arguments) -> int:
    if arguments.summarize_only:
        write_selected_artifacts()
        return 0
    BASE.mkdir(parents=True, exist_ok=True)
    results = []
    domain = arguments.domain_start
    with tempfile.TemporaryDirectory(prefix="horizontal-integral-") as directory:
        temporary = Path(directory)

        def execute(output, **kwargs):
            nonlocal domain
            kwargs.setdefault("unload_gain", arguments.unload_gain)
            result = run_case(temporary, output, domain, **kwargs)
            results.append(result)
            print(kwargs["name"], "ok" if result["returncode"] == 0 else "failed", flush=True)
            domain = 120 if domain >= 232 else domain + 1

        if arguments.stage in ("baseline", "all"):
            for scenario, force in (("hover", 0.0), ("short", 0.3), ("short", 0.8),
                                    ("persistent", 0.3), ("release", 0.3)):
                suffix = scenario if scenario != "short" else f"short_{str(force).replace('.', 'p')}"
                execute(BASE/"baseline_pd"/suffix, name=f"pd_{suffix}", scenario=scenario,
                        enabled=False, ki=0.0, kaw=1.0, limit=0.35, force=force)
        if arguments.stage in ("ki", "all"):
            for ki in (0.05, 0.10, 0.15, 0.20):
                label = str(ki).replace(".", "p")
                execute(BASE/"ki_scan"/f"ki_{label}", name=f"ki_{label}", scenario="persistent",
                        enabled=True, ki=ki, kaw=1.0, limit=0.35, force=0.3)
        if arguments.stage in ("kaw", "all"):
            for kaw in (0.5, 1.0, 2.0):
                for force in (0.3, 0.8):
                    label = f"kaw_{str(kaw).replace('.', 'p')}_force_{str(force).replace('.', 'p')}"
                    execute(BASE/"anti_windup_scan"/label, name=label, scenario="short",
                            enabled=True, ki=arguments.ki, kaw=kaw, limit=0.35, force=force)
        if arguments.stage in ("limit", "all"):
            for limit in (0.25, 0.35, 0.45):
                label = f"limit_{str(limit).replace('.', 'p')}"
                execute(BASE/"integral_limit_scan"/label, name=label, scenario="persistent",
                        enabled=True, ki=arguments.ki, kaw=arguments.kaw, limit=limit, force=0.3)
        if arguments.stage in ("selected", "all"):
            selected_filter = set(arguments.selected_only.split(",")) if arguments.selected_only else None
            for scenario, force in (("hover", 0.0), ("short", 0.3), ("short", 0.8),
                                    ("persistent", 0.3), ("release", 0.3)):
                suffix = scenario if scenario != "short" else f"short_{str(force).replace('.', 'p')}"
                if selected_filter is not None and suffix not in selected_filter:
                    continue
                execute(BASE/"selected"/suffix, name=f"selected_{suffix}", scenario=scenario,
                        enabled=True, ki=arguments.ki, kaw=arguments.kaw,
                        limit=arguments.limit, force=force)
            for mass in (1.1, 1.2):
                label = f"mass_{str(mass).replace('.', 'p')}"
                if selected_filter is not None and label not in selected_filter:
                    continue
                execute(BASE/"selected"/label, name=label, scenario="mass", enabled=True,
                        ki=arguments.ki, kaw=arguments.kaw, limit=arguments.limit, mass=mass)
            for force in (0.15, 0.30):
                label = f"bias_{str(force).replace('.', 'p')}"
                if selected_filter is not None and label not in selected_filter:
                    continue
                execute(BASE/"selected"/label, name=label, scenario="bias", enabled=True,
                        ki=arguments.ki, kaw=arguments.kaw, limit=arguments.limit, force=force)
        if arguments.stage == "repeat":
            for run in range(1, 4):
                execute(
                    BASE/"selected"/f"release_run_{run}",
                    name=f"selected_release_run_{run}", scenario="release",
                    enabled=True, ki=arguments.ki, kaw=arguments.kaw,
                    limit=arguments.limit, force=0.3)
    summary = BASE / f"{arguments.stage}_results.json"
    summary.write_text(json.dumps({"results": results}, indent=2)+"\n", encoding="utf-8")
    if arguments.stage in ("selected", "all"):
        write_selected_artifacts()
    return 0 if all(item.get("returncode") == 0 for item in results) else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--stage",
        choices=("baseline", "ki", "kaw", "limit", "selected", "repeat", "all"),
                        default="all")
    parser.add_argument("--ki", type=float, default=0.10)
    parser.add_argument("--kaw", type=float, default=1.0)
    parser.add_argument("--unload-gain", type=float, choices=(1.0, 2.0), default=2.0)
    parser.add_argument("--limit", type=float, default=0.35)
    parser.add_argument("--domain-start", type=int, default=150)
    parser.add_argument("--selected-only", default="")
    parser.add_argument("--worker", type=Path)
    parser.add_argument("--summarize-only", action="store_true")
    arguments = parser.parse_args()
    return worker(arguments.worker) if arguments.worker else parent(arguments)


if __name__ == "__main__":
    raise SystemExit(main())
