#!/usr/bin/env python3
"""Assemble the selected vehicle-model upgrade metrics and required figures."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[1]
SWEEP = ROOT / "results/vehicle_model_upgrade/controller_envelope_sweep/sweep.json"
MULTI = ROOT / "results/vehicle_model_upgrade/controller_envelope_sweep/refined_drag_0p01/multi_goal"
OLD_MULTI = ROOT / "results/multi_goal_evaluation/default_mission/metrics.json"
CAPABILITY = ROOT / "results/vehicle_capability_audit/baseline/capability.json"
OUTPUT = ROOT / "results/vehicle_model_upgrade/selected"


def experiments() -> dict:
    items = json.loads(SWEEP.read_text())["candidates"]
    return {(item["candidate"], item["kind"]): item for item in items}


def bar_plot(path: Path, title: str, ylabel: str, labels: list[str], values: list[float]) -> None:
    figure, axis = plt.subplots(figsize=(7.5, 4.5))
    bars = axis.bar(labels, values, color=("#607d8b", "#2e7d32", "#ef6c00")[:len(values)])
    axis.bar_label(bars, fmt="%.3g", padding=3)
    axis.set(title=title, ylabel=ylabel); axis.grid(axis="y", alpha=0.3)
    figure.tight_layout(); figure.savefig(path, dpi=160); plt.close(figure)


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    data = experiments()
    names = ["old 0.4/0.08", "selected 0.8/0.15", "aggressive 1.5/0.20"]
    candidates = ["old_conservative", "a0p8_t0p15", "aggressive_not_selected"]
    multi = json.loads((MULTI / "metrics.json").read_text())
    old_multi = json.loads(OLD_MULTI.read_text())
    capability = json.loads(CAPABILITY.read_text())

    selected = {
        "aerodynamics": {"enable_aerodynamic_drag": True,
            "linear_drag_n_per_m_s": {"x": 0.01, "y": 0.01, "z": 0.01},
            "quadratic_drag_n_per_m_s_squared": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular_damping_nm_per_rad_s": {"roll": 0.01, "pitch": 0.01, "yaw": 0.02}},
        "controller": {"max_horizontal_acceleration_m_s2": 0.8,
                       "max_tilt_angle_rad": 0.15,
                       "horizontal_position_kp": 0.4,
                       "horizontal_velocity_kd": 1.2},
        "selection": "first scanned envelope eliminating step saturation with adequate tilt margin; low drag refined after 0.20 failed multi-goal tracking",
        "rejected": {"coarse_drag_linear_xy": 0.20,
                     "coarse_drag_multi_goal_max_tracking_error_m": 0.25002751799609124,
                     "aggressive_envelope": [1.5, 0.20]},
    }
    (OUTPUT / "selected_parameters.json").write_text(json.dumps(selected, indent=2) + "\n")
    shutil.copyfile(CAPABILITY, OUTPUT / "capability.json")

    comparison_rows = []
    for metric, kind, key in (
        ("horizontal_step_rise_time_s", "horizontal_step", "rise_time_s"),
        ("horizontal_step_settling_time_s", "horizontal_step", "settling_time_s"),
        ("horizontal_step_overshoot_m", "horizontal_step", "overshoot_m"),
        ("horizontal_step_max_tilt_rad", "horizontal_step", "maximum_tilt_rad"),
        ("horizontal_step_max_rpm", "horizontal_step", "maximum_rpm"),
        ("horizontal_step_horizontal_saturation_count", "horizontal_step", None),
        ("disturbance_0p3_max_offset_m", "disturbance_0p3", "maximum_horizontal_offset_m"),
        ("disturbance_0p3_recovery_time_s", "disturbance_0p3", None),
        ("disturbance_0p8_max_offset_m", "disturbance_0p8", "maximum_horizontal_offset_m"),
        ("disturbance_0p8_recovery_time_s", "disturbance_0p8", None),
        ("disturbance_0p8_horizontal_saturation_count", "disturbance_0p8", None),
    ):
        values = []
        for candidate in candidates:
            item = data[(candidate, kind)]
            if "saturation_count" in metric:
                value = item["saturation_counts"]["horizontal"]
            elif "recovery_time" in metric:
                # force begins at t=1, lasts 2 s; settling_time includes the final
                # 2 s stable confirmation interval.
                value = item["settling_time_s"] - 5.0
            else:
                value = item[key]
            values.append(value)
        comparison_rows.append([metric, *values])
    comparison_rows += [
        ["multi_goal_max_tracking_error_m", old_multi["maximum_tracking_error_m"], multi["maximum_tracking_error_m"], "not_run"],
        ["multi_goal_minimum_clearance_m", old_multi["minimum_clearance_m"], multi["minimum_clearance_m"], "not_run"],
        ["multi_goal_completion_time_s", old_multi["launch_to_complete_time_s"], multi["launch_to_complete_time_s"], "not_run"],
        ["multi_goal_max_rpm", old_multi["maximum_motor_rpm"], multi["maximum_motor_rpm"], "not_run"],
        ["multi_goal_saturation_count", old_multi["controller_saturation_count"], multi["controller_saturation_count"], "not_run"],
        ["multi_goal_final_error_m", old_multi["final_position_error_m"], multi["final_position_error_m"], "not_run"],
        ["multi_goal_final_speed_m_s", old_multi["final_speed_m_s"], multi["final_speed_m_s"], "not_run"],
    ]
    with (OUTPUT / "comparison.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n"); writer.writerow(["metric", "old", "selected", "aggressive"])
        writer.writerows(comparison_rows)

    bar_plot(OUTPUT/"horizontal_step.png", "2 m horizontal step settling time", "seconds", names,
             [data[(name,"horizontal_step")]["settling_time_s"] for name in candidates])
    bar_plot(OUTPUT/"diagonal_step.png", "2 m × 1 m diagonal step maximum tilt", "rad", names,
             [data[(name,"diagonal_step")]["maximum_tilt_rad"] for name in candidates])
    for kind, filename, force in (("disturbance_0p3","disturbance_0p3.png","0.30 N"),
                                  ("disturbance_0p8","disturbance_0p8.png","0.80 N")):
        bar_plot(OUTPUT/filename, f"{force} × 2 s maximum horizontal offset", "m", names,
                 [data[(name,kind)]["maximum_horizontal_offset_m"] for name in candidates])
    bar_plot(OUTPUT/"persistent_disturbance.png", "0.30 N × 10 s PD offset", "m", names,
             [data[(name,"disturbance_persistent")]["maximum_horizontal_offset_m"] for name in candidates])
    bar_plot(OUTPUT/"saturation_summary.png", "0.80 N horizontal saturation samples", "100 Hz samples", names,
             [data[(name,"disturbance_0p8")]["saturation_counts"]["horizontal"] for name in candidates])

    trajectory_path = MULTI / "trajectory.csv"
    if trajectory_path.exists():
        with trajectory_path.open(encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        times = [float(row["time"]) for row in rows if row["tracking_error"]]
        errors = [float(row["tracking_error"]) for row in rows if row["tracking_error"]]
        figure, axis = plt.subplots(figsize=(9, 4.5)); axis.plot(times, errors, linewidth=0.8)
        axis.axhline(0.04, color="red", linestyle="--", label="acceptance 0.04 m")
        axis.set(title="Selected multi-goal tracking", xlabel="time since launch [s]", ylabel="tracking error [m]")
        axis.grid(alpha=0.3); axis.legend(); figure.tight_layout()
        figure.savefig(OUTPUT/"multi_goal_tracking.png", dpi=160); plt.close(figure)

    thrust = capability["thrust_capability"]
    bar_plot(OUTPUT/"capability_summary.png", "Physical thrust capability", "N",
             ["weight", "max total thrust"], [thrust["weight_n"], thrust["four_motor_max_total_thrust_n"]])

    baseline = {"controller_envelope": {"max_horizontal_acceleration": 0.4, "max_tilt_angle": 0.08},
                "aerodynamics_enabled": False,
                "experiments": [item for item in data.values() if item["candidate"] == "old_conservative"]}
    baseline_dir = ROOT / "results/vehicle_model_upgrade/baseline"; baseline_dir.mkdir(parents=True, exist_ok=True)
    (baseline_dir/"metrics.json").write_text(json.dumps(baseline, indent=2)+"\n")


if __name__ == "__main__":
    main()
