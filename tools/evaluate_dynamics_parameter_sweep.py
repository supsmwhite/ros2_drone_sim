#!/usr/bin/env python3
"""Staged offline sweep for lumped translational drag and angular damping."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "src/drone_bringup/config/dynamics.yaml"
DEFAULT_OUTPUT = ROOT / "results/vehicle_model_upgrade/drag_sweep"
LINEAR = (0.0, 0.10, 0.20, 0.30)
LINEAR_REFINEMENT = (0.01, 0.02, 0.03, 0.05)
ANGULAR_RP = (0.0, 0.005, 0.010, 0.020)
ANGULAR_YAW = (0.0, 0.010, 0.020, 0.040)


def decay(mass_or_inertia: float, linear: float, quadratic: float = 0.0,
          initial: float = 1.0, dt: float = 0.005, duration: float = 10.0) -> dict:
    value = initial
    values = [value]
    half_time = None
    ten_percent_time = None
    monotonic = True
    for index in range(round(duration / dt)):
        acceleration = (-linear * value - quadratic * abs(value) * value) / mass_or_inertia
        candidate = value + acceleration * dt
        if value * candidate < 0.0:  # passive drag cannot reverse velocity by itself
            candidate = 0.0
        monotonic = monotonic and abs(candidate) <= abs(value) + 1.0e-15
        value = candidate
        values.append(value)
        elapsed = (index + 1) * dt
        if half_time is None and abs(value) <= 0.5 * abs(initial):
            half_time = elapsed
        if ten_percent_time is None and abs(value) <= 0.1 * abs(initial):
            ten_percent_time = elapsed
    return {"initial": initial, "final": value, "half_time_s": half_time,
            "ten_percent_time_s": ten_percent_time, "monotonic_energy": monotonic,
            "peak": max(abs(item) for item in values)}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    params = next(iter(yaml.safe_load(CONFIG.read_text()).values()))["ros__parameters"]
    mass = float(params["mass"])
    inertia = [float(params[f"inertia_{axis}{axis}"]) for axis in "xyz"]
    records = []

    # Stage 1: vary one translational family at a time, not a Cartesian product.
    for axis_group in ("xy", "z"):
        for coefficient in LINEAR:
            metric = decay(mass, coefficient)
            records.append({"stage": "translational", "axis_group": axis_group,
                            "coefficient": coefficient, **metric})
    # Closed-loop trajectory tracking rejects the coarse 0.10+ region, so refine
    # the low end instead of changing controller gains or acceptance thresholds.
    for axis_group in ("xy_refinement", "z_refinement"):
        for coefficient in LINEAR_REFINEMENT:
            records.append({"stage": "translational_refinement", "axis_group": axis_group,
                            "coefficient": coefficient, **decay(mass, coefficient)})

    # Stage 2: roll/pitch share a candidate and yaw is swept independently.
    for axis_group, candidates, moment in (
            ("roll_pitch", ANGULAR_RP, inertia[0]), ("yaw", ANGULAR_YAW, inertia[2])):
        for coefficient in candidates:
            metric = decay(moment, coefficient)
            records.append({"stage": "angular", "axis_group": axis_group,
                            "coefficient": coefficient, **metric})

    # Stage 3: only three meaningful combined candidates.
    combinations = [
        {"name": "no_drag", "linear_xy": 0.0, "linear_z": 0.0,
         "angular_roll_pitch": 0.0, "angular_yaw": 0.0},
        {"name": "coarse_balanced_rejected_by_closed_loop", "linear_xy": 0.20, "linear_z": 0.10,
         "angular_roll_pitch": 0.010, "angular_yaw": 0.020},
        {"name": "refined_low_drag", "linear_xy": 0.01, "linear_z": 0.01,
         "angular_roll_pitch": 0.010, "angular_yaw": 0.020},
        {"name": "high_damping_not_selected", "linear_xy": 0.30, "linear_z": 0.30,
         "angular_roll_pitch": 0.020, "angular_yaw": 0.040},
    ]
    combined = []
    for candidate in combinations:
        combined.append({**candidate,
            "xy": decay(mass, candidate["linear_xy"]),
            "z": decay(mass, candidate["linear_z"]),
            "roll_pitch": decay(inertia[0], candidate["angular_roll_pitch"]),
            "yaw": decay(inertia[2], candidate["angular_yaw"]),
        })

    args.output_dir.mkdir(parents=True, exist_ok=True)
    with (args.output_dir / "staged_sweep.csv").open("w", newline="", encoding="utf-8") as stream:
        fields = list(records[0])
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader(); writer.writerows(records)
    report = {
        "method": "200 Hz semi-implicit Euler passive free-decay, staged without Cartesian product",
        "initial_linear_velocity_m_s": 1.0,
        "initial_angular_rate_rad_s": 1.0,
        "quadratic_drag_n_per_m_s_squared": 0.0,
        "staged_sweep": records,
        "combined_validation": combined,
        "recommended": combinations[2],
        "interpretation": (
            "These are lumped aerodynamic damping coefficients, not wind-tunnel identification "
            "for a specific airframe. They make medium/low-speed free decay more plausible than "
            "an undamped rigid body without freezing normal controlled flight."),
    }
    (args.output_dir / "sweep.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report["recommended"], indent=2))


if __name__ == "__main__":
    main()
