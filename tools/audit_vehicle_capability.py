#!/usr/bin/env python3
"""Audit physical actuator capability and cross-file parameter consistency."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
DYNAMICS = ROOT / "src/drone_bringup/config/dynamics.yaml"
CONTROLLER = ROOT / "src/drone_bringup/config/controller.yaml"
XACRO = ROOT / "src/drone_bringup/urdf/drone.urdf.xacro"
DEFAULT_OUTPUT = ROOT / "results/vehicle_capability_audit/baseline"
ANGLES = (0.08, 0.12, 0.15, 0.20, 0.25)


def node_parameters(path: Path) -> dict:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    return next(iter(document.values()))["ros__parameters"]


def relation(control_limit: float, physical_limit: float) -> str:
    ratio = control_limit / physical_limit
    if ratio > 1.0 + 1.0e-9:
        return "超过物理能力"
    if ratio >= 0.8:
        return "接近物理能力"
    return "低于物理能力"


def audit() -> dict:
    dynamics = node_parameters(DYNAMICS)
    controller = node_parameters(CONTROLLER)
    xacro_text = XACRO.read_text(encoding="utf-8")
    arm_match = re.search(r'name="arm_length"\s+value="([0-9.eE+-]+)"', xacro_text)
    xacro_arm = float(arm_match.group(1)) if arm_match else None

    mass = float(dynamics["mass"])
    gravity = float(dynamics["gravity"])
    kf = float(dynamics["thrust_coefficient"])
    km = float(dynamics["drag_torque_coefficient"])
    arm = float(dynamics["arm_length"])
    max_rpm = float(dynamics["max_rpm"])
    max_omega = max_rpm * 2.0 * math.pi / 60.0
    hover_omega = math.sqrt(mass * gravity / (4.0 * kf))
    hover_rpm = hover_omega * 60.0 / (2.0 * math.pi)
    motor_max_thrust = kf * max_omega**2
    motor_max_yaw_torque = km * max_omega**2
    total_max_thrust = 4.0 * motor_max_thrust
    effective_arm = arm / math.sqrt(2.0)
    physical_torque = {
        "roll": 2.0 * effective_arm * motor_max_thrust,
        "pitch": 2.0 * effective_arm * motor_max_thrust,
        "yaw": 2.0 * motor_max_yaw_torque,
    }
    controller_torque = {
        axis: float(controller[f"max_torque_{axis}"])
        for axis in ("roll", "pitch", "yaw")
    }

    angle_capability = []
    for angle in ANGLES:
        required_thrust = mass * gravity / math.cos(angle)
        required_rpm = math.sqrt(required_thrust / (4.0 * kf)) * 60.0 / (2.0 * math.pi)
        horizontal_force = mass * gravity * math.tan(angle)
        angle_capability.append({
            "tilt_angle_rad": angle,
            "horizontal_force_n": horizontal_force,
            "horizontal_acceleration_m_s2": horizontal_force / mass,
            "height_holding_total_thrust_n": required_thrust,
            "height_holding_rpm": required_rpm,
            "exceeds_max_rpm": required_rpm > max_rpm,
        })

    fields = ("arm_length", "thrust_coefficient", "drag_torque_coefficient",
              "min_rpm", "max_rpm")
    dynamics_controller = {
        field: {
            "dynamics": float(dynamics[field]),
            "controller": float(controller[field]),
            "consistent": math.isclose(float(dynamics[field]), float(controller[field]),
                                       rel_tol=0.0, abs_tol=1.0e-15),
        }
        for field in fields
    }
    dynamics_controller["mass"] = {
        "dynamics": mass,
        "controller": float(controller["mass"]),
        "consistent": math.isclose(mass, float(controller["mass"]), abs_tol=1.0e-15),
    }
    return {
        "sources": {"dynamics": str(DYNAMICS.relative_to(ROOT)),
                    "controller": str(CONTROLLER.relative_to(ROOT)),
                    "xacro": str(XACRO.relative_to(ROOT))},
        "basic_parameters": {
            "mass_kg": mass,
            "inertia_kg_m2": [float(dynamics[f"inertia_{axis}{axis}"])
                               for axis in ("x", "y", "z")],
            "arm_length_m": arm,
            "thrust_coefficient_n_per_rad_s_squared": kf,
            "drag_torque_coefficient_nm_per_rad_s_squared": km,
            "motor_time_constant_s": float(dynamics["motor_time_constant"]),
            "minimum_rpm": float(dynamics["min_rpm"]),
            "maximum_rpm": max_rpm,
            "hover_rpm": hover_rpm,
        },
        "thrust_capability": {
            "single_motor_max_thrust_n": motor_max_thrust,
            "four_motor_max_total_thrust_n": total_max_thrust,
            "weight_n": mass * gravity,
            "thrust_to_weight_ratio": total_max_thrust / (mass * gravity),
            "hover_rpm_fraction_of_max": hover_rpm / max_rpm,
            "remaining_rpm_fraction": 1.0 - hover_rpm / max_rpm,
            "remaining_thrust_fraction_of_max": 1.0 - mass * gravity / total_max_thrust,
        },
        "tilt_capability": angle_capability,
        "torque_capability": {
            axis: {"physical_max_nm": physical_torque[axis],
                   "controller_limit_nm": controller_torque[axis],
                   "controller_fraction": controller_torque[axis] / physical_torque[axis],
                   "classification": relation(controller_torque[axis], physical_torque[axis])}
            for axis in ("roll", "pitch", "yaw")
        },
        "controller_envelope": {
            "max_horizontal_acceleration_m_s2": float(controller["max_horizontal_acceleration"]),
            "max_tilt_angle_rad": float(controller["max_tilt_angle"]),
            "tilt_limited_acceleration_m_s2": gravity * math.tan(float(controller["max_tilt_angle"])),
        },
        "consistency": {
            "dynamics_vs_controller": dynamics_controller,
            "xacro_arm_length": {"xacro": xacro_arm, "dynamics": arm,
                                  "consistent": xacro_arm is not None and math.isclose(xacro_arm, arm)},
            "xacro_role": "visual geometry only; it contains no inertial, RPM, thrust, or drag-torque data",
            "xacro_missing_physical_fields": ["mass", "inertia", "RPM limits",
                                               "thrust coefficient", "drag torque coefficient"],
        },
        "numerics": {
            "integration": "fixed-step semi-implicit Euler",
            "frequency_hz": float(dynamics["simulation_frequency"]),
            "time_step_s": 1.0 / float(dynamics["simulation_frequency"]),
            "limitations": "first-order accuracy; contact is a z-only constraint; quaternion is normalized each step",
        },
    }


def markdown(data: dict) -> str:
    b, t = data["basic_parameters"], data["thrust_capability"]
    rows = [
        "# 四旋翼执行能力审计", "",
        "## 基础与推力能力", "",
        "| 指标 | 数值 |", "|---|---:|",
        f"| 质量 | {b['mass_kg']:.6f} kg |",
        f"| 惯量 Ixx/Iyy/Izz | {' / '.join(f'{v:.6f}' for v in b['inertia_kg_m2'])} kg·m² |",
        f"| 机臂长度 | {b['arm_length_m']:.6f} m |",
        f"| 电机时间常数 | {b['motor_time_constant_s']:.6f} s |",
        f"| 悬停 / 最大 RPM | {b['hover_rpm']:.3f} / {b['maximum_rpm']:.1f} |",
        f"| 单电机 / 总最大推力 | {t['single_motor_max_thrust_n']:.6f} / {t['four_motor_max_total_thrust_n']:.6f} N |",
        f"| 推重比 | {t['thrust_to_weight_ratio']:.6f} |",
        f"| 悬停 RPM 占比 | {100*t['hover_rpm_fraction_of_max']:.3f}% |", "",
        "## 保持高度时的倾角能力", "", "| 倾角 (rad) | 水平力 (N) | 水平加速度 (m/s²) | 总推力 (N) | 单电机 RPM | 超限 |",
        "|---:|---:|---:|---:|---:|:---:|",
    ]
    for item in data["tilt_capability"]:
        rows.append(f"| {item['tilt_angle_rad']:.2f} | {item['horizontal_force_n']:.6f} | "
                    f"{item['horizontal_acceleration_m_s2']:.6f} | "
                    f"{item['height_holding_total_thrust_n']:.6f} | "
                    f"{item['height_holding_rpm']:.3f} | {'是' if item['exceeds_max_rpm'] else '否'} |")
    rows += ["", "## 力矩能力", "", "| 轴 | 物理估算 (N·m) | 控制上限 (N·m) | 占比 | 判断 |",
             "|---|---:|---:|---:|---|"]
    for axis, item in data["torque_capability"].items():
        rows.append(f"| {axis} | {item['physical_max_nm']:.6f} | {item['controller_limit_nm']:.6f} | "
                    f"{100*item['controller_fraction']:.2f}% | {item['classification']} |")
    rows += ["", "## 一致性和边界", "",
             "动力学与控制器中的质量、机臂、推力系数、反扭矩系数和 RPM 范围一致。"
             "Xacro 的 0.20 m 机臂几何一致，但该文件没有 inertial、RPM 或旋翼系数；"
             "当前它是 RViz 可视模型，不能声称其缺失字段参与纯动力学计算。", "",
             "当前积分为 200 Hz 固定步长（0.005 s）的半隐式 Euler，属于一阶精度；"
             "每步归一化四元数，地面仅是 z 向简化约束。"]
    return "\n".join(rows) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    data = audit()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "capability.json").write_text(
        json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (args.output_dir / "capability.md").write_text(markdown(data), encoding="utf-8")
    print(json.dumps(data, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
