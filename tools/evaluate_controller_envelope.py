#!/usr/bin/env python3
"""Run isolated ROS 2 controller-envelope experiments with temporary YAML files."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import yaml


ROOT = Path(__file__).resolve().parents[1]
DYNAMICS = ROOT / "src/drone_bringup/config/dynamics.yaml"
CONTROLLER = ROOT / "src/drone_bringup/config/controller.yaml"
DEFAULT_OUTPUT = ROOT / "results/vehicle_model_upgrade/controller_envelope_sweep"
CANDIDATES = (
    (0.4, 0.08, "old_conservative"),
    (0.6, 0.12, "a0p6_t0p12"),
    (0.8, 0.15, "a0p8_t0p15"),
    (1.0, 0.15, "a1p0_t0p15"),
    (1.0, 0.18, "balanced"),
    (1.2, 0.18, "a1p2_t0p18"),
    (1.2, 0.20, "a1p2_t0p20"),
    (1.5, 0.20, "aggressive_not_selected"),
)


def load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def write_candidate(directory: Path, acceleration: float, tilt: float,
                    linear_xy: float = 0.20, linear_z: float = 0.10) -> tuple[Path, Path]:
    dynamics = load_yaml(DYNAMICS)
    controller = load_yaml(CONTROLLER)
    dp = dynamics["quadrotor_dynamics_node"]["ros__parameters"]
    cp = controller["position_controller_node"]["ros__parameters"]
    dp.update({"enable_aerodynamic_drag": True, "linear_drag_x": linear_xy,
               "linear_drag_y": linear_xy, "linear_drag_z": linear_z,
               "quadratic_drag_x": 0.0, "quadratic_drag_y": 0.0,
               "quadratic_drag_z": 0.0, "angular_damping_roll": 0.010,
               "angular_damping_pitch": 0.010, "angular_damping_yaw": 0.020})
    cp["max_horizontal_acceleration"] = acceleration
    cp["max_tilt_angle"] = tilt
    dynamics_path, controller_path = directory / "dynamics.yaml", directory / "controller.yaml"
    dynamics_path.write_text(yaml.safe_dump(dynamics, sort_keys=False), encoding="utf-8")
    controller_path.write_text(yaml.safe_dump(controller, sort_keys=False), encoding="utf-8")
    return dynamics_path, controller_path


def stop_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        process.terminate(); process.wait(timeout=3)


def quaternion_to_euler(q) -> tuple[float, float, float]:
    x, y, z, w = q
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1.0, min(1.0, 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def rotate(q, v) -> tuple[float, float, float]:
    x, y, z, w = q; vx, vy, vz = v
    norm = math.sqrt(x*x+y*y+z*z+w*w); x,y,z,w = x/norm,y/norm,z/norm,w/norm
    tx,ty,tz = 2*(y*vz-z*vy),2*(z*vx-x*vz),2*(x*vy-y*vx)
    return (vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx)


def worker(spec_path: Path) -> int:
    spec = json.loads(spec_path.read_text())
    # Import after the parent-set ROS_DOMAIN_ID is visible.
    import rclpy
    from drone_msgs.msg import ControllerDiagnostics, MotorRPM
    from geometry_msgs.msg import PoseStamped, WrenchStamped
    from nav_msgs.msg import Odometry

    output = Path(spec["output"]); output.mkdir(parents=True, exist_ok=True)
    kind = spec["kind"]
    launch = "disturbance_hover_sim.launch.py" if kind.startswith("disturbance") else "basic_sim.launch.py"
    log_stream = (output / "launch.log").open("w", encoding="utf-8")
    process = subprocess.Popen([
        "ros2", "launch", "drone_bringup", launch, "use_rviz:=false",
        f"dynamics_config:={spec['dynamics']}", f"controller_config:={spec['controller']}"],
        stdout=log_stream, stderr=subprocess.STDOUT, start_new_session=True, text=True,
        env=os.environ.copy())
    rclpy.init(); node = rclpy.create_node("controller_envelope_worker")
    goal_pub = node.create_publisher(PoseStamped, "/drone/goal", 10)
    wrench_pub = node.create_publisher(WrenchStamped, "/drone/external_wrench", 10)
    latest = {}; latest_diag = None; rpm = (0.0,)*4; records = []; diag_records = []

    def on_rpm(message):
        nonlocal rpm
        rpm = (message.m1_front_left_ccw_rpm, message.m2_rear_left_cw_rpm,
               message.m3_rear_right_ccw_rpm, message.m4_front_right_cw_rpm)

    def on_diag(message):
        nonlocal latest_diag
        latest_diag = message
        if latest.get("phase") == "MEASURE":
            diag_records.append({"a": math.hypot(message.horizontal_acceleration_x,
                                                  message.horizontal_acceleration_y),
                "horizontal": message.horizontal_saturated, "altitude": message.altitude_saturated,
                "attitude": message.attitude_saturated, "mixer": message.mixer_saturated,
                "pre_rpm": list(message.unclipped_motor_rpm), "rpm": list(message.motor_rpm)})

    def on_odom(message):
        pose = message.pose.pose; q=(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
        velocity = rotate(q, (message.twist.twist.linear.x,message.twist.twist.linear.y,
                              message.twist.twist.linear.z))
        roll,pitch,yaw = quaternion_to_euler(q)
        latest.update(position=(pose.position.x,pose.position.y,pose.position.z), velocity=velocity,
                      angles=(roll,pitch,yaw), rpm=rpm)
        if latest.get("phase") == "MEASURE":
            records.append({"t": time.monotonic()-latest["measure_start"],
                            "position": list(latest["position"]), "velocity": list(velocity),
                            "angles": [roll,pitch,yaw], "rpm": list(rpm)})

    node.create_subscription(Odometry,"/drone/odom",on_odom,10)
    node.create_subscription(MotorRPM,"/drone/motor_rpm_cmd",on_rpm,10)
    node.create_subscription(ControllerDiagnostics,"/drone/controller/diagnostics",on_diag,10)

    def publish_goal(target):
        message=PoseStamped(); message.header.frame_id="map"; message.pose.position.x=target[0]
        message.pose.position.y=target[1]; message.pose.position.z=target[2]; message.pose.orientation.w=1.0
        goal_pub.publish(message)

    try:
        deadline=time.monotonic()+25; stable_since=None
        while time.monotonic()<deadline:
            publish_goal((0.0,0.0,1.5)); rclpy.spin_once(node,timeout_sec=0.02)
            if "position" in latest:
                error=math.dist(latest["position"],(0.0,0.0,1.5)); speed=math.sqrt(sum(v*v for v in latest["velocity"]))
                stable_since = time.monotonic() if error<0.03 and speed<0.03 and stable_since is None else stable_since
                if error>=0.03 or speed>=0.03: stable_since=None
                if stable_since and time.monotonic()-stable_since>=1.0: break
        if not stable_since: raise RuntimeError("takeoff did not stabilize")
        target=(2.0,1.0,1.5) if kind=="diagonal_step" else (2.0,0.0,1.5)
        latest["phase"]="MEASURE"; latest["measure_start"]=time.monotonic(); stable_since=None
        disturbance_start = latest["measure_start"] + 1.0
        disturbance_duration = 10.0 if kind=="disturbance_persistent" else 2.0
        deadline=time.monotonic()+spec.get("timeout",30.0)
        while time.monotonic()<deadline:
            publish_goal((0.0,0.0,1.5) if kind.startswith("disturbance") else target)
            if kind.startswith("disturbance"):
                wrench=WrenchStamped(); wrench.header.frame_id="map"
                now=time.monotonic()
                if disturbance_start <= now < disturbance_start+disturbance_duration:
                    wrench.wrench.force.x=float(spec["force"])
                wrench_pub.publish(wrench)
            rclpy.spin_once(node,timeout_sec=0.01)
            if "position" not in latest: continue
            desired=(0.0,0.0,1.5) if kind.startswith("disturbance") else target
            error=math.dist(latest["position"],desired); speed=math.sqrt(sum(v*v for v in latest["velocity"]))
            disturbance_done = not kind.startswith("disturbance") or time.monotonic() >= disturbance_start+disturbance_duration
            stable_since = time.monotonic() if disturbance_done and error<0.03 and speed<0.03 and stable_since is None else stable_since
            if error>=0.03 or speed>=0.03: stable_since=None
            if stable_since and time.monotonic()-stable_since>=2.0: break
        if not records: raise RuntimeError("no measurement samples")
        direction=(target[0],target[1]); length=math.hypot(*direction); unit=(direction[0]/length,direction[1]/length)
        projections=[r["position"][0]*unit[0]+r["position"][1]*unit[1] for r in records]
        rise10=next((r["t"] for r,p in zip(records,projections) if p>=0.1*length),None)
        rise90=next((r["t"] for r,p in zip(records,projections) if p>=0.9*length),None)
        maximum_offset=max(math.hypot(r["position"][0],r["position"][1]) for r in records)
        metrics={"kind":kind,"completed":stable_since is not None,"rise_time_s":None if rise10 is None or rise90 is None else rise90-rise10,
            "overshoot_m":max(0.0,max(projections)-length) if not kind.startswith("disturbance") else None,
            "settling_time_s":None if stable_since is None else stable_since-latest["measure_start"],
            "maximum_horizontal_offset_m":maximum_offset,
            "final_error_m":math.dist(records[-1]["position"], (0.0,0.0,1.5) if kind.startswith("disturbance") else target),
            "final_speed_m_s":math.sqrt(sum(v*v for v in records[-1]["velocity"])),
            "maximum_tilt_rad":max(math.hypot(r["angles"][0],r["angles"][1]) for r in records),
            "maximum_rpm":max(max(r["rpm"]) for r in records),
            "maximum_horizontal_acceleration_request_m_s2":max((d["a"] for d in diag_records),default=0.0),
            "saturation_counts":{name:sum(bool(d[name]) for d in diag_records) for name in ("horizontal","altitude","attitude","mixer")},
            "diagnostic_samples":len(diag_records),"state_samples":len(records)}
        (output/"metrics.json").write_text(json.dumps(metrics,indent=2)+"\n")
        return 0
    except Exception as error:
        (output/"metrics.json").write_text(json.dumps({"kind":kind,"error":str(error)},indent=2)+"\n")
        return 1
    finally:
        node.destroy_node(); rclpy.shutdown(); stop_process(process); log_stream.close()


def parent(arguments) -> int:
    arguments.output.mkdir(parents=True,exist_ok=True); results=[]; domain=arguments.domain_start
    selected = CANDIDATES if not arguments.quick else (CANDIDATES[0],CANDIDATES[2],CANDIDATES[-1])
    with tempfile.TemporaryDirectory(prefix="controller-envelope-") as temporary:
        temporary=Path(temporary)
        for acceleration,tilt,name in selected:
            if 9.80665*math.tan(tilt) <= acceleration:
                raise RuntimeError(f"candidate {name} has no tilt margin")
            candidate_dir=temporary/name; candidate_dir.mkdir()
            drag = 0.0 if name == "old_conservative" else 0.01
            dynamics,controller=write_candidate(candidate_dir,acceleration,tilt,drag,drag)
            kinds=["horizontal_step","diagonal_step"]
            if name in ("old_conservative","a0p8_t0p15","balanced","aggressive_not_selected"):
                kinds += ["disturbance_0p3","disturbance_0p8","disturbance_persistent"]
            for kind in kinds:
                run_output=arguments.output/name/kind; run_output.mkdir(parents=True,exist_ok=True)
                metrics_path = run_output / "metrics.json"
                if arguments.resume and metrics_path.exists():
                    metrics=json.loads(metrics_path.read_text()); metrics.update(candidate=name,
                        max_horizontal_acceleration=acceleration,max_tilt_angle=tilt,ros_domain_id=None)
                    results.append(metrics); print(name,kind,"reused",flush=True); continue
                spec={"kind":kind,"dynamics":str(dynamics),"controller":str(controller),"output":str(run_output),
                      "force":0.3 if kind in ("disturbance_0p3","disturbance_persistent") else 0.8,"timeout":arguments.timeout}
                spec_path=run_output/"spec.json"; spec_path.write_text(json.dumps(spec))
                env=os.environ.copy(); env["ROS_DOMAIN_ID"]=str(domain); domain = domain+1 if domain<232 else 120
                completed=subprocess.run([sys.executable,__file__,"--worker",str(spec_path)],env=env,check=False)
                metrics=json.loads(metrics_path.read_text()); metrics.update(candidate=name,
                    max_horizontal_acceleration=acceleration,max_tilt_angle=tilt,ros_domain_id=int(env["ROS_DOMAIN_ID"]))
                results.append(metrics)
                print(name,kind,"ok" if completed.returncode==0 else metrics.get("error","failed"),flush=True)
    (arguments.output/"sweep.json").write_text(json.dumps({"candidates":results},indent=2)+"\n")
    return 0 if all("error" not in item for item in results) else 1


def main() -> int:
    parser=argparse.ArgumentParser(); parser.add_argument("--output",type=Path,default=DEFAULT_OUTPUT)
    parser.add_argument("--domain-start",type=int,default=120); parser.add_argument("--timeout",type=float,default=35.0)
    parser.add_argument("--quick",action="store_true"); parser.add_argument("--resume",action="store_true")
    parser.add_argument("--worker",type=Path)
    args=parser.parse_args(); return worker(args.worker) if args.worker else parent(args)


if __name__=="__main__": raise SystemExit(main())
