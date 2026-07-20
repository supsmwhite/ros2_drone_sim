#!/usr/bin/env python3
"""Record one assessment run from an already-running ROS 2 graph.

This process does not launch simulation, publish goals, select navigation targets, or plot.
Odometry is the sampling clock; the most recent values from the other topics are joined to it.
"""

import argparse
import csv
import json
import math
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import yaml

import rclpy
from drone_msgs.msg import ControllerDiagnostics, MotorRPM, TrajectorySetpoint
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, UInt32
from visualization_msgs.msg import MarkerArray


EXPERIMENTS = (
    "hover", "single_goal", "multi_goal", "navigation", "disturbance", "failure_case")
SAMPLE_FIELDS = (
    "time_s", "mission_time_s", "target_x", "target_y", "target_z", "actual_x",
    "actual_y", "actual_z", "error_x", "error_y", "error_z", "position_error",
    "velocity_x", "velocity_y", "velocity_z", "speed", "roll", "pitch", "yaw",
    "angular_speed_x", "angular_speed_y", "angular_speed_z", "m1_rpm", "m2_rpm",
    "m3_rpm", "m4_rpm", "horizontal_saturated", "altitude_saturated",
    "attitude_saturated", "mixer_saturated", "raw_obstacle_distance",
    "safety_clearance", "current_goal_index", "visited_goals", "mission_complete",
    "mission_success", "external_force_x", "external_force_y", "external_force_z",
    "integral_compensation_x", "integral_compensation_y")


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def quaternion_to_euler(q):
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not math.isfinite(norm) or norm < 1.0e-12:
        return math.nan, math.nan, math.nan
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    roll = math.atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0*(w*y - z*x))))
    yaw = math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
    return roll, pitch, yaw


def load_target(path):
    if not path:
        return None
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if isinstance(data, dict) and "target" in data:
        data = data["target"]
    if isinstance(data, dict):
        return [float(data[key]) for key in ("x", "y", "z")]
    if isinstance(data, (list, tuple)) and len(data) >= 3:
        return [float(value) for value in data[:3]]
    raise ValueError("target config must be {x,y,z}, {target:{x,y,z}}, or [x,y,z]")


def load_environment(path):
    if not path:
        return [], None
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    params = next(iter(data.values()))["ros__parameters"]
    values = params.get("obstacles", [])
    boxes = []
    for offset in range(0, len(values), 6):
        cx, cy, cz, sx, sy, sz = map(float, values[offset:offset+6])
        boxes.append(((cx-sx/2, cy-sy/2, cz-sz/2), (cx+sx/2, cy+sy/2, cz+sz/2)))
    return boxes, float(params["safety_radius"])


def point_box_distance(point, box):
    lower, upper = box
    return math.sqrt(sum(max(lower[i]-point[i], 0.0, point[i]-upper[i])**2 for i in range(3)))


def repository_commit():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


class Recorder:
    def __init__(self, node, args):
        self.node = node
        self.args = args
        self.output = Path(args.output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.start_wall = time.monotonic()
        self.start_ros = None
        self.mission_start_ros = None
        self.last_ros = None
        self.target = load_target(args.target_config)
        self.boxes, self.safety_radius = load_environment(args.environment_config)
        self.latest_rpm = None
        self.latest_diagnostics = None
        self.latest_imu = None
        self.latest_force = None
        self.current_goal_index = None
        self.visited_goals = None
        self.mission_complete = None
        self.mission_success = None
        self.arrival_candidate = None
        self.arrival_time = None
        self.finish_after = None
        self.stop = False
        self.stop_reason = None
        self.counts = {}
        self.events = []
        self.paths = {}
        self.last_event_values = {}
        self.log_lines = []
        self.samples_handle = (self.output / "samples.csv").open("w", newline="", encoding="utf-8")
        self.samples = csv.DictWriter(self.samples_handle, fieldnames=SAMPLE_FIELDS)
        self.samples.writeheader()
        self.subscribe()
        if self.target is not None:
            self.event("configured_target", {"position": self.target})

    def subscribe_one(self, message_type, topic, callback, qos=20):
        self.node.create_subscription(message_type, topic, callback, qos)
        self.counts[topic] = 0

    def subscribe(self):
        self.subscribe_one(Odometry, "/drone/odom", self.on_odom, 50)
        self.subscribe_one(Imu, "/drone/imu", self.on_imu, 50)
        self.subscribe_one(MotorRPM, "/drone/motor_rpm_cmd", self.on_rpm)
        self.subscribe_one(ControllerDiagnostics, "/drone/controller/diagnostics", self.on_diag)
        self.subscribe_one(PoseStamped, "/drone/goal", self.on_goal)
        self.subscribe_one(TrajectorySetpoint, "/drone/trajectory_setpoint", self.on_setpoint)
        self.subscribe_one(NavPath, "/drone/path", lambda msg: self.on_path("actual", msg))
        self.subscribe_one(NavPath, "/drone/planned_path", lambda msg: self.on_path("planned", msg))
        self.subscribe_one(NavPath, "/drone/simplified_path", lambda msg: self.on_path("simplified", msg))
        self.subscribe_one(NavPath, "/drone/reference_path", lambda msg: self.on_path("reference", msg))
        self.subscribe_one(MarkerArray, "/drone/environment/markers", self.on_markers)
        self.subscribe_one(Bool, "/drone/environment/in_collision", lambda msg: self.on_state("in_collision", msg.data))
        self.subscribe_one(UInt32, "/drone/multi_goal/current_goal_index", self.on_index)
        self.subscribe_one(UInt32, "/drone/multi_goal/visited_goals", self.on_visited)
        self.subscribe_one(Bool, "/drone/multi_goal/complete", self.on_complete)
        self.subscribe_one(Bool, "/drone/multi_goal/success", self.on_success)
        self.subscribe_one(Bool, "/drone/external_wrench/active", lambda msg: self.on_state("external_wrench_active", msg.data))
        self.subscribe_one(WrenchStamped, "/drone/external_wrench/applied", self.on_force)

    def tick(self, topic):
        self.counts[topic] = self.counts.get(topic, 0) + 1

    def event(self, name, details=None, ros_time=None):
        now = self.last_ros if ros_time is None else ros_time
        self.events.append({
            "time_s": None if self.start_ros is None or now is None else now-self.start_ros,
            "mission_time_s": None if self.mission_start_ros is None or now is None else now-self.mission_start_ros,
            "event": name, "details": details or {}})

    def changed_event(self, name, value):
        if self.last_event_values.get(name) != value:
            self.last_event_values[name] = value
            self.event(name, {"value": value})

    def on_goal(self, msg):
        self.tick("/drone/goal")
        self.target = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        ros_time = stamp_seconds(msg.header.stamp) or self.last_ros
        if self.mission_start_ros is None:
            self.mission_start_ros = ros_time
        self.event("goal_received", {"position": self.target}, ros_time)

    def on_setpoint(self, msg):
        self.tick("/drone/trajectory_setpoint")
        self.target = [msg.position.x, msg.position.y, msg.position.z]
        if self.mission_start_ros is None:
            self.mission_start_ros = stamp_seconds(msg.header.stamp) or self.last_ros
            self.event("trajectory_accepted", {"position": self.target}, self.mission_start_ros)

    def on_imu(self, msg):
        self.tick("/drone/imu")
        self.latest_imu = msg

    def on_rpm(self, msg):
        self.tick("/drone/motor_rpm_cmd")
        self.latest_rpm = msg

    def on_diag(self, msg):
        self.tick("/drone/controller/diagnostics")
        self.latest_diagnostics = msg

    def on_path(self, name, msg):
        self.tick("/drone/" + ("path" if name == "actual" else name + "_path"))
        self.paths[name] = [[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                            for p in msg.poses]

    def on_markers(self, msg):
        self.tick("/drone/environment/markers")

    def on_force(self, msg):
        self.tick("/drone/external_wrench/applied")
        self.latest_force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        self.changed_event("external_force", self.latest_force)

    def on_state(self, name, value):
        topic = {"in_collision": "/drone/environment/in_collision",
                 "external_wrench_active": "/drone/external_wrench/active"}[name]
        self.tick(topic)
        self.changed_event(name, bool(value))

    def on_index(self, msg):
        self.tick("/drone/multi_goal/current_goal_index")
        self.current_goal_index = int(msg.data)
        self.changed_event("current_goal_index", self.current_goal_index)

    def on_visited(self, msg):
        self.tick("/drone/multi_goal/visited_goals")
        self.visited_goals = int(msg.data)
        self.changed_event("visited_goals", self.visited_goals)

    def on_complete(self, msg):
        self.tick("/drone/multi_goal/complete")
        self.mission_complete = bool(msg.data)
        self.changed_event("mission_complete", self.mission_complete)

    def on_success(self, msg):
        self.tick("/drone/multi_goal/success")
        self.mission_success = bool(msg.data)
        self.changed_event("mission_success", self.mission_success)

    def on_odom(self, msg):
        self.tick("/drone/odom")
        ros_time = stamp_seconds(msg.header.stamp)
        if self.start_ros is None:
            self.start_ros = ros_time
            self.event("recording_started", ros_time=ros_time)
        self.last_ros = ros_time
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        roll, pitch, yaw = quaternion_to_euler(msg.pose.pose.orientation)
        if self.latest_imu is not None:
            roll, pitch, yaw = quaternion_to_euler(self.latest_imu.orientation)
            w = self.latest_imu.angular_velocity
        target = self.target
        errors = [target[i]-value for i, value in enumerate((p.x, p.y, p.z))] if target else [None]*3
        error = math.sqrt(sum(value*value for value in errors)) if target else None
        speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
        raw_distance = min((point_box_distance((p.x, p.y, p.z), box) for box in self.boxes), default=None)
        rpm = self.latest_rpm
        diag = self.latest_diagnostics
        row = dict.fromkeys(SAMPLE_FIELDS, "")
        row.update({
            "time_s": ros_time-self.start_ros,
            "mission_time_s": "" if self.mission_start_ros is None else ros_time-self.mission_start_ros,
            "target_x": "" if not target else target[0], "target_y": "" if not target else target[1],
            "target_z": "" if not target else target[2], "actual_x": p.x, "actual_y": p.y,
            "actual_z": p.z, "error_x": "" if error is None else errors[0],
            "error_y": "" if error is None else errors[1], "error_z": "" if error is None else errors[2],
            "position_error": "" if error is None else error, "velocity_x": v.x,
            "velocity_y": v.y, "velocity_z": v.z, "speed": speed, "roll": roll,
            "pitch": pitch, "yaw": yaw, "angular_speed_x": w.x, "angular_speed_y": w.y,
            "angular_speed_z": w.z, "raw_obstacle_distance": "" if raw_distance is None else raw_distance,
            "safety_clearance": "" if raw_distance is None or self.safety_radius is None else raw_distance-self.safety_radius,
            "current_goal_index": "" if self.current_goal_index is None else self.current_goal_index,
            "visited_goals": "" if self.visited_goals is None else self.visited_goals,
            "mission_complete": "" if self.mission_complete is None else int(self.mission_complete),
            "mission_success": "" if self.mission_success is None else int(self.mission_success)})
        if rpm:
            row.update({"m1_rpm": rpm.m1_front_left_ccw_rpm, "m2_rpm": rpm.m2_rear_left_cw_rpm,
                        "m3_rpm": rpm.m3_rear_right_ccw_rpm, "m4_rpm": rpm.m4_front_right_cw_rpm})
        if diag:
            row.update({"horizontal_saturated": int(diag.horizontal_saturated),
                        "altitude_saturated": int(diag.altitude_saturated),
                        "attitude_saturated": int(diag.attitude_saturated),
                        "mixer_saturated": int(diag.mixer_saturated)})
            row.update({"integral_compensation_x": diag.horizontal_i_acceleration_x,
                        "integral_compensation_y": diag.horizontal_i_acceleration_y})
        if self.latest_force:
            row.update({"external_force_x": self.latest_force[0],
                        "external_force_y": self.latest_force[1],
                        "external_force_z": self.latest_force[2]})
        self.samples.writerow(row)
        if self.counts["/drone/odom"] % 100 == 0:
            self.samples_handle.flush()
        if error is not None and self.mission_start_ros is not None:
            eligible = error < self.args.arrival_position_threshold and speed < self.args.arrival_speed_threshold
            if eligible and self.arrival_candidate is None:
                self.arrival_candidate = ros_time
            elif not eligible:
                self.arrival_candidate = None
            if self.arrival_time is None and self.arrival_candidate is not None and ros_time-self.arrival_candidate >= self.args.arrival_hold_time:
                self.arrival_time = self.arrival_candidate-self.mission_start_ros
                self.finish_after = ros_time + self.args.steady_window
                self.event("arrival_confirmed", {"arrival_time_s": self.arrival_time}, ros_time)
            if self.finish_after is not None and ros_time >= self.finish_after:
                self.stop = True
                self.stop_reason = "arrival_and_steady_window_complete"

    def write_outputs(self):
        self.samples_handle.flush()
        self.samples_handle.close()
        ended = datetime.now(timezone.utc).isoformat()
        metadata = {
            "schema_version": 1, "experiment": self.args.experiment, "status": "smoke" if "smoke" in str(self.output) else "recorded",
            "repository_commit": repository_commit(), "generated_at": ended,
            "target_config": self.args.target_config, "environment_config": self.args.environment_config,
            "thresholds": {"steady_window_s": self.args.steady_window,
                           "arrival_position_threshold_m": self.args.arrival_position_threshold,
                           "arrival_speed_threshold_m_s": self.args.arrival_speed_threshold,
                           "arrival_hold_time_s": self.args.arrival_hold_time,
                           "timeout_s": self.args.timeout},
            "target_position": self.target, "safety_radius_m": self.safety_radius,
            "stop_reason": self.stop_reason, "arrival_time_s": self.arrival_time,
            "topic_message_counts": self.counts}
        (self.output / "metadata.json").write_text(json.dumps(metadata, indent=2)+"\n", encoding="utf-8")
        with (self.output / "events.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=("time_s", "mission_time_s", "event", "details"))
            writer.writeheader()
            for item in self.events:
                writer.writerow({**item, "details": json.dumps(item["details"], separators=(",", ":"))})
        (self.output / "paths.json").write_text(json.dumps(self.paths, indent=2)+"\n", encoding="utf-8")
        self.log_lines.append(f"stop_reason={self.stop_reason}")
        self.log_lines.append("topic_message_counts=" + json.dumps(self.counts, sort_keys=True))
        (self.output / "recorder.log").write_text("\n".join(self.log_lines)+"\n", encoding="utf-8")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", choices=EXPERIMENTS, required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--target-config")
    parser.add_argument("--environment-config", default="src/drone_bringup/config/environment.yaml")
    parser.add_argument("--steady-window", type=float, default=3.0)
    parser.add_argument("--arrival-position-threshold", type=float, default=0.10)
    parser.add_argument("--arrival-speed-threshold", type=float, default=0.08)
    parser.add_argument("--arrival-hold-time", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()
    for name in ("steady_window", "arrival_position_threshold", "arrival_speed_threshold", "arrival_hold_time", "timeout"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0.0:
            parser.error("--" + name.replace("_", "-") + " must be finite and positive")
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node("assessment_recorder")
    recorder = Recorder(node, args)
    def request_stop(_signum, _frame):
        recorder.stop = True
        recorder.stop_reason = "signal"
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not recorder.stop:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic()-recorder.start_wall >= args.timeout:
                recorder.stop = True
                recorder.stop_reason = "timeout"
        return 0 if recorder.counts.get("/drone/odom", 0) > 0 else 2
    finally:
        recorder.write_outputs()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
