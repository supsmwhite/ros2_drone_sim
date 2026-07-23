#!/usr/bin/env python3
"""Read-only terminal dashboard for every ROS 2 quadrotor assessment mode."""

import argparse
import math
import os
from pathlib import Path
import re
import sys
import time

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools"))

from live_monitor_metrics import (  # noqa: E402
    OnlineExtrema, format_duration, obstacle_boxes, quaternion_yaw, safety_clearance,
    vector_error, wrapped_error)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Display read-only live metrics for all assessment scenarios.")
    parser.add_argument(
        "--mode", choices=("auto", "basic", "navigation", "disturbance"),
        default="auto", help="force a display mode instead of topic-based detection")
    parser.add_argument(
        "--rate", type=float, default=2.0, help="dashboard refresh rate in Hz (default: 2)")
    parser.add_argument(
        "--domain-id", type=int, help="set ROS_DOMAIN_ID before connecting")
    parser.add_argument(
        "--environment-config", type=Path,
        default=REPO_ROOT / "src/drone_bringup/config/environment.yaml",
        help="environment YAML used to calculate obstacle clearance")
    parser.add_argument(
        "--no-clear", action="store_true", help="append frames instead of clearing the terminal")
    args = parser.parse_args()
    if not math.isfinite(args.rate) or not 0.2 <= args.rate <= 20.0:
        parser.error("--rate must be within 0.2..20 Hz")
    if args.domain_id is not None and not 0 <= args.domain_id <= 232:
        parser.error("--domain-id must be within 0..232")
    return args


def load_environment(path):
    data = yaml.safe_load(path.read_text())
    parameters = next(iter(data.values()))["ros__parameters"]
    return obstacle_boxes(parameters.get("obstacles", [])), float(parameters["safety_radius"])


def pose_values(pose):
    position = (pose.position.x, pose.position.y, pose.position.z)
    orientation = pose.orientation
    yaw = quaternion_yaw((orientation.x, orientation.y, orientation.z, orientation.w))
    return position, yaw


def finite_vector(values):
    return values is not None and all(math.isfinite(value) for value in values)


def vector_text(values, unit="m"):
    if not finite_vector(values):
        return "--"
    return f"({values[0]:7.3f}, {values[1]:7.3f}, {values[2]:7.3f}) {unit}"


def value_text(value, unit="", digits=3):
    if value is None or not math.isfinite(value):
        return "--"
    suffix = f" {unit}" if unit else ""
    return f"{value:.{digits}f}{suffix}"


def boolean_text(value):
    return "--" if value is None else ("YES" if value else "no")


class AssessmentMonitor:
    def __init__(self, node, args, message_types, qos_types):
        self.node = node
        self.args = args
        self.types = message_types
        self.subscriptions = []
        self.position = self.velocity = None
        self.actual_yaw = None
        self.goal = self.goal_yaw = None
        self.reference = self.reference_yaw = None
        self.basic_goals = []
        self.basic_index = None
        self.basic_complete = None
        self.navigation_index = self.visited = self.goal_count = None
        self.navigation_complete = self.navigation_success = None
        self.navigation_status = ""
        self.collision = None
        self.force = (0.0, 0.0, 0.0)
        self.force_active = None
        self.force_started_at = self.force_released_at = None
        self.integral = None
        self.saturation = None
        self.disturbance_status = ""
        self.saw_basic = self.saw_navigation = self.saw_disturbance = False
        self.odom_received_at = None
        self.monitor_started_at = time.monotonic()
        self.mission_started_at = None
        self.mission_signature = None
        self.mission_active = False
        self.metrics = OnlineExtrema()
        self.summaries = []
        self.last_summary = None
        self.frame_count = 0
        self.boxes, self.safety_radius = load_environment(args.environment_config)

        transient = qos_types["QoSProfile"](
            depth=1,
            reliability=qos_types["ReliabilityPolicy"].RELIABLE,
            durability=qos_types["DurabilityPolicy"].TRANSIENT_LOCAL)
        self.subscribe(message_types["Odometry"], "/drone/odom", self.on_odometry, 50)
        self.subscribe(message_types["PoseStamped"], "/drone/goal", self.on_goal, 20)
        self.subscribe(message_types["PoseArray"], "/drone/mission/goals", self.on_basic_goals, transient)
        self.subscribe(message_types["UInt32"], "/drone/mission/current_waypoint_index", self.on_basic_index, transient)
        self.subscribe(message_types["Bool"], "/drone/mission/complete", self.on_basic_complete, transient)
        self.subscribe(message_types["TrajectorySetpoint"], "/drone/trajectory_setpoint", self.on_reference, 20)
        self.subscribe(message_types["PoseArray"], "/drone/interactive_goals/selected_goals", self.on_navigation_goals, transient)
        self.subscribe(message_types["PoseStamped"], "/drone/multi_goal/current_goal_pose", self.on_navigation_goal, transient)
        self.subscribe(message_types["UInt32"], "/drone/multi_goal/current_goal_index", self.on_navigation_index, 20)
        self.subscribe(message_types["UInt32"], "/drone/multi_goal/visited_goals", self.on_visited, 20)
        self.subscribe(message_types["Bool"], "/drone/multi_goal/complete", self.on_navigation_complete, 20)
        self.subscribe(message_types["Bool"], "/drone/multi_goal/success", self.on_navigation_success, 20)
        self.subscribe(message_types["String"], "/drone/interactive_mission/status", self.on_navigation_status, transient)
        self.subscribe(message_types["Bool"], "/drone/interactive_mission/active", self.on_navigation_active, transient)
        self.subscribe(message_types["Bool"], "/drone/environment/in_collision", self.on_collision, 20)
        self.subscribe(message_types["Bool"], "/drone/external_wrench/active", self.on_force_active, transient)
        self.subscribe(message_types["WrenchStamped"], "/drone/external_wrench/applied", self.on_force, transient)
        self.subscribe(message_types["ControllerDiagnostics"], "/drone/controller/diagnostics", self.on_diagnostics, 20)
        self.subscribe(message_types["MarkerArray"], "/drone/disturbance/markers", self.on_disturbance_markers, transient)
        self.timer = node.create_timer(1.0 / args.rate, self.render)

    def subscribe(self, message_type, topic, callback, qos):
        self.subscriptions.append(self.node.create_subscription(message_type, topic, callback, qos))

    def start_mission(self, signature=None, force=False):
        signature_changed = signature is not None and signature != self.mission_signature
        if self.mission_active and not force and not signature_changed:
            return
        if not self.mission_active and not force and signature is not None and not signature_changed:
            return
        if self.mission_active:
            self.finish_mission("SUPERSEDED BY NEW TASK")
        self.mission_started_at = time.monotonic()
        self.mission_active = True
        self.metrics = OnlineExtrema()
        if signature is not None:
            self.mission_signature = signature

    def observe_metrics(self):
        if not self.mission_active:
            return
        mode = self.detected_mode()
        goal_error = vector_error(self.position, self.goal)
        tracking_error = vector_error(self.position, self.reference)
        yaw_error = wrapped_error(self.goal_yaw, self.actual_yaw)
        speed = None if self.velocity is None else math.sqrt(
            sum(value * value for value in self.velocity))
        clearance = (
            safety_clearance(self.position, self.boxes, self.safety_radius)
            if mode == "navigation" else None)
        force_magnitude = math.sqrt(sum(value * value for value in self.force))
        self.metrics.observe(
            goal_error=None if goal_error is None else goal_error["distance"],
            horizontal_error=None if goal_error is None else goal_error["horizontal_distance"],
            tracking_error=None if tracking_error is None else tracking_error["distance"],
            yaw_error=yaw_error,
            speed=speed,
            safety_clearance_m=None if clearance is None else clearance[1],
            force_magnitude=force_magnitude,
            saturation=self.saturation)

    def finish_mission(self, reason):
        if not self.mission_active:
            return
        self.observe_metrics()
        now = time.monotonic()
        duration = max(0.0, now - self.mission_started_at)
        goal_error = vector_error(self.position, self.goal)
        tracking_error = vector_error(self.position, self.reference)
        summary = {
            "mode": self.detected_mode(),
            "reason": reason,
            "duration": duration,
            "position": self.position,
            "goal": self.goal,
            "final_goal_error": None if goal_error is None else goal_error["distance"],
            "final_horizontal_error": (
                None if goal_error is None else goal_error["horizontal_distance"]),
            "final_tracking_error": (
                None if tracking_error is None else tracking_error["distance"]),
            "final_yaw_error": wrapped_error(self.goal_yaw, self.actual_yaw),
            "metrics": self.metrics.snapshot(),
        }
        self.summaries.append(summary)
        self.last_summary = summary
        self.mission_active = False
        self.mission_started_at = None

    def on_odometry(self, message):
        self.position, self.actual_yaw = pose_values(message.pose.pose)
        velocity = message.twist.twist.linear
        self.velocity = (velocity.x, velocity.y, velocity.z)
        self.odom_received_at = time.monotonic()
        self.observe_metrics()

    def on_goal(self, message):
        position, yaw = pose_values(message.pose)
        self.saw_basic = True
        self.goal, self.goal_yaw = position, yaw
        if not self.basic_goals and not self.saw_navigation:
            self.start_mission(("single",) + tuple(round(value, 9) for value in position))

    def on_basic_goals(self, message):
        goals = [pose_values(pose) for pose in message.poses]
        if not goals:
            return
        self.saw_basic = True
        self.basic_goals = goals
        signature = ("basic",) + tuple(
            round(value, 9) for position, yaw in goals for value in (*position, yaw or 0.0))
        if (self.mission_active and self.mission_signature and
                self.mission_signature[0] == "single"):
            self.mission_signature = signature
        else:
            self.start_mission(signature)
        index = 0 if self.basic_index is None else min(self.basic_index, len(goals) - 1)
        self.goal, self.goal_yaw = goals[index]

    def on_basic_index(self, message):
        self.saw_basic = True
        self.basic_index = int(message.data)
        if self.basic_goals and self.basic_index < len(self.basic_goals):
            self.goal, self.goal_yaw = self.basic_goals[self.basic_index]

    def on_basic_complete(self, message):
        self.saw_basic = True
        previous = self.basic_complete
        self.basic_complete = bool(message.data)
        if self.basic_complete:
            self.finish_mission("MISSION COMPLETE")
        elif previous is True:
            self.start_mission(self.mission_signature, force=True)

    def on_reference(self, message):
        self.reference = (message.position.x, message.position.y, message.position.z)
        self.reference_yaw = float(message.yaw)

    def on_navigation_goals(self, message):
        goals = [pose_values(pose) for pose in message.poses]
        if goals:
            self.saw_navigation = True
            self.goal_count = len(goals)

    def on_navigation_goal(self, message):
        self.saw_navigation = True
        self.goal, self.goal_yaw = pose_values(message.pose)

    def on_navigation_index(self, message):
        self.saw_navigation = True
        self.navigation_index = int(message.data)

    def on_visited(self, message):
        self.saw_navigation = True
        self.visited = int(message.data)

    def on_navigation_complete(self, message):
        self.saw_navigation = True
        self.navigation_complete = bool(message.data)
        if self.navigation_complete:
            result = "MISSION COMPLETE" if self.navigation_success is not False else "MISSION FAILED"
            self.finish_mission(result)

    def on_navigation_success(self, message):
        self.saw_navigation = True
        self.navigation_success = bool(message.data)

    def on_navigation_active(self, message):
        self.saw_navigation = True
        if bool(message.data):
            self.start_mission()

    def on_navigation_status(self, message):
        self.saw_navigation = True
        self.navigation_status = message.data
        match = re.search(r"P(\d+)\s*/\s*(\d+)", message.data)
        if match:
            self.navigation_index = int(match.group(1)) - 1
            self.goal_count = int(match.group(2))
        upper = message.data.upper()
        if any(label in upper for label in ("PLANNING", "EXECUTING", "HOLDING")):
            self.start_mission()
        elif "MISSION COMPLETE" in upper:
            self.finish_mission("MISSION COMPLETE")
        elif any(label in upper for label in ("MISSION FAILED", "REJECTED", "INVALID")):
            self.finish_mission(message.data)

    def on_collision(self, message):
        self.collision = bool(message.data)

    def on_force_active(self, message):
        now = time.monotonic()
        active = bool(message.data)
        if active and self.force_active is not True:
            self.force_started_at = now
            self.force_released_at = None
        elif not active and self.force_active is True:
            self.force_released_at = now
        self.force_active = active

    def on_force(self, message):
        force = message.wrench.force
        self.force = (force.x, force.y, force.z)

    def on_diagnostics(self, message):
        self.integral = (
            message.horizontal_i_acceleration_x,
            message.horizontal_i_acceleration_y)
        self.saturation = (
            bool(message.horizontal_saturated), bool(message.altitude_saturated),
            bool(message.attitude_saturated), bool(message.mixer_saturated))

    def on_disturbance_markers(self, message):
        self.saw_disturbance = True
        for marker in message.markers:
            if marker.ns == "disturbance_status" and marker.text:
                self.disturbance_status = marker.text.replace("\n", " | ")
        if self.disturbance_status.upper().startswith("COMPLETE"):
            self.finish_mission("DISTURBANCE COMPLETE")
        else:
            self.start_mission()

    def detected_mode(self):
        if self.args.mode != "auto":
            return self.args.mode
        if self.saw_disturbance:
            return "disturbance"
        if self.saw_navigation:
            return "navigation"
        return "basic"

    def mission_state(self, mode):
        if mode == "disturbance":
            return self.disturbance_status or "waiting for disturbance status"
        if mode == "navigation":
            return self.navigation_status or "waiting for navigation mission"
        if self.basic_complete:
            return "MISSION COMPLETE"
        if self.basic_index is not None and self.basic_goals:
            return f"EXECUTING P{self.basic_index + 1} / {len(self.basic_goals)}"
        return "waiting for basic mission"

    def render(self):
        now = time.monotonic()
        self.frame_count += 1
        mode = self.detected_mode()
        goal_error = vector_error(self.position, self.goal)
        tracking_error = vector_error(self.position, self.reference)
        yaw_error = wrapped_error(self.goal_yaw, self.actual_yaw)
        speed = None if self.velocity is None else math.sqrt(sum(value * value for value in self.velocity))
        clearance = (
            safety_clearance(self.position, self.boxes, self.safety_radius)
            if mode == "navigation" else None)
        odom_age = None if self.odom_received_at is None else now - self.odom_received_at
        mission_time = (
            now - self.mission_started_at
            if self.mission_active and self.mission_started_at is not None else 0.0)
        force_magnitude = math.sqrt(sum(value * value for value in self.force))
        domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
        progress_index = self.navigation_index if mode == "navigation" else self.basic_index
        progress_total = self.goal_count if mode == "navigation" else len(self.basic_goals) or None
        progress_current = (
            "--" if progress_index is None or progress_total is None else progress_index + 1)
        displayed_summary = None if self.mission_active else self.last_summary
        extrema = (
            self.metrics.snapshot() if self.mission_active else
            displayed_summary["metrics"] if displayed_summary else OnlineExtrema().snapshot())
        summary_label = "本任务在线累计" if self.mission_active else "上一任务最终汇总"

        lines = [
            f"ROS 2 Quadrotor Assessment Monitor | mode={mode.upper()} | domain={domain_id}",
            "=" * 78,
            f"刷新心跳       : {time.strftime('%H:%M:%S')}  frame={self.frame_count}",
            f"连接状态       : {'WAITING FOR /drone/odom' if odom_age is None else ('LIVE' if odom_age < 1.0 else 'STALE')}"
            + ("" if odom_age is None else f"  (age={odom_age:.3f} s)"),
            f"任务计时       : {format_duration(mission_time)}   活动状态: {'RUNNING' if self.mission_active else 'IDLE'}",
            f"监控运行       : {format_duration(now - self.monitor_started_at)}"
            + ("" if displayed_summary is None else
               f"   上次任务: {format_duration(displayed_summary['duration'])}"),
            f"任务状态       : {self.mission_state(mode)}",
            f"目标进度       : {progress_current} / {progress_total or '--'}"
            + (f"   visited={self.visited}" if mode == "navigation" and self.visited is not None else ""),
            "-" * 78,
            f"当前位置       : {vector_text(self.position)}",
            f"当前目标       : {vector_text(self.goal)}",
            f"目标误差向量   : {vector_text(None if goal_error is None else goal_error['vector'])}",
            f"目标距离       : {value_text(None if goal_error is None else goal_error['distance'], 'm')}",
            f"水平目标误差   : {value_text(None if goal_error is None else goal_error['horizontal_distance'], 'm')}",
            f"当前速度       : {value_text(speed, 'm/s')}",
            f"当前/目标 yaw  : {value_text(self.actual_yaw, 'rad')} / {value_text(self.goal_yaw, 'rad')}",
            f"终端 yaw 误差  : {value_text(yaw_error, 'rad')}",
            "-" * 78,
            f"轨迹参考点     : {vector_text(self.reference)}",
            f"轨迹跟踪误差   : {value_text(None if tracking_error is None else tracking_error['distance'], 'm')}",
            f"障碍原始距离   : {value_text(None if clearance is None else clearance[0], 'm')}",
            f"安全净空       : {value_text(None if clearance is None else clearance[1], 'm')}",
            f"碰撞状态       : {boolean_text(self.collision)}",
            f"控制饱和 H/Z/ATT/MIX: {('--' if self.saturation is None else '/'.join('1' if flag else '0' for flag in self.saturation))}",
            "-" * 78,
            f"{summary_label}:",
            f"  最大目标距离 : {value_text(extrema['maximum_goal_error'], 'm')}"
            f"   最大水平误差: {value_text(extrema['maximum_horizontal_error'], 'm')}",
            f"  最大跟踪误差 : {value_text(extrema['maximum_tracking_error'], 'm')}"
            f"   最大 yaw误差: {value_text(extrema['maximum_absolute_yaw_error'], 'rad')}",
            f"  最大速度     : {value_text(extrema['maximum_speed'], 'm/s')}"
            f"   最小安全净空: {value_text(extrema['minimum_safety_clearance'], 'm')}",
            "  曾发生饱和   : " + "/".join(
                "1" if flag else "0" for flag in extrema["saturation_observed"]),
        ]
        if mode == "disturbance":
            force_phase_time = None
            force_phase = "waiting"
            if self.force_active:
                force_phase = "ACTIVE"
                force_phase_time = now - self.force_started_at if self.force_started_at else None
            elif self.force_released_at is not None:
                force_phase = "RECOVERY"
                force_phase_time = now - self.force_released_at
            lines.extend([
                "-" * 78,
                f"外力状态       : {force_phase}  phase_time={format_duration(force_phase_time)}",
                f"外力向量/大小  : {vector_text(self.force, 'N')} / {value_text(force_magnitude, 'N')}",
                f"积分补偿 I_xy  : {('--' if self.integral is None else f'({self.integral[0]:.3f}, {self.integral[1]:.3f}) m/s^2')}",
                f"峰值外力       : {value_text(extrema['maximum_force'], 'N')}",
            ])
        lines.append("Ctrl-C 退出；本程序只订阅 Topic，不发送目标或控制命令。")
        prefix = "" if self.args.no_clear or not sys.stdout.isatty() else "\033[2J\033[H"
        print(prefix + "\n".join(lines), flush=True)

    def print_session_summary(self):
        print("\033[0m\nAssessment monitor session summary", flush=True)
        print("=" * 78, flush=True)
        if not self.summaries:
            print("No task was observed during this monitor session.", flush=True)
            return
        for index, summary in enumerate(self.summaries, start=1):
            metrics = summary["metrics"]
            saturation = "/".join(
                "1" if flag else "0" for flag in metrics["saturation_observed"])
            print(
                f"Task {index}: mode={summary['mode']} result={summary['reason']} "
                f"duration={format_duration(summary['duration'])}\n"
                f"  final position      : {vector_text(summary['position'])}\n"
                f"  final goal          : {vector_text(summary['goal'])}\n"
                f"  final/max goal error: {value_text(summary['final_goal_error'], 'm')} / "
                f"{value_text(metrics['maximum_goal_error'], 'm')}\n"
                f"  final/max track err : {value_text(summary['final_tracking_error'], 'm')} / "
                f"{value_text(metrics['maximum_tracking_error'], 'm')}\n"
                f"  final/max yaw error : {value_text(summary['final_yaw_error'], 'rad')} / "
                f"{value_text(metrics['maximum_absolute_yaw_error'], 'rad')}\n"
                f"  max speed           : {value_text(metrics['maximum_speed'], 'm/s')}\n"
                f"  min safety clearance: {value_text(metrics['minimum_safety_clearance'], 'm')}\n"
                f"  peak force          : {value_text(metrics['maximum_force'], 'N')}\n"
                f"  saturation H/Z/ATT/MIX ever: {saturation}",
                flush=True)


def main():
    args = parse_arguments()
    if args.domain_id is not None:
        os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from drone_msgs.msg import ControllerDiagnostics, TrajectorySetpoint
    from geometry_msgs.msg import PoseArray, PoseStamped, WrenchStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String, UInt32
    from visualization_msgs.msg import MarkerArray

    message_types = {
        "Bool": Bool, "ControllerDiagnostics": ControllerDiagnostics,
        "MarkerArray": MarkerArray, "Odometry": Odometry, "PoseArray": PoseArray,
        "PoseStamped": PoseStamped, "String": String,
        "TrajectorySetpoint": TrajectorySetpoint, "UInt32": UInt32,
        "WrenchStamped": WrenchStamped,
    }
    qos_types = {
        "DurabilityPolicy": DurabilityPolicy, "QoSProfile": QoSProfile,
        "ReliabilityPolicy": ReliabilityPolicy,
    }
    rclpy.init()
    node = rclpy.create_node("assessment_live_monitor")
    monitor = AssessmentMonitor(node, args, message_types, qos_types)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        monitor.finish_mission("MONITOR STOPPED")
        monitor.print_session_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
