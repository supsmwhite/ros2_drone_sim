#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import yaml

from drone_msgs.msg import MotorRPM, TrajectorySetpoint
from nav_msgs.msg import Odometry, Path as PathMessage
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt32


DOMAIN_ID = 114
POST_COMPLETE_DURATION = 3.0
CSV_FIELDS = (
    'time', 'mission_elapsed_time',
    'current_goal_index', 'current_segment', 'visited_goals',
    'actual_x', 'actual_y', 'actual_z',
    'reference_x', 'reference_y', 'reference_z',
    'tracking_error', 'actual_speed', 'reference_speed',
    'reference_acceleration', 'clearance',
    'm1_rpm', 'm2_rpm', 'm3_rpm', 'm4_rpm',
    'mission_complete', 'mission_success', 'collision_state',
)


def norm3(values):
    return math.sqrt(sum(value * value for value in values))


def path_length(points):
    return sum(math.dist(start, end) for start, end in zip(points, points[1:]))


def distance_to_box(point, lower, upper):
    return norm3(tuple(
        max(lower[index] - point[index], 0.0, point[index] - upper[index])
        for index in range(3)
    ))


def inside_closed_box(point, lower, upper):
    return all(lower[index] <= point[index] <= upper[index] for index in range(3))


def segment_intersects_closed_box(start, end, lower, upper):
    interval_min = 0.0
    interval_max = 1.0
    for index in range(3):
        delta = end[index] - start[index]
        if delta == 0.0:
            if start[index] < lower[index] or start[index] > upper[index]:
                return False
            continue
        axis_min = (lower[index] - start[index]) / delta
        axis_max = (upper[index] - start[index]) / delta
        if axis_min > axis_max:
            axis_min, axis_max = axis_max, axis_min
        interval_min = max(interval_min, axis_min)
        interval_max = min(interval_max, axis_max)
        if interval_min > interval_max:
            return False
    return True


def path_points(message):
    return [
        (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        for pose in message.poses
    ]


def load_yaml_parameters(path, node_name):
    with path.open(encoding='utf-8') as config_file:
        document = yaml.safe_load(config_file)
    if node_name not in document or 'ros__parameters' not in document[node_name]:
        raise ValueError(f'{path} does not contain parameters for {node_name}')
    return document[node_name]['ros__parameters']


def load_configuration(repo_root):
    config_directory = repo_root / 'src' / 'drone_bringup' / 'config'
    mission_path = config_directory / 'multi_goal_mission.yaml'
    environment_path = config_directory / 'environment.yaml'
    dynamics_path = config_directory / 'dynamics.yaml'
    trajectory_path = config_directory / 'planned_trajectory.yaml'
    mission = load_yaml_parameters(mission_path, 'multi_goal_static_avoidance_node')
    environment = load_yaml_parameters(environment_path, '/**')
    dynamics = load_yaml_parameters(dynamics_path, 'quadrotor_dynamics_node')
    trajectory = load_yaml_parameters(trajectory_path, '/**')

    goal_values = mission['goals']
    if not goal_values or len(goal_values) % 4 != 0:
        raise ValueError('mission goals must be non-empty [x,y,z,yaw] groups')
    goals = []
    for offset in range(0, len(goal_values), 4):
        goal = tuple(float(value) for value in goal_values[offset:offset + 4])
        if not all(math.isfinite(value) for value in goal):
            raise ValueError('mission goals must be finite')
        goals.append(goal)

    obstacle_values = environment['obstacles']
    if not obstacle_values or len(obstacle_values) % 6 != 0:
        raise ValueError('environment obstacles must be center/size groups')
    original_obstacles = []
    for offset in range(0, len(obstacle_values), 6):
        center = obstacle_values[offset:offset + 3]
        size = obstacle_values[offset + 3:offset + 6]
        lower = tuple(center[axis] - 0.5 * size[axis] for axis in range(3))
        upper = tuple(center[axis] + 0.5 * size[axis] for axis in range(3))
        original_obstacles.append((lower, upper))
    safety_radius = float(environment['safety_radius'])
    inflated_obstacles = tuple(
        (tuple(value - safety_radius for value in lower),
         tuple(value + safety_radius for value in upper))
        for lower, upper in original_obstacles
    )
    return {
        'mission_path': mission_path,
        'environment_path': environment_path,
        'dynamics_path': dynamics_path,
        'trajectory_path': trajectory_path,
        'goals': tuple(goals),
        'nominal_speed': float(mission.get(
            'nominal_speed', trajectory['nominal_speed'])),
        'goal_position_tolerance': float(mission['goal_position_tolerance']),
        'goal_speed_tolerance': float(mission['goal_speed_tolerance']),
        'safety_radius': safety_radius,
        'max_rpm': float(dynamics['max_rpm']),
        'original_obstacles': tuple(original_obstacles),
        'inflated_obstacles': inflated_obstacles,
    }


def new_segment_metrics(goal_index, goal):
    return {
        'goal_index': goal_index,
        'goal_position': list(goal[:3]),
        'segment_start_time': None,
        'segment_accept_time': None,
        'segment_elapsed_time': None,
        'raw_path_points': 0,
        'raw_path_length': 0.0,
        'simplified_path_points': 0,
        'simplified_path_length': 0.0,
        'reference_path_points': 0,
        'reference_path_length': 0.0,
        'final_waypoint_count': None,
        'initial_simplified_points': None,
        'refinement_iterations': None,
        'selected_velocity_scale': None,
        'selected_duration_scale': None,
        'trajectory_duration': None,
        'expanded_nodes': None,
        'maximum_tracking_error': 0.0,
        'minimum_clearance': math.inf,
        'maximum_actual_speed': 0.0,
        'maximum_reference_speed': 0.0,
        'maximum_reference_acceleration': 0.0,
        'maximum_motor_rpm': 0.0,
        'minimum_motor_rpm': math.inf,
        'acceptance_position_error': None,
        'acceptance_speed': None,
    }


class EvaluationCollector(Node):

    def __init__(self, configuration, launch_start, context):
        super().__init__('multi_goal_mission_evaluation', context=context)
        self.configuration = configuration
        self.goals = configuration['goals']
        self.launch_start = launch_start
        self.takeoff_start_time = None
        self.navigation_start_time = None
        self.completion_time = None
        self.latest_position = None
        self.previous_position = None
        self.latest_actual_speed = math.inf
        self.latest_setpoint = None
        self.latest_rpm = None
        self.current_goal_index = None
        self.current_segment = None
        self.visited_goals = None
        self.mission_complete = False
        self.mission_success = None
        self.collision_state = False
        self.collision_observed = False
        self.point_collision_observed = False
        self.segment_collision_observed = False
        self.nonfinite_observed = False
        self.maximum_actual_speed = 0.0
        self.maximum_reference_speed = 0.0
        self.maximum_reference_acceleration = 0.0
        self.maximum_tracking_error = 0.0
        self.minimum_clearance = math.inf
        self.maximum_motor_rpm = 0.0
        self.minimum_motor_rpm = math.inf
        self.maximum_motor_peak_difference = 0.0
        self.post_complete_minimum_rpm = math.inf
        self.post_complete_maximum_rpm = 0.0
        self.raw_paths = []
        self.simplified_paths = []
        self.reference_paths = []
        self.latest_actual_path = []
        self.acceptance_positions = []
        self.observed_goal_indices = []
        self.observed_visited_counts = []
        self.rows = []
        self.rpm_rows = []
        self.segment_metrics = [
            new_segment_metrics(index, goal)
            for index, goal in enumerate(self.goals)
        ]
        self.health_errors = []
        self.odom_samples = 0
        self.setpoint_samples = 0
        self.rpm_samples = 0
        self.collision_samples = 0

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            PathMessage, '/drone/planned_path', self._on_raw_path, latched_qos)
        self.create_subscription(
            PathMessage, '/drone/simplified_path',
            self._on_simplified_path, latched_qos)
        self.create_subscription(
            PathMessage, '/drone/reference_path',
            self._on_reference_path, latched_qos)
        self.create_subscription(
            PathMessage, '/drone/path', self._on_actual_path, 10)
        self.create_subscription(
            Odometry, '/drone/odom', self._on_odometry, 10)
        self.create_subscription(
            TrajectorySetpoint, '/drone/trajectory_setpoint',
            self._on_setpoint, 10)
        self.create_subscription(
            MotorRPM, '/drone/motor_rpm_cmd', self._on_rpm, 10)
        self.create_subscription(
            UInt32, '/drone/multi_goal/current_goal_index',
            self._on_goal_index, 10)
        self.create_subscription(
            UInt32, '/drone/multi_goal/current_segment',
            self._on_segment, 10)
        self.create_subscription(
            UInt32, '/drone/multi_goal/visited_goals',
            self._on_visited, 10)
        self.create_subscription(
            Bool, '/drone/multi_goal/complete', self._on_complete, 10)
        self.create_subscription(
            Bool, '/drone/multi_goal/success', self._on_success, 10)
        self.create_subscription(
            Bool, '/drone/environment/in_collision', self._on_collision, 10)

    def _record_error(self, message):
        if message not in self.health_errors:
            self.health_errors.append(message)

    def _elapsed(self, now=None):
        return (now or time.monotonic()) - self.launch_start

    def _on_raw_path(self, message):
        points = path_points(message)
        if points:
            self.raw_paths.append(points)

    def _on_simplified_path(self, message):
        points = path_points(message)
        if points:
            self.simplified_paths.append(points)

    def _on_reference_path(self, message):
        points = path_points(message)
        if points:
            self.reference_paths.append(points)

    def _on_actual_path(self, message):
        points = path_points(message)
        if points:
            self.latest_actual_path = points

    def _on_goal_index(self, message):
        value = int(message.data)
        if value >= len(self.goals):
            self._record_error(f'current_goal_index out of range: {value}')
            return
        if self.current_goal_index is not None and value < self.current_goal_index:
            self._record_error('current_goal_index regressed')
        self.current_goal_index = value
        if not self.observed_goal_indices or self.observed_goal_indices[-1] != value:
            self.observed_goal_indices.append(value)
            if self.observed_goal_indices != list(range(len(self.observed_goal_indices))):
                self._record_error(
                    f'goals did not execute in order: {self.observed_goal_indices}')

    def _on_segment(self, message):
        self.current_segment = int(message.data)

    def _on_visited(self, message):
        value = int(message.data)
        if value > len(self.goals):
            self._record_error(f'visited_goals out of range: {value}')
            return
        if self.visited_goals is not None:
            if value < self.visited_goals:
                self._record_error('visited_goals regressed')
            if value > self.visited_goals + 1:
                self._record_error('visited_goals skipped a goal')
        if self.visited_goals is None or value != self.visited_goals:
            self.observed_visited_counts.append(value)
        if self.visited_goals is not None and value == self.visited_goals + 1:
            goal_index = value - 1
            now = time.monotonic()
            metrics = self.segment_metrics[goal_index]
            metrics['segment_accept_time'] = self._elapsed(now)
            if metrics['segment_start_time'] is not None:
                metrics['segment_elapsed_time'] = (
                    metrics['segment_accept_time'] - metrics['segment_start_time'])
            if self.latest_position is not None:
                goal = self.goals[goal_index]
                metrics['acceptance_position_error'] = math.dist(
                    self.latest_position, goal[:3])
                metrics['acceptance_speed'] = self.latest_actual_speed
                self.acceptance_positions.append(self.latest_position)
        self.visited_goals = value

    def _on_complete(self, message):
        value = bool(message.data)
        if value and self.completion_time is None:
            self.completion_time = time.monotonic()
        elif not value and self.completion_time is not None:
            self._record_error('mission complete regressed to false')
        self.mission_complete = value

    def _on_success(self, message):
        self.mission_success = bool(message.data)
        if not self.mission_success:
            self._record_error('mission success became false')

    def _on_collision(self, message):
        self.collision_samples += 1
        self.collision_state = bool(message.data)
        if self.collision_state:
            self.collision_observed = True
            self._record_error('/drone/environment/in_collision became true')

    def _on_setpoint(self, message):
        values = (
            message.position.x, message.position.y, message.position.z,
            message.velocity.x, message.velocity.y, message.velocity.z,
            message.acceleration.x, message.acceleration.y,
            message.acceleration.z, message.yaw,
        )
        if message.header.frame_id != 'map':
            self._record_error(f'invalid setpoint frame: {message.header.frame_id!r}')
            return
        if not all(math.isfinite(value) for value in values):
            self.nonfinite_observed = True
            self._record_error('non-finite trajectory setpoint')
            return
        now = time.monotonic()
        if self.takeoff_start_time is None:
            self.takeoff_start_time = now
        self.latest_setpoint = message
        self.setpoint_samples += 1
        reference_speed = norm3(values[3:6])
        reference_acceleration = norm3(values[6:9])
        self.maximum_reference_speed = max(
            self.maximum_reference_speed, reference_speed)
        self.maximum_reference_acceleration = max(
            self.maximum_reference_acceleration, reference_acceleration)
        if (self.navigation_start_time is None and
                (reference_speed > 1.0e-4 or reference_acceleration > 1.0e-4)):
            self.navigation_start_time = now
        if (self.navigation_start_time is not None and
                self.current_goal_index is not None and
                (reference_speed > 1.0e-4 or reference_acceleration > 1.0e-4)):
            metrics = self.segment_metrics[self.current_goal_index]
            if metrics['segment_start_time'] is None:
                metrics['segment_start_time'] = self._elapsed(now)
            metrics['maximum_reference_speed'] = max(
                metrics['maximum_reference_speed'], reference_speed)
            metrics['maximum_reference_acceleration'] = max(
                metrics['maximum_reference_acceleration'], reference_acceleration)

    def _on_rpm(self, message):
        values = (
            message.m1_front_left_ccw_rpm,
            message.m2_rear_left_cw_rpm,
            message.m3_rear_right_ccw_rpm,
            message.m4_front_right_cw_rpm,
        )
        if not all(math.isfinite(value) for value in values):
            self.nonfinite_observed = True
            self._record_error('non-finite motor RPM')
            return
        if any(value < 0.0 for value in values):
            self._record_error('negative motor RPM observed')
        if any(value > self.configuration['max_rpm'] for value in values):
            self._record_error('motor RPM exceeded configured maximum')
        now = time.monotonic()
        self.latest_rpm = values
        self.rpm_rows.append((self._elapsed(now),) + values)
        self.rpm_samples += 1
        self.maximum_motor_rpm = max(self.maximum_motor_rpm, max(values))
        self.minimum_motor_rpm = min(self.minimum_motor_rpm, min(values))
        self.maximum_motor_peak_difference = max(
            self.maximum_motor_peak_difference, max(values) - min(values))
        if self.completion_time is not None:
            self.post_complete_minimum_rpm = min(
                self.post_complete_minimum_rpm, min(values))
            self.post_complete_maximum_rpm = max(
                self.post_complete_maximum_rpm, max(values))
        if self.navigation_start_time is not None and self.current_goal_index is not None:
            metrics = self.segment_metrics[self.current_goal_index]
            if metrics['segment_start_time'] is not None and metrics['segment_accept_time'] is None:
                metrics['maximum_motor_rpm'] = max(
                    metrics['maximum_motor_rpm'], max(values))
                metrics['minimum_motor_rpm'] = min(
                    metrics['minimum_motor_rpm'], min(values))

    def _on_odometry(self, message):
        pose = message.pose.pose
        twist = message.twist.twist
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z,
            pose.orientation.w,
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.nonfinite_observed = True
            self.previous_position = None
            self._record_error('non-finite odometry')
            return
        now = time.monotonic()
        position = values[:3]
        speed = norm3(values[7:10])
        clearance = min(
            distance_to_box(position, lower, upper)
            for lower, upper in self.configuration['inflated_obstacles'])
        self.minimum_clearance = min(self.minimum_clearance, clearance)
        self.maximum_actual_speed = max(self.maximum_actual_speed, speed)
        for lower, upper in self.configuration['inflated_obstacles']:
            if inside_closed_box(position, lower, upper):
                self.point_collision_observed = True
                self._record_error(f'Odom point entered inflated obstacle: {position}')
            if (self.previous_position is not None and
                    segment_intersects_closed_box(
                        self.previous_position, position, lower, upper)):
                self.segment_collision_observed = True
                self._record_error(
                    f'Odom segment intersected inflated obstacle: '
                    f'{self.previous_position} -> {position}')
        self.previous_position = position
        self.latest_position = position
        self.latest_actual_speed = speed
        self.odom_samples += 1

        reference = None
        reference_speed = None
        reference_acceleration = None
        tracking_error = None
        if self.latest_setpoint is not None:
            reference = (
                self.latest_setpoint.position.x,
                self.latest_setpoint.position.y,
                self.latest_setpoint.position.z,
            )
            reference_velocity = (
                self.latest_setpoint.velocity.x,
                self.latest_setpoint.velocity.y,
                self.latest_setpoint.velocity.z,
            )
            reference_acceleration_vector = (
                self.latest_setpoint.acceleration.x,
                self.latest_setpoint.acceleration.y,
                self.latest_setpoint.acceleration.z,
            )
            reference_speed = norm3(reference_velocity)
            reference_acceleration = norm3(reference_acceleration_vector)
            if self.navigation_start_time is not None:
                tracking_error = math.dist(position, reference)
                if self.completion_time is None:
                    self.maximum_tracking_error = max(
                        self.maximum_tracking_error, tracking_error)
        if (self.navigation_start_time is not None and
                self.current_goal_index is not None):
            metrics = self.segment_metrics[self.current_goal_index]
            if metrics['segment_start_time'] is not None and metrics['segment_accept_time'] is None:
                metrics['maximum_actual_speed'] = max(
                    metrics['maximum_actual_speed'], speed)
                metrics['minimum_clearance'] = min(
                    metrics['minimum_clearance'], clearance)
                if tracking_error is not None:
                    metrics['maximum_tracking_error'] = max(
                        metrics['maximum_tracking_error'], tracking_error)

        rpm = self.latest_rpm or (None, None, None, None)
        self.rows.append({
            'time': self._elapsed(now),
            'mission_elapsed_time': (
                now - self.navigation_start_time
                if self.navigation_start_time is not None else None),
            'current_goal_index': self.current_goal_index,
            'current_segment': self.current_segment,
            'visited_goals': self.visited_goals,
            'actual_x': position[0],
            'actual_y': position[1],
            'actual_z': position[2],
            'reference_x': reference[0] if reference else None,
            'reference_y': reference[1] if reference else None,
            'reference_z': reference[2] if reference else None,
            'tracking_error': tracking_error,
            'actual_speed': speed,
            'reference_speed': reference_speed,
            'reference_acceleration': reference_acceleration,
            'clearance': clearance,
            'm1_rpm': rpm[0],
            'm2_rpm': rpm[1],
            'm3_rpm': rpm[2],
            'm4_rpm': rpm[3],
            'mission_complete': self.mission_complete,
            'mission_success': self.mission_success,
            'collision_state': self.collision_state,
        })


class LaunchOutput:

    def __init__(self, process, log_path):
        self.process = process
        self.log_path = log_path
        self.lines = []
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _read(self):
        with self.log_path.open('w', encoding='utf-8') as log_file:
            for line in self.process.stdout:
                self.lines.append(line)
                log_file.write(line)
                log_file.flush()
                print(line, end='')

    def join(self):
        self._thread.join(timeout=5.0)

    def text(self):
        return ''.join(self.lines)


def stop_launch(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5.0)


def parse_segment_logs(log_text):
    pattern = re.compile(
        r'ordered goal (\d+) trajectory ready: raw_points=(\d+) '
        r'simplified_points=(\d+) initial_simplified_points=(\d+) '
        r'refinements=(\d+) duration=([0-9.eE+-]+) s '
        r'velocity_scale=([0-9.eE+-]+) duration_scale=([0-9.eE+-]+) '
        r'max_speed=([0-9.eE+-]+) m/s '
        r'max_acceleration=([0-9.eE+-]+) m/s\^2 '
        r'raw_length=([0-9.eE+-]+) m simplified_length=([0-9.eE+-]+) m '
        r'expanded_nodes=(\d+)')
    results = []
    for match in pattern.finditer(log_text):
        values = match.groups()
        results.append({
            'goal_index': int(values[0]),
            'raw_path_points': int(values[1]),
            'final_waypoint_count': int(values[2]),
            'initial_simplified_points': int(values[3]),
            'refinement_iterations': int(values[4]),
            'trajectory_duration': float(values[5]),
            'selected_velocity_scale': float(values[6]),
            'selected_duration_scale': float(values[7]),
            'validated_maximum_reference_speed': float(values[8]),
            'validated_maximum_reference_acceleration': float(values[9]),
            'raw_path_length_from_log': float(values[10]),
            'simplified_path_length_from_log': float(values[11]),
            'expanded_nodes': int(values[12]),
        })
    return results


def finalize_segments(collector, log_text):
    for index, metrics in enumerate(collector.segment_metrics):
        if index < len(collector.raw_paths):
            metrics['raw_path_points'] = len(collector.raw_paths[index])
            metrics['raw_path_length'] = path_length(collector.raw_paths[index])
        if index < len(collector.simplified_paths):
            metrics['simplified_path_points'] = len(collector.simplified_paths[index])
            metrics['simplified_path_length'] = path_length(
                collector.simplified_paths[index])
        if index < len(collector.reference_paths):
            metrics['reference_path_points'] = len(collector.reference_paths[index])
            metrics['reference_path_length'] = path_length(
                collector.reference_paths[index])
        if not math.isfinite(metrics['minimum_clearance']):
            metrics['minimum_clearance'] = None
        if not math.isfinite(metrics['minimum_motor_rpm']):
            metrics['minimum_motor_rpm'] = None
    log_metrics = parse_segment_logs(log_text)
    for expected_index, parsed in enumerate(log_metrics):
        if parsed['goal_index'] != expected_index:
            collector._record_error('trajectory generation logs were not ordered by goal')
            continue
        if expected_index >= len(collector.segment_metrics):
            collector._record_error('trajectory generation produced an extra goal log')
            continue
        collector.segment_metrics[expected_index].update(parsed)


def write_csv(output_directory, rows):
    with (output_directory / 'trajectory.csv').open(
            'w', newline='', encoding='utf-8') as csv_file:
        writer = csv.DictWriter(
            csv_file, fieldnames=CSV_FIELDS, lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def add_obstacle_patches(axis, configuration):
    for index, (lower, upper) in enumerate(configuration['original_obstacles']):
        axis.add_patch(Rectangle(
            (lower[0], lower[1]), upper[0] - lower[0], upper[1] - lower[1],
            facecolor='0.55', edgecolor='0.25', alpha=0.65,
            label='original obstacle' if index == 0 else None))
    for index, (lower, upper) in enumerate(configuration['inflated_obstacles']):
        axis.add_patch(Rectangle(
            (lower[0], lower[1]), upper[0] - lower[0], upper[1] - lower[1],
            facecolor='none', edgecolor='tab:red', linestyle='--', linewidth=1.2,
            label=f'{configuration["safety_radius"]:.2f} m inflated boundary'
            if index == 0 else None))


def add_acceptance_lines(axis, collector):
    for index, metrics in enumerate(collector.segment_metrics):
        if metrics['segment_accept_time'] is not None:
            axis.axvline(
                metrics['segment_accept_time'], color=f'C{index + 2}',
                linestyle=':', alpha=0.8, label=f'P{index + 1} accepted')


def plot_xy(axis, collector, configuration):
    add_obstacle_patches(axis, configuration)
    for index, reference_path in enumerate(collector.reference_paths):
        axis.plot(
            [point[0] for point in reference_path],
            [point[1] for point in reference_path],
            linestyle='--', linewidth=1.4, label=f'P{index + 1} reference')
    axis.plot(
        [row['actual_x'] for row in collector.rows],
        [row['actual_y'] for row in collector.rows],
        color='tab:green', linewidth=1.5, label='actual trajectory')
    if collector.rows:
        axis.scatter(
            collector.rows[0]['actual_x'], collector.rows[0]['actual_y'],
            marker='o', color='black', s=35, label='start', zorder=5)
    for index, goal in enumerate(collector.goals):
        axis.scatter(goal[0], goal[1], marker='*', s=90, zorder=5)
        axis.annotate(f'P{index + 1}', (goal[0], goal[1]), xytext=(4, 5),
                      textcoords='offset points')
    for index, point in enumerate(collector.acceptance_positions):
        axis.scatter(point[0], point[1], marker='x', s=45, zorder=6,
                     label='acceptance point' if index == 0 else None)
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')
    axis.axis('equal')
    axis.grid(True, alpha=0.3)


def write_plots(output_directory, collector, configuration):
    times = [row['time'] for row in collector.rows]

    figure, axis = plt.subplots(figsize=(9.0, 6.5))
    plot_xy(axis, collector, configuration)
    axis.set_title('Complete multi-goal mission XY path')
    axis.legend(loc='best', fontsize=8)
    figure.tight_layout()
    figure.savefig(output_directory / 'xy_path.png', dpi=160)
    plt.close(figure)

    figure, axes = plt.subplots(3, 1, figsize=(10.0, 8.5), sharex=True)
    for axis_index, coordinate in enumerate(('x', 'y', 'z')):
        axis = axes[axis_index]
        axis.plot(times, [row[f'actual_{coordinate}'] for row in collector.rows],
                  label=f'actual {coordinate}', color='tab:green')
        axis.plot(times, [row[f'reference_{coordinate}'] for row in collector.rows],
                  label=f'reference {coordinate}', color='tab:blue', linestyle='--')
        add_acceptance_lines(axis, collector)
        axis.set_ylabel(f'{coordinate} [m]')
        axis.grid(True, alpha=0.3)
        axis.legend(loc='best', fontsize=8)
    axes[-1].set_xlabel('time since launch [s]')
    figure.suptitle('Position tracking')
    figure.tight_layout()
    figure.savefig(output_directory / 'position_tracking.png', dpi=160)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(10.0, 4.8))
    axis.plot(times, [row['actual_speed'] for row in collector.rows],
              color='tab:green', label='Actual speed')
    axis.plot(times, [row['reference_speed'] for row in collector.rows],
              color='tab:blue', linestyle='--', label='Reference speed')
    axis.axhline(
        configuration['nominal_speed'], color='tab:orange', linestyle=':',
        label='Nominal speed (planning parameter, not constant command)')
    add_acceptance_lines(axis, collector)
    axis.set_xlabel('time since launch [s]')
    axis.set_ylabel('speed [m/s]')
    axis.set_title('Actual and reference speed tracking')
    axis.grid(True, alpha=0.3)
    axis.legend(loc='best', fontsize=8)
    figure.tight_layout()
    figure.savefig(output_directory / 'speed_tracking.png', dpi=160)
    plt.close(figure)

    tracking_rows = [
        row for row in collector.rows
        if row['tracking_error'] is not None]
    figure, axis = plt.subplots(figsize=(10.0, 4.8))
    axis.plot(
        [row['time'] for row in tracking_rows],
        [row['tracking_error'] for row in tracking_rows],
        color='tab:purple', label='3D tracking error')
    axis.axhline(0.05, color='tab:orange', linestyle='--', label='0.05 m acceptance')
    axis.axhline(0.10, color='tab:red', linestyle='--', label='0.10 m acceptance')
    add_acceptance_lines(axis, collector)
    axis.set_xlabel('time since launch [s]')
    axis.set_ylabel('tracking error [m]')
    axis.set_title('Multi-goal trajectory tracking error')
    axis.grid(True, alpha=0.3)
    axis.legend(loc='best', fontsize=8)
    figure.tight_layout()
    figure.savefig(output_directory / 'tracking_error.png', dpi=160)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(10.0, 4.8))
    axis.plot(times, [row['clearance'] for row in collector.rows],
              color='tab:green', label='actual obstacle clearance')
    axis.axhline(
        0.05, color='tab:red', linestyle='--',
        label='0.05 m regression threshold')
    add_acceptance_lines(axis, collector)
    axis.set_xlabel('time since launch [s]')
    axis.set_ylabel('clearance [m]')
    axis.set_title('Clearance from base-inflated obstacles')
    axis.grid(True, alpha=0.3)
    axis.legend(loc='best', fontsize=8)
    figure.tight_layout()
    figure.savefig(output_directory / 'clearance.png', dpi=160)
    plt.close(figure)

    rpm_times = [row[0] for row in collector.rpm_rows]
    figure, axis = plt.subplots(figsize=(10.0, 5.2))
    labels = ('M1 front-left CCW', 'M2 rear-left CW',
              'M3 rear-right CCW', 'M4 front-right CW')
    for motor_index, label in enumerate(labels):
        axis.plot(rpm_times, [row[motor_index + 1] for row in collector.rpm_rows],
                  linewidth=1.0, label=label)
    axis.axvspan(
        collector._elapsed(collector.takeoff_start_time)
        if collector.takeoff_start_time else 0.0,
        collector._elapsed(collector.navigation_start_time)
        if collector.navigation_start_time else 0.0,
        color='tab:cyan', alpha=0.10, label='takeoff phase')
    axis.axhline(
        configuration['max_rpm'], color='tab:red', linestyle='--',
        label=f'{configuration["max_rpm"]:.0f} RPM limit')
    add_acceptance_lines(axis, collector)
    axis.annotate(
        f'max {collector.maximum_motor_rpm:.1f} RPM',
        xy=(0.99, 0.97), xycoords='axes fraction', ha='right', va='top')
    axis.set_xlabel('time since launch [s]')
    axis.set_ylabel('motor speed [RPM]')
    axis.set_title('Four-motor RPM history')
    axis.grid(True, alpha=0.3)
    axis.legend(loc='best', fontsize=8)
    figure.tight_layout()
    figure.savefig(output_directory / 'motor_rpm.png', dpi=160)
    plt.close(figure)

    figure, axes = plt.subplots(3, 2, figsize=(13.0, 12.0))
    plot_xy(axes[0, 0], collector, configuration)
    axes[0, 0].set_title('XY path')
    axes[0, 0].legend(loc='best', fontsize=7)
    axes[0, 1].plot(times, [row['actual_speed'] for row in collector.rows],
                    label='actual', color='tab:green')
    axes[0, 1].plot(times, [row['reference_speed'] for row in collector.rows],
                    label='reference', color='tab:blue', linestyle='--')
    axes[0, 1].axhline(configuration['nominal_speed'], color='tab:orange', linestyle=':')
    axes[0, 1].set_title('Speed [m/s]')
    axes[1, 0].plot(
        [row['time'] for row in tracking_rows],
        [row['tracking_error'] for row in tracking_rows], color='tab:purple')
    axes[1, 0].axhline(0.05, color='tab:red', linestyle='--')
    axes[1, 0].set_title('Tracking error [m]')
    axes[1, 1].plot(times, [row['clearance'] for row in collector.rows],
                    color='tab:green')
    axes[1, 1].axhline(0.05, color='tab:red', linestyle='--')
    axes[1, 1].set_title('Obstacle clearance [m]')
    for motor_index, label in enumerate(labels):
        axes[2, 0].plot(
            rpm_times, [row[motor_index + 1] for row in collector.rpm_rows],
            linewidth=0.8, label=label.split()[0])
    axes[2, 0].axhline(configuration['max_rpm'], color='tab:red', linestyle='--')
    axes[2, 0].set_title('Motor RPM')
    axes[2, 0].legend(loc='best', fontsize=7)
    axes[2, 1].axis('off')
    summary = (
        f'Goals completed: {collector.visited_goals}/{len(collector.goals)}\n'
        f'Launch to complete: '
        f'{collector._elapsed(collector.completion_time):.3f} s\n'
        f'Max tracking error: {collector.maximum_tracking_error:.6f} m\n'
        f'Min clearance: {collector.minimum_clearance:.6f} m\n'
        f'Max actual/reference speed: {collector.maximum_actual_speed:.6f} / '
        f'{collector.maximum_reference_speed:.6f} m/s\n'
        f'Motor RPM range: {collector.minimum_motor_rpm:.1f} - '
        f'{collector.maximum_motor_rpm:.1f}')
    axes[2, 1].text(0.05, 0.95, summary, va='top', family='monospace')
    for axis in axes.flat[:-1]:
        axis.grid(True, alpha=0.3)
    figure.suptitle('Default multi-goal mission evaluation')
    figure.tight_layout()
    figure.savefig(output_directory / 'mission_summary.png', dpi=160)
    plt.close(figure)


def validate_metrics(metrics, collector, configuration):
    failures = []
    if not metrics['mission_complete']:
        failures.append('mission did not complete')
    if not metrics['mission_success']:
        failures.append('mission success was not true')
    if metrics['visited_goals'] != len(configuration['goals']):
        failures.append('not all configured goals were visited')
    if collector.observed_goal_indices != list(range(len(configuration['goals']))):
        failures.append('goals were not executed in configured order')
    if metrics['collision_observed']:
        failures.append('environment collision state became true')
    if metrics['actual_point_collision_observed']:
        failures.append('actual Odom point entered a base-inflated obstacle')
    if metrics['actual_segment_collision_observed']:
        failures.append('actual Odom segment intersected a base-inflated obstacle')
    if metrics['nonfinite_observed']:
        failures.append('NaN or Inf was observed')
    if metrics['controller_saturation_count'] != 0:
        failures.append('controller saturation was observed')
    if metrics['maximum_tracking_error_m'] >= 0.05:
        failures.append('maximum tracking error was not below 0.05 m')
    if metrics['minimum_clearance_m'] <= 0.05:
        failures.append('minimum clearance did not exceed 0.05 m')
    if metrics['final_position_error_m'] >= 0.05:
        failures.append('final position error was not below 0.05 m')
    if metrics['final_speed_m_s'] >= 0.03:
        failures.append('final speed was not below 0.03 m/s')
    if metrics['minimum_motor_rpm'] < 0.0:
        failures.append('negative RPM was observed')
    if metrics['maximum_motor_rpm'] > configuration['max_rpm']:
        failures.append('configured RPM maximum was exceeded')
    if metrics['post_complete_minimum_motor_rpm'] <= 1000.0:
        failures.append('post-completion motors did not remain in hover operation')
    if len(collector.raw_paths) != len(configuration['goals']):
        failures.append('did not receive one raw path per goal')
    if len(collector.simplified_paths) != len(configuration['goals']):
        failures.append('did not receive one simplified path per goal')
    if len(collector.reference_paths) != len(configuration['goals']):
        failures.append('did not receive one reference path per goal')
    failures.extend(collector.health_errors)
    metrics['acceptance_failures'] = list(dict.fromkeys(failures))
    metrics['passed'] = not metrics['acceptance_failures'] and metrics['runtime_error'] is None


def build_metrics(collector, configuration, log_text, runtime_error, repo_root):
    finalize_segments(collector, log_text)
    final_position_error = math.inf
    if collector.latest_position is not None:
        final_position_error = math.dist(
            collector.latest_position, collector.goals[-1][:3])
    metrics = {
        'ros_domain_id': DOMAIN_ID,
        'mission_config': str(configuration['mission_path'].relative_to(repo_root)),
        'environment_config': str(
            configuration['environment_path'].relative_to(repo_root)),
        'goals': [list(goal) for goal in collector.goals],
        'nominal_speed_m_s': configuration['nominal_speed'],
        'launch_to_complete_time_s': (
            collector._elapsed(collector.completion_time)
            if collector.completion_time is not None else None),
        'takeoff_start_time_s': (
            collector._elapsed(collector.takeoff_start_time)
            if collector.takeoff_start_time is not None else None),
        'navigation_start_time_s': (
            collector._elapsed(collector.navigation_start_time)
            if collector.navigation_start_time is not None else None),
        'mission_complete_time_s': (
            collector._elapsed(collector.completion_time)
            if collector.completion_time is not None else None),
        'navigation_execution_time_s': (
            collector.completion_time - collector.navigation_start_time
            if collector.completion_time is not None and
            collector.navigation_start_time is not None else None),
        'visited_goals': collector.visited_goals,
        'mission_complete': collector.mission_complete,
        'mission_success': collector.mission_success is True,
        'maximum_tracking_error_m': collector.maximum_tracking_error,
        'minimum_clearance_m': collector.minimum_clearance,
        'maximum_actual_speed_m_s': collector.maximum_actual_speed,
        'maximum_reference_speed_m_s': collector.maximum_reference_speed,
        'maximum_reference_acceleration_m_s2':
            collector.maximum_reference_acceleration,
        'maximum_motor_rpm': collector.maximum_motor_rpm,
        'minimum_motor_rpm': collector.minimum_motor_rpm,
        'maximum_motor_peak_difference_rpm':
            collector.maximum_motor_peak_difference,
        'post_complete_minimum_motor_rpm': collector.post_complete_minimum_rpm,
        'post_complete_maximum_motor_rpm': collector.post_complete_maximum_rpm,
        'motor_rpm_limit': configuration['max_rpm'],
        'final_position_error_m': final_position_error,
        'final_speed_m_s': collector.latest_actual_speed,
        'collision_observed': collector.collision_observed,
        'actual_point_collision_observed': collector.point_collision_observed,
        'actual_segment_collision_observed': collector.segment_collision_observed,
        'controller_saturation_count': log_text.count('saturated=true'),
        'nonfinite_observed': collector.nonfinite_observed,
        'observed_goal_indices': collector.observed_goal_indices,
        'observed_visited_counts': collector.observed_visited_counts,
        'actual_path_points': len(collector.latest_actual_path),
        'odom_samples': collector.odom_samples,
        'setpoint_samples': collector.setpoint_samples,
        'rpm_samples': collector.rpm_samples,
        'collision_state_samples': collector.collision_samples,
        'segments': collector.segment_metrics,
        'runtime_error': runtime_error,
    }
    validate_metrics(metrics, collector, configuration)
    return metrics


def write_metrics(output_directory, metrics):
    def json_safe(value):
        if isinstance(value, float) and not math.isfinite(value):
            return None
        if isinstance(value, dict):
            return {key: json_safe(item) for key, item in value.items()}
        if isinstance(value, list):
            return [json_safe(item) for item in value]
        return value

    with (output_directory / 'metrics.json').open(
            'w', encoding='utf-8') as metrics_file:
        json.dump(
            json_safe(metrics), metrics_file,
            indent=2, sort_keys=True, allow_nan=False)
        metrics_file.write('\n')


def run_evaluation(repo_root, output_directory, timeout):
    configuration = load_configuration(repo_root)
    output_directory.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = str(DOMAIN_ID)
    command = [
        'ros2', 'launch', 'drone_bringup',
        'multi_goal_static_avoidance_sim.launch.py', 'use_rviz:=false',
        f'mission_config:={configuration["mission_path"]}',
    ]
    print(
        f'=== default multi-goal mission: domain={DOMAIN_ID} '
        f'goals={len(configuration["goals"])} ===')
    launch_start = time.monotonic()
    process = subprocess.Popen(
        command, cwd=repo_root, env=environment,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1, start_new_session=True)
    output = LaunchOutput(process, output_directory / 'launch.log')
    context = Context()
    collector = None
    executor = None
    runtime_error = None
    try:
        rclpy.init(context=context, domain_id=DOMAIN_ID)
        collector = EvaluationCollector(configuration, launch_start, context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(collector)
        deadline = launch_start + timeout
        graph_checked = False
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            now = time.monotonic()
            if process.poll() is not None:
                raise RuntimeError(
                    f'launch exited early with code {process.returncode}')
            if not graph_checked and now - launch_start >= 3.0:
                graph_checked = True
                required_topics = (
                    '/drone/odom', '/drone/trajectory_setpoint',
                    '/drone/motor_rpm_cmd', '/drone/multi_goal/complete')
                conflicts = [
                    topic for topic in required_topics
                    if collector.count_publishers(topic) != 1]
                if conflicts:
                    raise RuntimeError(
                        f'ROS Domain {DOMAIN_ID} conflict or missing publishers: {conflicts}')
            if now - launch_start > 10.0 and collector.odom_samples == 0:
                raise RuntimeError('odometry did not appear within 10 seconds')
            if collector.health_errors:
                raise RuntimeError(collector.health_errors[0])
            if (collector.completion_time is not None and
                    now - collector.completion_time >= POST_COMPLETE_DURATION):
                break
        else:
            raise TimeoutError('multi-goal mission did not complete before timeout')
    except Exception as error:
        runtime_error = str(error)
    finally:
        if executor is not None and collector is not None:
            executor.remove_node(collector)
            executor.shutdown()
        if collector is not None:
            collector.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)
        stop_launch(process)
        output.join()

    if collector is None:
        raise RuntimeError(runtime_error or 'collector initialization failed')
    log_text = output.text()
    write_csv(output_directory, collector.rows)
    metrics = build_metrics(
        collector, configuration, log_text, runtime_error, repo_root)
    write_metrics(output_directory, metrics)
    if collector.rows and collector.rpm_rows:
        write_plots(output_directory, collector, configuration)
    print(json.dumps(metrics, indent=2, sort_keys=True))
    return metrics


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Evaluate the complete default ordered multi-goal mission.')
    parser.add_argument('--timeout', type=float, default=200.0)
    parser.add_argument(
        '--output-directory', type=Path,
        help='Defaults to results/multi_goal_evaluation/default_mission.')
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    if not math.isfinite(arguments.timeout) or arguments.timeout <= 0.0:
        raise ValueError('timeout must be finite and positive')
    repo_root = Path(__file__).resolve().parents[1]
    output_directory = arguments.output_directory or (
        repo_root / 'results' / 'multi_goal_evaluation' / 'default_mission')
    metrics = run_evaluation(
        repo_root, output_directory.resolve(), arguments.timeout)
    if not metrics['passed']:
        raise SystemExit('multi-goal mission evaluation failed')
    print('\nDefault multi-goal mission evaluation passed.')


if __name__ == '__main__':
    main()
