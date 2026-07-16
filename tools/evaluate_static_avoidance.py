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

from drone_msgs.msg import MotorRPM, TrajectorySetpoint
from nav_msgs.msg import Odometry, Path as PathMessage
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, UInt32


START = (0.0, 0.0, 1.5)
ORIGINAL_OBSTACLES = (
    ((2.1, -0.5, 0.0), (2.9, 2.5, 3.0)),
    ((5.6, 2.5, 0.0), (6.4, 5.5, 3.0)),
)
BASE_INFLATED_OBSTACLES = (
    ((1.85, -0.75, -0.25), (3.15, 2.75, 3.25)),
    ((5.35, 2.25, -0.25), (6.65, 5.75, 3.25)),
)
SCENARIOS = {
    'scenario_a_default': {
        'goal': (8.0, 5.0, 1.5),
        'config': 'astar_evaluation_scenario_a.yaml',
        'domain_id': 110,
    },
    'scenario_b_horizontal': {
        'goal': (8.0, 6.5, 1.5),
        'config': 'astar_evaluation_scenario_b.yaml',
        'domain_id': 111,
    },
    'scenario_c_3d': {
        'goal': (8.0, 5.0, 4.0),
        'config': 'astar_evaluation_scenario_c.yaml',
        'domain_id': 112,
    },
}
CSV_FIELDS = (
    'time',
    'actual_x', 'actual_y', 'actual_z',
    'reference_x', 'reference_y', 'reference_z',
    'tracking_error', 'actual_speed', 'reference_speed',
    'clearance', 'segment_index',
)


def norm3(values):
    return math.sqrt(sum(value * value for value in values))


def path_length(points):
    return sum(
        norm3(tuple(points[index][axis] - points[index - 1][axis]
                    for axis in range(3)))
        for index in range(1, len(points))
    )


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


class EvaluationCollector(Node):

    def __init__(self, scenario_name, goal, launch_start, context):
        super().__init__(f'static_avoidance_evaluation_{scenario_name}', context=context)
        self.goal = goal
        self.launch_start = launch_start
        self.planning_success = None
        self.generation_success = None
        self.planning_result_receive_time = None
        self.raw_path = []
        self.simplified_path = []
        self.reference_path = []
        self.trajectory_duration = None
        self.selected_velocity_scale = None
        self.execution_start_time = None
        self.completion_time = None
        self.latest_setpoint = None
        self.latest_segment = 0
        self.latest_position = None
        self.latest_speed = math.inf
        self.previous_odom_position = None
        self.maximum_tracking_error = 0.0
        self.minimum_clearance = math.inf
        self.maximum_sampled_reference_speed = 0.0
        self.maximum_sampled_reference_acceleration = 0.0
        self.odom_samples = 0
        self.setpoint_samples = 0
        self.rpm_samples = 0
        self.collision_samples = 0
        self.collision_observed = False
        self.point_collision_observed = False
        self.segment_collision_observed = False
        self.nonfinite_observed = False
        self.rows = []
        self.health_errors = []

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            Bool, '/drone/planning/success', self._on_planning_success, latched_qos)
        self.create_subscription(
            Bool, '/drone/trajectory_generation/success',
            self._on_generation_success, latched_qos)
        self.create_subscription(
            PathMessage, '/drone/planned_path', self._on_raw_path, latched_qos)
        self.create_subscription(
            PathMessage, '/drone/simplified_path', self._on_simplified_path, latched_qos)
        self.create_subscription(
            PathMessage, '/drone/reference_path', self._on_reference_path, latched_qos)
        self.create_subscription(
            Float64, '/drone/trajectory_generation/duration',
            self._on_duration, latched_qos)
        self.create_subscription(
            Float64, '/drone/trajectory_generation/selected_velocity_scale',
            self._on_velocity_scale, latched_qos)
        self.create_subscription(
            UInt32, '/drone/planned_trajectory/current_segment', self._on_segment, 10)
        self.create_subscription(
            Bool, '/drone/planned_trajectory/complete', self._on_complete, 10)
        self.create_subscription(
            Bool, '/drone/environment/in_collision', self._on_collision, 10)
        self.create_subscription(
            TrajectorySetpoint, '/drone/trajectory_setpoint', self._on_setpoint, 10)
        self.create_subscription(Odometry, '/drone/odom', self._on_odometry, 10)
        self.create_subscription(MotorRPM, '/drone/motor_rpm_cmd', self._on_rpm, 10)

    def _record_error(self, message):
        if message not in self.health_errors:
            self.health_errors.append(message)

    def _on_planning_success(self, message):
        if self.planning_result_receive_time is None:
            self.planning_result_receive_time = time.monotonic() - self.launch_start
        self.planning_success = bool(message.data)

    def _on_generation_success(self, message):
        self.generation_success = bool(message.data)

    def _on_raw_path(self, message):
        self.raw_path = path_points(message)

    def _on_simplified_path(self, message):
        self.simplified_path = path_points(message)

    def _on_reference_path(self, message):
        self.reference_path = path_points(message)

    def _on_duration(self, message):
        self.trajectory_duration = float(message.data)

    def _on_velocity_scale(self, message):
        self.selected_velocity_scale = float(message.data)

    def _on_segment(self, message):
        self.latest_segment = int(message.data)

    def _on_complete(self, message):
        if message.data and self.completion_time is None:
            self.completion_time = time.monotonic()
        elif not message.data and self.completion_time is not None:
            self._record_error('mission completion state regressed to false')

    def _on_collision(self, message):
        self.collision_samples += 1
        if message.data:
            self.collision_observed = True
            self._record_error('/drone/environment/in_collision became true')

    def _on_setpoint(self, message):
        values = (
            message.position.x, message.position.y, message.position.z,
            message.velocity.x, message.velocity.y, message.velocity.z,
            message.acceleration.x, message.acceleration.y, message.acceleration.z,
            message.yaw,
        )
        if message.header.frame_id != 'map':
            self._record_error(f'invalid setpoint frame {message.header.frame_id!r}')
            return
        if not all(math.isfinite(value) for value in values):
            self.nonfinite_observed = True
            self._record_error('non-finite trajectory setpoint')
            return
        reference_speed = norm3(values[3:6])
        reference_acceleration = norm3(values[6:9])
        self.maximum_sampled_reference_speed = max(
            self.maximum_sampled_reference_speed, reference_speed)
        self.maximum_sampled_reference_acceleration = max(
            self.maximum_sampled_reference_acceleration, reference_acceleration)
        if (self.execution_start_time is None and
                (reference_speed > 1.0e-4 or reference_acceleration > 1.0e-4)):
            self.execution_start_time = time.monotonic()
        self.latest_setpoint = message
        self.setpoint_samples += 1

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
            self.previous_odom_position = None
            self._record_error('non-finite odometry')
            return

        now = time.monotonic()
        position = values[0:3]
        speed = norm3(values[7:10])
        clearance = min(
            distance_to_box(position, lower, upper)
            for lower, upper in BASE_INFLATED_OBSTACLES
        )
        self.minimum_clearance = min(self.minimum_clearance, clearance)
        for lower, upper in BASE_INFLATED_OBSTACLES:
            if inside_closed_box(position, lower, upper):
                self.point_collision_observed = True
                self._record_error(f'Odom point entered base-inflated obstacle: {position}')
            if (self.previous_odom_position is not None and
                    segment_intersects_closed_box(
                        self.previous_odom_position, position, lower, upper)):
                self.segment_collision_observed = True
                self._record_error(
                    'Odom segment intersected base-inflated obstacle: '
                    f'{self.previous_odom_position} -> {position}')
        self.previous_odom_position = position
        self.latest_position = position
        self.latest_speed = speed
        self.odom_samples += 1

        if self.execution_start_time is None or self.latest_setpoint is None:
            return
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
        tracking_error = norm3(tuple(
            position[index] - reference[index] for index in range(3)))
        if self.completion_time is None:
            self.maximum_tracking_error = max(
                self.maximum_tracking_error, tracking_error)
        self.rows.append({
            'time': now - self.execution_start_time,
            'actual_x': position[0],
            'actual_y': position[1],
            'actual_z': position[2],
            'reference_x': reference[0],
            'reference_y': reference[1],
            'reference_z': reference[2],
            'tracking_error': tracking_error,
            'actual_speed': speed,
            'reference_speed': norm3(reference_velocity),
            'clearance': clearance,
            'segment_index': self.latest_segment,
        })

    def _on_rpm(self, message):
        values = (
            message.m1_front_left_ccw_rpm,
            message.m2_rear_left_cw_rpm,
            message.m3_rear_right_ccw_rpm,
            message.m4_front_right_cw_rpm,
        )
        if not all(math.isfinite(value) for value in values):
            self.nonfinite_observed = True
            self._record_error('non-finite motor RPM command')
            return
        self.rpm_samples += 1


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


def validated_reference_extrema(log_text, collector):
    matches = re.findall(
        r'max_speed=([0-9.eE+-]+) m/s max_acceleration=([0-9.eE+-]+) m/s\^2',
        log_text,
    )
    if matches:
        speed, acceleration = matches[-1]
        return float(speed), float(acceleration)
    return (
        collector.maximum_sampled_reference_speed,
        collector.maximum_sampled_reference_acceleration,
    )


def write_csv(output_directory, rows):
    with (output_directory / 'trajectory.csv').open(
            'w', newline='', encoding='utf-8') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def add_obstacle_patches(axis):
    for index, (lower, upper) in enumerate(ORIGINAL_OBSTACLES):
        axis.add_patch(Rectangle(
            (lower[0], lower[1]),
            upper[0] - lower[0],
            upper[1] - lower[1],
            facecolor='0.55',
            edgecolor='0.25',
            alpha=0.65,
            label='original obstacle' if index == 0 else None,
        ))
    for index, (lower, upper) in enumerate(BASE_INFLATED_OBSTACLES):
        axis.add_patch(Rectangle(
            (lower[0], lower[1]),
            upper[0] - lower[0],
            upper[1] - lower[1],
            facecolor='none',
            edgecolor='tab:red',
            linestyle='--',
            linewidth=1.5,
            label='0.25 m inflated boundary' if index == 0 else None,
        ))


def write_plots(output_directory, collector):
    times = [row['time'] for row in collector.rows]

    figure, axis = plt.subplots(figsize=(8.0, 6.0))
    add_obstacle_patches(axis)
    axis.plot(
        [point[0] for point in collector.reference_path],
        [point[1] for point in collector.reference_path],
        color='tab:blue', label='reference trajectory')
    axis.plot(
        [row['actual_x'] for row in collector.rows],
        [row['actual_y'] for row in collector.rows],
        color='tab:orange', label='actual trajectory')
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')
    axis.set_title('Static avoidance XY path')
    axis.axis('equal')
    axis.grid(True, alpha=0.3)
    axis.legend(loc='best')
    figure.tight_layout()
    figure.savefig(output_directory / 'xy_path.png', dpi=160)
    plt.close(figure)

    figure, axes = plt.subplots(3, 1, figsize=(9.0, 8.0), sharex=True)
    for axis_index, coordinate in enumerate(('x', 'y', 'z')):
        axes[axis_index].plot(
            times, [row[f'actual_{coordinate}'] for row in collector.rows],
            label=f'actual {coordinate}', color='tab:orange')
        axes[axis_index].plot(
            times, [row[f'reference_{coordinate}'] for row in collector.rows],
            label=f'reference {coordinate}', color='tab:blue', linestyle='--')
        axes[axis_index].set_ylabel(f'{coordinate} [m]')
        axes[axis_index].grid(True, alpha=0.3)
        axes[axis_index].legend(loc='best')
    axes[-1].set_xlabel('trajectory execution time [s]')
    figure.suptitle('Position tracking')
    figure.tight_layout()
    figure.savefig(output_directory / 'position_tracking.png', dpi=160)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(9.0, 4.5))
    axis.plot(times, [row['tracking_error'] for row in collector.rows],
              color='tab:purple', label='3D tracking error')
    axis.axhline(0.10, color='tab:red', linestyle='--', label='0.10 m acceptance')
    axis.set_xlabel('trajectory execution time [s]')
    axis.set_ylabel('tracking error [m]')
    axis.set_title('Trajectory tracking error')
    axis.grid(True, alpha=0.3)
    axis.legend(loc='best')
    figure.tight_layout()
    figure.savefig(output_directory / 'tracking_error.png', dpi=160)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(9.0, 4.5))
    axis.plot(times, [row['clearance'] for row in collector.rows],
              color='tab:green', label='actual obstacle clearance')
    axis.axhline(0.05, color='tab:red', linestyle='--', label='0.05 m regression')
    axis.set_xlabel('trajectory execution time [s]')
    axis.set_ylabel('clearance [m]')
    axis.set_title('Clearance from base-inflated obstacles')
    axis.grid(True, alpha=0.3)
    axis.legend(loc='best')
    figure.tight_layout()
    figure.savefig(output_directory / 'clearance.png', dpi=160)
    plt.close(figure)


def validate_metrics(metrics, collector):
    failures = []
    if not metrics['planning_success']:
        failures.append('A* no path or planning failure')
    if not metrics['trajectory_generation_success']:
        failures.append('trajectory generation or dynamic constraint failure')
    if not metrics['mission_complete']:
        failures.append('execution did not complete')
    if metrics['collision_observed']:
        failures.append('environment collision state became true')
    if metrics['actual_point_collision_observed']:
        failures.append('actual Odom point entered a base-inflated obstacle')
    if metrics['actual_segment_collision_observed']:
        failures.append('actual Odom segment intersected a base-inflated obstacle')
    if metrics['maximum_tracking_error_m'] >= 0.10:
        failures.append('execution tracking error exceeded 0.10 m')
    if metrics['minimum_clearance_m'] <= 0.05:
        failures.append('safety clearance did not exceed 0.05 m')
    if metrics['final_position_error_m'] >= 0.20:
        failures.append('final position error exceeded 0.20 m')
    if metrics['final_speed_m_s'] >= 0.15:
        failures.append('final speed exceeded 0.15 m/s')
    if metrics['controller_saturated_true_count'] != 0:
        failures.append('controller saturation was observed')
    if metrics['nonfinite_observed']:
        failures.append('NaN or Inf was observed')
    failures.extend(collector.health_errors)
    metrics['acceptance_failures'] = list(dict.fromkeys(failures))
    metrics['passed'] = not metrics['acceptance_failures']


def run_scenario(repo_root, scenario_name, scenario, output_root, timeout):
    output_directory = output_root / scenario_name
    output_directory.mkdir(parents=True, exist_ok=True)
    config_path = (
        repo_root / 'src' / 'drone_bringup' / 'config' / scenario['config'])
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = str(scenario['domain_id'])
    command = [
        'ros2', 'launch', 'drone_bringup', 'static_avoidance_sim.launch.py',
        'use_rviz:=false', f'astar_config:={config_path}',
    ]
    print(f'\n=== {scenario_name}: domain={scenario["domain_id"]} goal={scenario["goal"]} ===')
    launch_start = time.monotonic()
    process = subprocess.Popen(
        command,
        cwd=repo_root,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        start_new_session=True,
    )
    output = LaunchOutput(process, output_directory / 'launch.log')
    context = Context()
    rclpy.init(context=context, domain_id=scenario['domain_id'])
    collector = EvaluationCollector(
        scenario_name, scenario['goal'], launch_start, context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(collector)
    runtime_error = None
    try:
        deadline = launch_start + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if process.poll() is not None:
                raise RuntimeError(
                    f'launch exited before evaluation completed with code {process.returncode}')
            if collector.planning_success is False:
                raise RuntimeError('A* no path or planning failure')
            if collector.generation_success is False:
                raise RuntimeError('trajectory generation or dynamic constraint failure')
            if collector.health_errors:
                raise RuntimeError(collector.health_errors[0])
            if (collector.completion_time is not None and
                    time.monotonic() - collector.completion_time >= 3.0):
                break
        else:
            raise TimeoutError('execution did not complete before the scenario timeout')
    except Exception as error:
        runtime_error = str(error)
    finally:
        executor.remove_node(collector)
        executor.shutdown()
        collector.destroy_node()
        rclpy.shutdown(context=context)
        stop_launch(process)
        output.join()

    log_text = output.text()
    maximum_reference_speed, maximum_reference_acceleration = \
        validated_reference_extrema(log_text, collector)
    saturation_count = log_text.count('saturated=true')
    final_position_error = math.inf
    if collector.latest_position is not None:
        final_position_error = norm3(tuple(
            collector.latest_position[index] - scenario['goal'][index]
            for index in range(3)))
    metrics = {
        'scenario': scenario_name,
        'ros_domain_id': scenario['domain_id'],
        'start': list(START),
        'goal': list(scenario['goal']),
        'astar_config': str(config_path.relative_to(repo_root)),
        'planning_result_receive_time_s': collector.planning_result_receive_time,
        'planning_success': collector.planning_success is True,
        'raw_path_points': len(collector.raw_path),
        'raw_path_length_m': path_length(collector.raw_path),
        'simplified_path_points': len(collector.simplified_path),
        'simplified_path_length_m': path_length(collector.simplified_path),
        'trajectory_generation_success': collector.generation_success is True,
        'trajectory_total_duration_s': collector.trajectory_duration,
        'selected_velocity_scale': collector.selected_velocity_scale,
        'maximum_reference_speed_m_s': maximum_reference_speed,
        'maximum_reference_acceleration_m_s2': maximum_reference_acceleration,
        'task_completion_time_s': (
            collector.completion_time - launch_start
            if collector.completion_time is not None else None),
        'mission_complete': collector.completion_time is not None,
        'maximum_tracking_error_m': collector.maximum_tracking_error,
        'minimum_clearance_m': collector.minimum_clearance,
        'final_position_error_m': final_position_error,
        'final_speed_m_s': collector.latest_speed,
        'controller_saturated_true_count': saturation_count,
        'odom_samples': collector.odom_samples,
        'setpoint_samples': collector.setpoint_samples,
        'rpm_samples': collector.rpm_samples,
        'collision_state_samples': collector.collision_samples,
        'collision_observed': collector.collision_observed,
        'actual_point_collision_observed': collector.point_collision_observed,
        'actual_segment_collision_observed': collector.segment_collision_observed,
        'nonfinite_observed': collector.nonfinite_observed,
        'runtime_error': runtime_error,
    }
    if runtime_error is not None:
        collector._record_error(runtime_error)
    validate_metrics(metrics, collector)
    with (output_directory / 'metrics.json').open('w', encoding='utf-8') as metrics_file:
        json.dump(metrics, metrics_file, indent=2, sort_keys=True)
        metrics_file.write('\n')
    write_csv(output_directory, collector.rows)
    if collector.rows and collector.reference_path:
        write_plots(output_directory, collector)
    print(json.dumps(metrics, indent=2, sort_keys=True))
    return metrics


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Run reproducible static-avoidance scenarios and save metrics and plots.')
    parser.add_argument(
        '--scenario', action='append', choices=SCENARIOS,
        help='Run only the selected scenario; may be repeated. Defaults to all scenarios.')
    parser.add_argument(
        '--output-root', type=Path,
        help='Output directory; defaults to results/static_avoidance in the repository.')
    parser.add_argument('--timeout', type=float, default=90.0)
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    if arguments.timeout <= 0.0:
        raise ValueError('timeout must be positive')
    repo_root = Path(__file__).resolve().parents[1]
    output_root = arguments.output_root or repo_root / 'results' / 'static_avoidance'
    selected_scenarios = arguments.scenario or list(SCENARIOS)
    results = []
    for scenario_name in selected_scenarios:
        results.append(run_scenario(
            repo_root,
            scenario_name,
            SCENARIOS[scenario_name],
            output_root,
            arguments.timeout,
        ))
    failed = [result['scenario'] for result in results if not result['passed']]
    if failed:
        raise SystemExit(f'evaluation failed for: {", ".join(failed)}')
    print('\nAll selected static-avoidance evaluation scenarios passed.')


if __name__ == '__main__':
    main()
