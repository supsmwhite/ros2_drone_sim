#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt


TARGET = (0.0, 0.0, 1.5)


def parse_arguments():
    parser = argparse.ArgumentParser(description='Evaluate hover recovery from a force pulse.')
    parser.add_argument('--timeout', type=float, default=45.0)
    parser.add_argument('--force-x', type=float, default=0.3)
    parser.add_argument('--force-y', type=float, default=0.0)
    parser.add_argument('--force-z', type=float, default=0.0)
    parser.add_argument('--duration', type=float, default=2.0)
    parser.add_argument('--rate', type=float, default=20.0)
    parser.add_argument('--ros-domain-id', type=int, default=119)
    parser.add_argument(
        '--output', type=Path,
        default=Path('results/hover_disturbance/default'))
    return parser.parse_args()


def validate_arguments(arguments):
    values = (
        arguments.timeout, arguments.force_x, arguments.force_y, arguments.force_z,
        arguments.duration, arguments.rate)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('numeric arguments must be finite')
    if arguments.timeout <= 0.0 or arguments.duration <= 0.0 or arguments.rate <= 0.0:
        raise ValueError('timeout, duration, and rate must be greater than zero')
    magnitude = math.sqrt(
        arguments.force_x ** 2 + arguments.force_y ** 2 + arguments.force_z ** 2)
    if magnitude > 2.0 + 1.0e-12:
        raise ValueError(f'disturbance magnitude {magnitude:.3f} N exceeds 2.0 N')
    if not 0 <= arguments.ros_domain_id <= 232:
        raise ValueError('ros-domain-id must be in [0, 232]')


def rotate_body_to_world(quaternion, vector):
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise ValueError('invalid quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def quaternion_to_euler(quaternion):
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise ValueError('invalid quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_argument = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_argument)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def save_csv(samples, path):
    fieldnames = [
        'time', 'phase', 'actual_x', 'actual_y', 'actual_z',
        'target_x', 'target_y', 'target_z', 'position_error', 'horizontal_error',
        'vx', 'vy', 'vz', 'speed', 'roll', 'pitch', 'yaw',
        'm1_rpm', 'm2_rpm', 'm3_rpm', 'm4_rpm',
        'external_fx', 'external_fy', 'external_fz', 'external_wrench_active',
    ]
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator='\n')
        writer.writeheader()
        writer.writerows(samples)


def shade_phases(axis, samples):
    colors = {
        'DISTURBANCE': '#ffcc80', 'RECOVERY': '#bbdefb', 'FINAL_HOLD': '#c8e6c9'}
    for phase, color in colors.items():
        times = [sample['time'] for sample in samples if sample['phase'] == phase]
        if times:
            axis.axvspan(min(times), max(times), color=color, alpha=0.25, label=phase)


def save_plots(samples, output):
    times = [sample['time'] for sample in samples]

    figure, axis = plt.subplots(figsize=(9, 4.5))
    axis.plot(times, [sample['position_error'] for sample in samples], label='3D error')
    axis.plot(times, [sample['horizontal_error'] for sample in samples], label='horizontal')
    axis.axhline(0.05, color='red', linestyle='--', label='recovery threshold')
    shade_phases(axis, samples)
    axis.set(xlabel='Time [s]', ylabel='Error [m]', title='Hover position error')
    axis.grid(True); axis.legend(ncol=3, fontsize=8); figure.tight_layout()
    figure.savefig(output / 'position_error.png', dpi=160); plt.close(figure)

    figure, axis = plt.subplots(figsize=(9, 4.5))
    for key, label in (('actual_x', 'x'), ('actual_y', 'y'), ('actual_z', 'z')):
        axis.plot(times, [sample[key] for sample in samples], label=label)
    axis.axhline(TARGET[2], color='black', linestyle=':', label='target z')
    shade_phases(axis, samples)
    axis.set(xlabel='Time [s]', ylabel='Position [m]', title='Position components')
    axis.grid(True); axis.legend(ncol=3, fontsize=8); figure.tight_layout()
    figure.savefig(output / 'position_xyz.png', dpi=160); plt.close(figure)

    figure, axis = plt.subplots(figsize=(9, 4.5))
    for key in ('vx', 'vy', 'vz', 'speed'):
        axis.plot(times, [sample[key] for sample in samples], label=key)
    axis.axhline(0.03, color='red', linestyle='--')
    shade_phases(axis, samples)
    axis.set(xlabel='Time [s]', ylabel='Velocity [m/s]', title='World velocity')
    axis.grid(True); axis.legend(ncol=4, fontsize=8); figure.tight_layout()
    figure.savefig(output / 'velocity.png', dpi=160); plt.close(figure)

    figure, axis = plt.subplots(figsize=(9, 4.5))
    for key in ('roll', 'pitch', 'yaw'):
        axis.plot(times, [sample[key] for sample in samples], label=key)
    shade_phases(axis, samples)
    axis.set(xlabel='Time [s]', ylabel='Angle [rad]', title='Attitude')
    axis.grid(True); axis.legend(fontsize=8); figure.tight_layout()
    figure.savefig(output / 'attitude.png', dpi=160); plt.close(figure)

    figure, axis = plt.subplots(figsize=(9, 4.5))
    for key in ('m1_rpm', 'm2_rpm', 'm3_rpm', 'm4_rpm'):
        axis.plot(times, [sample[key] for sample in samples], label=key)
    axis.axhline(20000.0, color='red', linestyle='--', label='limit')
    shade_phases(axis, samples)
    axis.set(xlabel='Time [s]', ylabel='Command [RPM]', title='Motor RPM commands')
    axis.grid(True); axis.legend(ncol=3, fontsize=8); figure.tight_layout()
    figure.savefig(output / 'motor_rpm.png', dpi=160); plt.close(figure)

    figure, axis = plt.subplots(figsize=(9, 4.5))
    for key, label in (('external_fx', 'Fx'), ('external_fy', 'Fy'), ('external_fz', 'Fz')):
        axis.plot(times, [sample[key] for sample in samples], label=label)
    shade_phases(axis, samples)
    axis.set(xlabel='Time [s]', ylabel='Force [N]', title='Applied external force')
    axis.grid(True); axis.legend(fontsize=8); figure.tight_layout()
    figure.savefig(output / 'external_force.png', dpi=160); plt.close(figure)

    figure, axes = plt.subplots(2, 2, figsize=(11, 7))
    axes[0, 0].plot(times, [sample['horizontal_error'] for sample in samples])
    axes[0, 0].set(title='Horizontal displacement', ylabel='m')
    axes[0, 1].plot(times, [sample['speed'] for sample in samples])
    axes[0, 1].set(title='Speed', ylabel='m/s')
    axes[1, 0].plot(times, [sample['pitch'] for sample in samples], label='pitch')
    axes[1, 0].plot(times, [sample['roll'] for sample in samples], label='roll')
    axes[1, 0].set(title='Recovery attitude', ylabel='rad'); axes[1, 0].legend()
    axes[1, 1].plot(times, [sample['external_fx'] for sample in samples], label='Fx')
    axes[1, 1].set(title='Disturbance pulse', ylabel='N')
    for axis in axes.flat:
        shade_phases(axis, samples); axis.grid(True); axis.set_xlabel('Time [s]')
    figure.suptitle('Hover disturbance summary'); figure.tight_layout()
    figure.savefig(output / 'disturbance_summary.png', dpi=160); plt.close(figure)


def stop_launch(process):
    if process.poll() is not None:
        return
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=3.0)


def main():
    arguments = parse_arguments()
    try:
        validate_arguments(arguments)
    except ValueError as error:
        print(f'error: {error}', file=sys.stderr)
        return 2
    os.environ['ROS_DOMAIN_ID'] = str(arguments.ros_domain_id)
    arguments.output.mkdir(parents=True, exist_ok=True)
    launch_log_path = arguments.output / 'launch.log'

    import rclpy
    from drone_msgs.msg import MotorRPM
    from geometry_msgs.msg import WrenchStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool

    launch_log = launch_log_path.open('w', encoding='utf-8')
    process = subprocess.Popen(
        ['ros2', 'launch', 'drone_bringup', 'disturbance_hover_sim.launch.py',
         'use_rviz:=false'], stdout=launch_log, stderr=subprocess.STDOUT,
        text=True, start_new_session=True, env=os.environ.copy())
    rclpy.init()
    node = rclpy.create_node('hover_disturbance_evaluator')
    publisher = node.create_publisher(WrenchStamped, '/drone/external_wrench', 10)
    latched_qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    phase = 'TAKEOFF'
    samples = []
    latest_rpm = (0.0, 0.0, 0.0, 0.0)
    latest_force = (0.0, 0.0, 0.0)
    latest_active = False
    health_errors = []
    start_time = time.monotonic()
    latest = {}

    def on_rpm(message):
        nonlocal latest_rpm
        latest_rpm = (
            message.m1_front_left_ccw_rpm, message.m2_rear_left_cw_rpm,
            message.m3_rear_right_ccw_rpm, message.m4_front_right_cw_rpm)
        if not all(math.isfinite(value) for value in latest_rpm):
            health_errors.append('non-finite motor command')

    def on_applied(message):
        nonlocal latest_force
        latest_force = (
            message.wrench.force.x, message.wrench.force.y, message.wrench.force.z)
        if not all(math.isfinite(value) for value in latest_force):
            health_errors.append('non-finite applied wrench')

    def on_active(message):
        nonlocal latest_active
        latest_active = message.data

    def on_odom(message):
        pose = message.pose.pose
        quaternion = (
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        position = (pose.position.x, pose.position.y, pose.position.z)
        try:
            velocity = rotate_body_to_world(
                quaternion,
                (message.twist.twist.linear.x, message.twist.twist.linear.y,
                 message.twist.twist.linear.z))
            roll, pitch, yaw = quaternion_to_euler(quaternion)
        except ValueError as error:
            health_errors.append(str(error)); return
        values = position + velocity + (roll, pitch, yaw) + latest_rpm + latest_force
        if not all(math.isfinite(value) for value in values):
            health_errors.append('non-finite flight sample'); return
        error = math.dist(position, TARGET)
        horizontal_error = math.hypot(position[0] - TARGET[0], position[1] - TARGET[1])
        speed = math.sqrt(sum(value * value for value in velocity))
        latest.update(position=position, velocity=velocity, speed=speed, error=error)
        samples.append({
            'time': time.monotonic() - start_time, 'phase': phase,
            'actual_x': position[0], 'actual_y': position[1], 'actual_z': position[2],
            'target_x': TARGET[0], 'target_y': TARGET[1], 'target_z': TARGET[2],
            'position_error': error, 'horizontal_error': horizontal_error,
            'vx': velocity[0], 'vy': velocity[1], 'vz': velocity[2], 'speed': speed,
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
            'm1_rpm': latest_rpm[0], 'm2_rpm': latest_rpm[1],
            'm3_rpm': latest_rpm[2], 'm4_rpm': latest_rpm[3],
            'external_fx': latest_force[0], 'external_fy': latest_force[1],
            'external_fz': latest_force[2],
            'external_wrench_active': int(latest_active),
        })

    subscriptions = [
        node.create_subscription(Odometry, '/drone/odom', on_odom, 30),
        node.create_subscription(MotorRPM, '/drone/motor_rpm_cmd', on_rpm, 30),
        node.create_subscription(
            WrenchStamped, '/drone/external_wrench/applied', on_applied, latched_qos),
        node.create_subscription(
            Bool, '/drone/external_wrench/active', on_active, latched_qos),
    ]

    def spin(duration=0.02):
        rclpy.spin_once(node, timeout_sec=duration)
        if process.poll() is not None:
            raise RuntimeError(f'launch exited early with code {process.returncode}')
        if health_errors:
            raise RuntimeError(health_errors[0])
        if time.monotonic() - start_time > arguments.timeout:
            raise TimeoutError('overall evaluation timeout')

    def publish_force(force):
        message = WrenchStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = force
        publisher.publish(message)

    recovery_time = None
    try:
        discovery_deadline = time.monotonic() + 8.0
        while time.monotonic() < discovery_deadline:
            spin()
            if 'position' in latest and publisher.get_subscription_count() > 0:
                break
        else:
            raise TimeoutError('ROS graph discovery timeout')

        stable_since = None
        while True:
            spin()
            if latest.get('error', math.inf) < 0.03 and latest.get('speed', math.inf) < 0.02:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 1.0:
                    break
            else:
                stable_since = None

        phase = 'BASELINE'
        baseline_start = time.monotonic()
        while time.monotonic() - baseline_start < 2.0:
            spin()

        phase = 'DISTURBANCE'
        force = (arguments.force_x, arguments.force_y, arguments.force_z)
        disturbance_start = time.monotonic()
        next_publish = disturbance_start
        while time.monotonic() - disturbance_start < arguments.duration:
            now = time.monotonic()
            if now >= next_publish:
                publish_force(force)
                next_publish += 1.0 / arguments.rate
            spin(min(0.02, max(0.0, next_publish - now)))
        publish_force((0.0, 0.0, 0.0))

        phase = 'RECOVERY'
        recovery_start = time.monotonic()
        recovered_since = None
        while time.monotonic() - recovery_start < 12.0:
            spin()
            if latest.get('error', math.inf) < 0.05 and latest.get('speed', math.inf) < 0.03:
                recovered_since = recovered_since or time.monotonic()
                if time.monotonic() - recovered_since >= 1.0:
                    recovery_time = recovered_since - recovery_start
                    break
            else:
                recovered_since = None

        phase = 'FINAL_HOLD'
        final_start = time.monotonic()
        while time.monotonic() - final_start < 2.0:
            spin()
    except (KeyboardInterrupt, RuntimeError, TimeoutError) as error:
        health_errors.append(str(error))
    finally:
        publish_force((0.0, 0.0, 0.0))
        for _ in range(3):
            rclpy.spin_once(node, timeout_sec=0.02)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_publisher(publisher)
        node.destroy_node()
        rclpy.shutdown()
        stop_launch(process)
        launch_log.close()

    if not samples:
        print(f'error: no samples collected: {health_errors}', file=sys.stderr)
        return 1
    launch_text = launch_log_path.read_text(encoding='utf-8', errors='replace')
    baseline = [sample for sample in samples if sample['phase'] == 'BASELINE']
    evaluated = [
        sample for sample in samples
        if sample['phase'] in ('DISTURBANCE', 'RECOVERY', 'FINAL_HOLD')]
    final_hold = [sample for sample in samples if sample['phase'] == 'FINAL_HOLD']
    final_sample = final_hold[-1] if final_hold else samples[-1]
    motor_values = [
        sample[key] for sample in baseline + evaluated
        for key in ('m1_rpm', 'm2_rpm', 'm3_rpm', 'm4_rpm')]
    active_motor_values = [value for value in motor_values if value > 1.0e-6]
    saturation_count = launch_text.count('saturated=true')
    force_magnitude = math.sqrt(
        arguments.force_x ** 2 + arguments.force_y ** 2 + arguments.force_z ** 2)
    maximum_horizontal = max(sample['horizontal_error'] for sample in evaluated)
    nonfinite = bool(health_errors) or any(
        not all(math.isfinite(value) for key, value in sample.items()
                if key not in ('phase',)) for sample in samples)
    attitude_divergence = any(
        abs(sample['roll']) > 1.0 or abs(sample['pitch']) > 1.0 for sample in evaluated)
    passed = (
        not nonfinite and not attitude_divergence and recovery_time is not None and
        recovery_time < 10.0 and maximum_horizontal < 0.50 and
        maximum_horizontal > 0.02 and max(motor_values) < 20000.0 and
        saturation_count == 0 and final_sample['position_error'] < 0.05 and
        final_sample['speed'] < 0.03)
    metrics = {
        'target_position': list(TARGET),
        'disturbance_force_n': {
            'vector': [arguments.force_x, arguments.force_y, arguments.force_z],
            'magnitude': force_magnitude},
        'disturbance_duration_s': arguments.duration,
        'baseline_position_error_m': sum(
            sample['position_error'] for sample in baseline) / len(baseline),
        'maximum_position_error_m': max(sample['position_error'] for sample in evaluated),
        'maximum_horizontal_displacement_m': maximum_horizontal,
        'maximum_vertical_error_m': max(
            abs(sample['actual_z'] - TARGET[2]) for sample in evaluated),
        'maximum_speed_m_s': max(sample['speed'] for sample in evaluated),
        'maximum_roll_rad': max(abs(sample['roll']) for sample in evaluated),
        'maximum_pitch_rad': max(abs(sample['pitch']) for sample in evaluated),
        'maximum_motor_rpm': max(motor_values),
        'minimum_active_motor_rpm': min(active_motor_values),
        'controller_saturation_count': saturation_count,
        'recovery_time_s': recovery_time,
        'final_position_error_m': final_sample['position_error'],
        'final_speed_m_s': final_sample['speed'],
        'nonfinite_observed': nonfinite,
        'attitude_divergence_observed': attitude_divergence,
        'passed': passed,
        'sample_count': len(samples),
        'health_errors': health_errors,
        'recovery_definition': 'first continuous 1.0 s with error <0.05 m and speed <0.03 m/s',
    }
    save_csv(samples, arguments.output / 'trajectory.csv')
    save_plots(samples, arguments.output)
    with (arguments.output / 'metrics.json').open('w', encoding='utf-8') as stream:
        json.dump(metrics, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    print(json.dumps(metrics, indent=2, ensure_ascii=False))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
