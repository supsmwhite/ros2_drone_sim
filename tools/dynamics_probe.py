#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from drone_msgs.msg import MotorRPM


class DynamicsProbe(Node):
    def __init__(self, motor_rpm, settle_time, sample_time):
        super().__init__('dynamics_probe')
        self._motor_rpm = motor_rpm
        self._settle_time = settle_time
        self._sample_time = sample_time
        self._command_start_time = None
        self._first_odometry = None
        self._last_odometry = None
        self._sample_count = 0

        self._publisher = self.create_publisher(
            MotorRPM, '/drone/motor_rpm_cmd', 10)
        self._subscription = self.create_subscription(
            Odometry, '/drone/odom', self._odometry_callback, 10)
        self._timer = self.create_timer(0.01, self._publish_command)

    def _publish_command(self):
        message = MotorRPM()
        message.m1_front_left_ccw_rpm = self._motor_rpm[0]
        message.m2_rear_left_cw_rpm = self._motor_rpm[1]
        message.m3_rear_right_ccw_rpm = self._motor_rpm[2]
        message.m4_front_right_cw_rpm = self._motor_rpm[3]
        self._publisher.publish(message)
        if self._command_start_time is None and self._publisher.get_subscription_count() > 0:
            self._command_start_time = time.monotonic()

    def _odometry_callback(self, message):
        if self._command_start_time is None:
            return
        elapsed = time.monotonic() - self._command_start_time
        if elapsed < self._settle_time:
            return
        if self._first_odometry is None:
            self._first_odometry = message
        self._last_odometry = message
        self._sample_count += 1

    def complete(self):
        if self._command_start_time is None:
            return False
        elapsed = time.monotonic() - self._command_start_time
        return elapsed >= self._settle_time + self._sample_time

    def result(self):
        if self._first_odometry is None or self._last_odometry is None:
            raise RuntimeError('No odometry samples were received')

        first = self._first_odometry
        last = self._last_odometry
        orientation = last.pose.pose.orientation
        quaternion_norm = math.sqrt(
            orientation.x ** 2 + orientation.y ** 2 +
            orientation.z ** 2 + orientation.w ** 2)

        return {
            'samples': self._sample_count,
            'delta_position_z_m': (
                last.pose.pose.position.z - first.pose.pose.position.z),
            'delta_linear_velocity_z_m_s': (
                last.twist.twist.linear.z - first.twist.twist.linear.z),
            'angular_velocity_x_rad_s': last.twist.twist.angular.x,
            'angular_velocity_y_rad_s': last.twist.twist.angular.y,
            'angular_velocity_z_rad_s': last.twist.twist.angular.z,
            'quaternion_norm': quaternion_norm,
        }


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Publish four motor RPM commands and summarize odometry response.')
    parser.add_argument('m1', type=float)
    parser.add_argument('m2', type=float)
    parser.add_argument('m3', type=float)
    parser.add_argument('m4', type=float)
    parser.add_argument('--settle', type=float, default=0.30)
    parser.add_argument('--duration', type=float, default=0.30)
    parser.add_argument('--timeout', type=float, default=5.0)
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    if arguments.settle < 0.0 or arguments.duration <= 0.0:
        raise ValueError('settle must be non-negative and duration must be positive')

    rclpy.init()
    probe = DynamicsProbe(
        [arguments.m1, arguments.m2, arguments.m3, arguments.m4],
        arguments.settle,
        arguments.duration,
    )
    deadline = time.monotonic() + arguments.timeout
    try:
        while rclpy.ok() and not probe.complete():
            if time.monotonic() >= deadline:
                raise TimeoutError('Timed out while waiting for the dynamics response')
            rclpy.spin_once(probe, timeout_sec=0.05)

        result = probe.result()
        print(' '.join(f'{key}={value}' for key, value in result.items()))
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
