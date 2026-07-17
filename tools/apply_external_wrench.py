#!/usr/bin/env python3

import argparse
import math
import os
import sys
import time


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Periodically publish a bounded map-frame force disturbance.')
    parser.add_argument('--force-x', type=float, default=0.3)
    parser.add_argument('--force-y', type=float, default=0.0)
    parser.add_argument('--force-z', type=float, default=0.0)
    parser.add_argument('--duration', type=float, default=2.0)
    parser.add_argument('--rate', type=float, default=20.0)
    parser.add_argument('--max-force', type=float, default=2.0)
    parser.add_argument('--ros-domain-id', type=int)
    return parser.parse_args()


def validate(arguments):
    values = (
        arguments.force_x, arguments.force_y, arguments.force_z,
        arguments.duration, arguments.rate, arguments.max_force)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('all numeric arguments must be finite')
    if arguments.duration <= 0.0 or arguments.rate <= 0.0 or arguments.max_force <= 0.0:
        raise ValueError('duration, rate, and max-force must be greater than zero')
    magnitude = math.sqrt(
        arguments.force_x ** 2 + arguments.force_y ** 2 + arguments.force_z ** 2)
    if magnitude > arguments.max_force + 1.0e-12:
        raise ValueError(
            f'force magnitude {magnitude:.3f} N exceeds {arguments.max_force:.3f} N')


def main():
    arguments = parse_arguments()
    try:
        validate(arguments)
    except ValueError as error:
        print(f'error: {error}', file=sys.stderr)
        return 2
    if arguments.ros_domain_id is not None:
        if arguments.ros_domain_id < 0 or arguments.ros_domain_id > 232:
            print('error: ros-domain-id must be in [0, 232]', file=sys.stderr)
            return 2
        os.environ['ROS_DOMAIN_ID'] = str(arguments.ros_domain_id)

    import rclpy
    from geometry_msgs.msg import WrenchStamped

    rclpy.init()
    node = rclpy.create_node('external_wrench_command_tool')
    publisher = node.create_publisher(WrenchStamped, '/drone/external_wrench', 10)

    def publish(fx, fy, fz):
        message = WrenchStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.wrench.force.x = fx
        message.wrench.force.y = fy
        message.wrench.force.z = fz
        publisher.publish(message)

    try:
        discovery_deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError('no /drone/external_wrench subscriber discovered')
        period = 1.0 / arguments.rate
        deadline = time.monotonic() + arguments.duration
        next_publish = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                publish(arguments.force_x, arguments.force_y, arguments.force_z)
                next_publish += period
            rclpy.spin_once(node, timeout_sec=min(0.02, max(0.0, next_publish - now)))
        return 0
    except (KeyboardInterrupt, RuntimeError) as error:
        if not isinstance(error, KeyboardInterrupt):
            print(f'error: {error}', file=sys.stderr)
            return 1
        return 130
    finally:
        publish(0.0, 0.0, 0.0)
        for _ in range(3):
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
