#!/usr/bin/env python3

import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '119'

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import WrenchStamped
import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    dynamics_config = os.path.join(
        get_package_share_directory('drone_bringup'), 'config', 'dynamics.yaml')
    enabled = Node(
        package='drone_dynamics', executable='quadrotor_dynamics_node',
        name='quadrotor_dynamics_node', output='screen',
        parameters=[dynamics_config, {'enable_external_wrench': True}],
    )
    disabled = Node(
        package='drone_dynamics', executable='quadrotor_dynamics_node',
        name='disabled_quadrotor_dynamics_node', output='screen',
        parameters=[{'enable_ground_contact': True}],
        remappings=[
            ('/drone/motor_rpm_cmd', '/disabled/motor_rpm_cmd'),
            ('/drone/odom', '/disabled/odom'),
            ('/drone/imu', '/disabled/imu'),
            ('/drone/path', '/disabled/path'),
            ('/drone/external_wrench', '/disabled/external_wrench'),
            ('/drone/external_wrench/active', '/disabled/external_wrench/active'),
            ('/drone/external_wrench/applied', '/disabled/external_wrench/applied'),
            ('/tf', '/disabled/tf'),
        ],
    )
    return launch.LaunchDescription([
        enabled, disabled, launch_testing.actions.ReadyToTest(),
    ])


class TestExternalWrenchNode(unittest.TestCase):

    def test_validation_application_timeout_and_default_disabled(self):
        rclpy.init()
        node = rclpy.create_node('external_wrench_node_test')
        latched_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        latest = {}

        def applied_callback(message):
            latest['applied'] = (
                message.wrench.force.x,
                message.wrench.force.y,
                message.wrench.force.z)

        subscriptions = [
            node.create_subscription(
                Bool, '/drone/external_wrench/active',
                lambda message: latest.__setitem__('active', message.data), latched_qos),
            node.create_subscription(
                WrenchStamped, '/drone/external_wrench/applied',
                applied_callback, latched_qos),
            node.create_subscription(
                Odometry, '/disabled/odom',
                lambda message: latest.__setitem__(
                    'disabled_position', (
                        message.pose.pose.position.x,
                        message.pose.pose.position.y,
                        message.pose.pose.position.z)), 10),
        ]
        publisher = node.create_publisher(WrenchStamped, '/drone/external_wrench', 10)
        disabled_publisher = node.create_publisher(
            WrenchStamped, '/disabled/external_wrench', 10)

        def spin_until(predicate, timeout, description):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                if predicate():
                    return
            self.fail(f'timed out waiting for {description}; latest={latest}')

        def message(frame='map', force=(0.0, 0.0, 0.0)):
            wrench = WrenchStamped()
            wrench.header.frame_id = frame
            wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z = force
            return wrench

        try:
            spin_until(
                lambda: latest.get('active') is False and
                latest.get('applied') == (0.0, 0.0, 0.0) and
                'disabled_position' in latest,
                5.0, 'initial inactive status')
            spin_until(lambda: publisher.get_subscription_count() == 1, 3.0, 'enabled subscriber')
            self.assertEqual(disabled_publisher.get_subscription_count(), 0)

            publisher.publish(message(force=(0.8, 0.0, 0.0)))
            spin_until(
                lambda: latest.get('active') is True and
                latest.get('applied') == (0.8, 0.0, 0.0),
                2.0, 'valid force application')

            publisher.publish(message(frame='base_link', force=(0.2, 0.0, 0.0)))
            end = time.monotonic() + 0.05
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertEqual(latest['applied'], (0.8, 0.0, 0.0))

            spin_until(
                lambda: latest.get('active') is False and
                latest.get('applied') == (0.0, 0.0, 0.0),
                2.0, 'wrench timeout clear')

            publisher.publish(message(force=(2.01, 0.0, 0.0)))
            publisher.publish(message(force=(math.nan, 0.0, 0.0)))
            nonzero_torque = message()
            nonzero_torque.wrench.torque.z = 0.01
            publisher.publish(nonzero_torque)
            end = time.monotonic() + 0.15
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertFalse(latest['active'])
            self.assertEqual(latest['applied'], (0.0, 0.0, 0.0))

            disabled_publisher.publish(message(force=(0.8, 0.0, 0.0)))
            start_position = latest['disabled_position']
            end = time.monotonic() + 0.3
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
            final_position = latest['disabled_position']
            self.assertLess(math.dist(start_position, final_position), 1.0e-9)
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_publisher(publisher)
            node.destroy_publisher(disabled_publisher)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestExternalWrenchShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
