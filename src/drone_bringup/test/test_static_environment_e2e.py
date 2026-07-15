#!/usr/bin/env python3

import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '96'

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
from visualization_msgs.msg import Marker, MarkerArray


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    environment_node = Node(
        package='drone_planning',
        executable='static_environment_node',
        name='static_environment_node',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'workspace': [-2.0, 2.0, -2.0, 2.0, -1.0, 3.0],
            'obstacles': [0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            'safety_radius': 0.25,
            'odometry_timeout': 0.25,
        }],
    )
    return launch.LaunchDescription([
        environment_node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestStaticEnvironmentEndToEnd(unittest.TestCase):

    def test_markers_and_collision_state(self):
        rclpy.init()
        node = rclpy.create_node('static_environment_e2e_test')
        odometry_publisher = node.create_publisher(
            Odometry, '/drone/odom', 10)
        collision_events = []
        latest_markers = None

        def on_collision(message):
            collision_events.append((time.monotonic(), bool(message.data)))

        def on_markers(message):
            nonlocal latest_markers
            latest_markers = message

        collision_subscription = node.create_subscription(
            Bool, '/drone/environment/in_collision', on_collision, 10)
        marker_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        marker_subscription = node.create_subscription(
            MarkerArray, '/drone/environment/markers', on_markers, marker_qos)

        def odometry(x, y, z):
            message = Odometry()
            message.header.frame_id = 'map'
            message.child_frame_id = 'base_link'
            message.pose.pose.position.x = x
            message.pose.pose.position.y = y
            message.pose.pose.position.z = z
            message.pose.pose.orientation.w = 1.0
            return message

        def publish_and_assert_state(message, expected):
            first_new_event = len(collision_events)
            deadline = time.monotonic() + 0.25
            while time.monotonic() < deadline:
                odometry_publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.03)
            new_states = [value for _, value in collision_events[first_new_event:]]
            self.assertTrue(new_states, 'no collision state received for published Odom')
            self.assertEqual(
                new_states[-1], expected,
                f'unexpected collision state history: {new_states}')

        try:
            discovery_deadline = time.monotonic() + 5.0
            while time.monotonic() < discovery_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if ('static_environment_node' in node.get_node_names() and
                        odometry_publisher.get_subscription_count() > 0 and
                        latest_markers is not None):
                    break
            else:
                self.fail(
                    f'environment graph not ready; nodes={node.get_node_names()}, '
                    f'markers={latest_markers}')

            self.assertEqual(len(latest_markers.markers), 3)
            self.assertTrue(all(
                marker.header.frame_id == 'map'
                for marker in latest_markers.markers))
            namespaces = {marker.ns for marker in latest_markers.markers}
            self.assertEqual(
                namespaces, {'workspace', 'obstacles', 'inflated_obstacles'})
            marker_types = {marker.ns: marker.type for marker in latest_markers.markers}
            self.assertEqual(marker_types['workspace'], Marker.LINE_LIST)
            self.assertEqual(marker_types['obstacles'], Marker.CUBE)
            self.assertEqual(marker_types['inflated_obstacles'], Marker.CUBE)

            publish_and_assert_state(odometry(-1.0, -1.0, 0.5), False)
            publish_and_assert_state(odometry(0.0, 0.0, 1.0), True)
            publish_and_assert_state(odometry(0.70, 0.0, 1.0), True)
            publish_and_assert_state(odometry(1.90, 0.0, 1.0), True)
            publish_and_assert_state(odometry(-1.0, -1.0, 0.5), False)

            invalid = odometry(math.nan, -1.0, 0.5)
            invalid_settle_deadline = time.monotonic() + 0.30
            while time.monotonic() < invalid_settle_deadline:
                odometry_publisher.publish(invalid)
                rclpy.spin_once(node, timeout_sec=0.03)
            event_count_after_invalid = len(collision_events)
            quiet_deadline = time.monotonic() + 0.35
            while time.monotonic() < quiet_deadline:
                rclpy.spin_once(node, timeout_sec=0.03)
            self.assertEqual(
                len(collision_events), event_count_after_invalid,
                'invalid Odom must suppress collision state publication')

            publish_and_assert_state(odometry(-1.0, -1.0, 0.5), False)
            self.assertIn(True, [value for _, value in collision_events])
            self.assertFalse(collision_events[-1][1])
        finally:
            node.destroy_subscription(collision_subscription)
            node.destroy_subscription(marker_subscription)
            node.destroy_publisher(odometry_publisher)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestStaticEnvironmentShutdown(unittest.TestCase):

    def test_process_exits_cleanly(self, proc_info):
        self.assertTrue(any(
            'static_environment_node' in name
            for name in proc_info.process_names()))
        launch_testing.asserts.assertExitCodes(proc_info)
