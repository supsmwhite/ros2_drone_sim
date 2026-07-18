#!/usr/bin/env python3
# flake8: noqa: E402

import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '121'

from drone_msgs.msg import ControllerDiagnostics
from geometry_msgs.msg import PoseStamped, WrenchStamped
import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    demo = Node(
        package='drone_bringup', executable='disturbance_demo_node',
        name='disturbance_demo_node', output='screen', parameters=[{
            'profile': 'short_gust',
            'target_x': 0.0,
            'target_y': 0.0,
            'target_z': 1.5,
            'start_delay': 0.20,
            'force_x': 0.30,
            'force_y': 0.0,
            'force_z': 0.0,
            'disturbance_duration': 0.60,
            'recovery_duration': 0.40,
            'force_publish_rate': 50.0,
            'force_arrow_scale': 1.5,
            'integral_arrow_scale': 1.5,
            'show_integral_arrow': True,
            'show_status_text': True,
            'settle_duration': 0.20,
        }])
    return launch.LaunchDescription([
        demo,
        launch_testing.actions.ReadyToTest(),
    ])


class TestDisturbanceDemoNode(unittest.TestCase):

    def test_stage_sequence_wrench_and_markers(self):
        rclpy.init()
        node = rclpy.create_node('disturbance_demo_node_test')
        odom_publisher = node.create_publisher(Odometry, '/drone/odom', 10)
        diagnostics_publisher = node.create_publisher(
            ControllerDiagnostics, '/drone/controller/diagnostics', 10)
        wrench_messages = []
        marker_messages = []
        goal_messages = []
        marker_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = [
            node.create_subscription(
                WrenchStamped, '/drone/external_wrench', wrench_messages.append, 20),
            node.create_subscription(
                MarkerArray, '/drone/disturbance/markers', marker_messages.append, marker_qos),
            node.create_subscription(PoseStamped, '/drone/goal', goal_messages.append, 20),
        ]

        def spin_for(duration, publish_settled_odom=False, saturated=False):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                if publish_settled_odom:
                    odometry = Odometry()
                    odometry.header.stamp = node.get_clock().now().to_msg()
                    odometry.header.frame_id = 'map'
                    odometry.child_frame_id = 'base_link'
                    odometry.pose.pose.position.z = 1.5
                    odometry.pose.pose.orientation.w = 1.0
                    odom_publisher.publish(odometry)

                    diagnostics = ControllerDiagnostics()
                    diagnostics.horizontal_i_acceleration_x = -0.10
                    diagnostics.horizontal_i_acceleration_y = 0.02
                    diagnostics.horizontal_saturated = saturated
                    diagnostics.mixer_saturated = saturated
                    diagnostics_publisher.publish(diagnostics)
                rclpy.spin_once(node, timeout_sec=0.01)

        def force_tuple(message):
            return (message.wrench.force.x, message.wrench.force.y,
                    message.wrench.force.z)

        def marker_texts():
            return [marker.text for array in marker_messages for marker in array.markers
                    if marker.type == Marker.TEXT_VIEW_FACING and marker.action == Marker.ADD]

        def marker_values_are_finite(marker):
            values = [
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z,
                marker.pose.orientation.x, marker.pose.orientation.y,
                marker.pose.orientation.z, marker.pose.orientation.w,
                marker.scale.x, marker.scale.y, marker.scale.z,
            ]
            for point in marker.points:
                values.extend([point.x, point.y, point.z])
            return all(math.isfinite(value) for value in values)

        try:
            spin_for(0.30)
            self.assertTrue(wrench_messages)
            self.assertTrue(all(force_tuple(message) == (0.0, 0.0, 0.0)
                                for message in wrench_messages))
            self.assertTrue(goal_messages)
            self.assertAlmostEqual(goal_messages[-1].pose.position.z, 1.5)
            self.assertTrue(any(text == 'TAKEOFF / SETTLING' for text in marker_texts()))
            self.assertTrue(all(
                marker_values_are_finite(marker)
                for array in marker_messages for marker in array.markers))

            wrench_messages.clear()
            marker_messages.clear()
            spin_for(0.65, publish_settled_odom=True)
            self.assertTrue(any(force_tuple(message) == (0.30, 0.0, 0.0)
                                for message in wrench_messages))
            active_arrays = [
                array for array in marker_messages
                if any('GUST ACTIVE' in marker.text for marker in array.markers)]
            self.assertTrue(active_arrays)
            self.assertTrue(any(
                marker.ns == 'equivalent_external_force' and
                marker.action == Marker.ADD and len(marker.points) == 2 and
                marker.points[1].x > marker.points[0].x
                for array in active_arrays for marker in array.markers))
            self.assertTrue(any(
                marker.ns == 'horizontal_integral_acceleration' and
                marker.action == Marker.ADD and len(marker.points) == 2 and
                marker.points[1].x < marker.points[0].x
                for array in active_arrays for marker in array.markers))
            self.assertTrue(any(
                'F=[0.30, 0.00, 0.00] N' in text
                for text in marker_texts()))
            normal_active_texts = [text for text in marker_texts() if 'GUST ACTIVE' in text]
            self.assertTrue(normal_active_texts)
            countdown_texts = [text for text in marker_texts() if text.startswith('GUST IN ')]
            self.assertTrue(countdown_texts)
            self.assertTrue(all(len(text.splitlines()) == 1 for text in countdown_texts))
            self.assertTrue(all(len(text.splitlines()) <= 3 for text in normal_active_texts))
            self.assertTrue(all('SATURATION' not in text for text in normal_active_texts))
            self.assertTrue(all('DISTURBANCE_ACTIVE' not in text for text in normal_active_texts))
            self.assertTrue(all(
                marker_values_are_finite(marker)
                for array in marker_messages for marker in array.markers))

            marker_messages.clear()
            spin_for(0.06, publish_settled_odom=True, saturated=True)
            warning_texts = [text for text in marker_texts() if 'WARNING:' in text]
            self.assertTrue(warning_texts)
            self.assertTrue(any(
                'H SATURATION | MIXER SATURATION' in text for text in warning_texts))
            self.assertTrue(all(len(text.splitlines()) <= 4 for text in warning_texts))

            wrench_messages.clear()
            marker_messages.clear()
            spin_for(0.85, publish_settled_odom=True)
            self.assertTrue(wrench_messages)
            self.assertTrue(all(force_tuple(message) == (0.0, 0.0, 0.0)
                                for message in wrench_messages[-5:]))
            self.assertTrue(any(
                'RECOVERY' in text or 'COMPLETE' in text for text in marker_texts()))
            recovery_texts = [text for text in marker_texts() if text.startswith('RECOVERY')]
            complete_texts = [text for text in marker_texts() if text.startswith('COMPLETE')]
            self.assertTrue(recovery_texts)
            self.assertTrue(complete_texts)
            self.assertTrue(all(len(text.splitlines()) <= 2 for text in recovery_texts))
            self.assertTrue(all(len(text.splitlines()) == 1 for text in complete_texts))
            self.assertTrue(any(
                marker.ns == 'equivalent_external_force' and marker.action == Marker.DELETE
                for array in marker_messages for marker in array.markers))
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_publisher(odom_publisher)
            node.destroy_publisher(diagnostics_publisher)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestDisturbanceDemoShutdown(unittest.TestCase):

    def test_process_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
