#!/usr/bin/env python3

import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '132'

from ament_index_python.packages import get_package_share_directory
from drone_msgs.msg import ControllerDiagnostics, MotorRPM
from drone_msgs.srv import ExecuteGoalSequence
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt32


GOALS = (
    (0.0, 0.0, 1.5, 0.0),
    (2.0, 0.0, 1.5, math.pi / 2.0),
    (2.0, 2.0, 1.5, math.pi),
    (0.0, 2.0, 1.5, -math.pi / 2.0),
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('drone_bringup'), 'launch',
        'assessment_basic_sim.launch.py')
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'use_rviz': 'false'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def make_pose(values):
    x, y, z, yaw = values
    result = Pose()
    result.position.x, result.position.y, result.position.z = x, y, z
    result.orientation.z = math.sin(0.5 * yaw)
    result.orientation.w = math.cos(0.5 * yaw)
    return result


class TestAssessmentBasicMulti(unittest.TestCase):

    def test_runtime_square_mission(self, proc_output):
        rclpy.init()
        node = rclpy.create_node('assessment_basic_multi_e2e_test')
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        latest = {}
        indices = []
        health_errors = []
        consecutive_saturation_samples = 0
        maximum_consecutive_saturation_samples = 0

        def on_odom(message):
            values = (
                message.pose.pose.position.x, message.pose.pose.position.y,
                message.pose.pose.position.z, message.twist.twist.linear.x,
                message.twist.twist.linear.y, message.twist.twist.linear.z)
            if not all(math.isfinite(value) for value in values):
                health_errors.append('non-finite Odom')
                return
            latest['position'] = values[:3]
            latest['speed'] = math.sqrt(sum(value * value for value in values[3:]))

        def on_rpm(message):
            values = (
                message.m1_front_left_ccw_rpm, message.m2_rear_left_cw_rpm,
                message.m3_rear_right_ccw_rpm, message.m4_front_right_cw_rpm)
            if not all(math.isfinite(value) and 0.0 <= value <= 20000.0
                       for value in values):
                health_errors.append('invalid RPM')

        def on_diagnostics(message):
            nonlocal consecutive_saturation_samples
            nonlocal maximum_consecutive_saturation_samples
            if (message.horizontal_saturated or message.altitude_saturated or
                    message.attitude_saturated or message.mixer_saturated):
                consecutive_saturation_samples += 1
                maximum_consecutive_saturation_samples = max(
                    maximum_consecutive_saturation_samples,
                    consecutive_saturation_samples)
            else:
                consecutive_saturation_samples = 0

        def on_index(message):
            if not indices or indices[-1] != message.data:
                indices.append(message.data)

        subscriptions = [
            node.create_subscription(Odometry, '/drone/odom', on_odom, 20),
            node.create_subscription(MotorRPM, '/drone/motor_rpm_cmd', on_rpm, 20),
            node.create_subscription(
                ControllerDiagnostics, '/drone/controller/diagnostics',
                on_diagnostics, 20),
            node.create_subscription(
                UInt32, '/drone/mission/current_waypoint_index', on_index, latched),
            node.create_subscription(
                Bool, '/drone/mission/complete',
                lambda msg: latest.__setitem__('complete', msg.data), latched),
            node.create_subscription(
                PoseArray, '/drone/mission/goals',
                lambda msg: latest.__setitem__('goals', msg), latched),
            node.create_subscription(
                PoseStamped, '/drone/goal',
                lambda msg: latest.__setitem__('goal', msg), 20),
        ]
        client = node.create_client(ExecuteGoalSequence, '/drone/mission/execute')

        def spin_until(predicate, timeout, description):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                if health_errors:
                    self.fail(health_errors[0])
                if predicate():
                    return
            self.fail(f'timed out waiting for {description}; latest={latest}')

        try:
            self.assertTrue(client.wait_for_service(timeout_sec=8.0))
            spin_until(lambda: 'position' in latest, 5.0, 'initial Odom')
            request = ExecuteGoalSequence.Request()
            request.goals.header.frame_id = 'map'
            request.goals.poses = [make_pose(goal) for goal in GOALS]
            request.draft_revision = 1
            future = client.call_async(request)
            spin_until(future.done, 5.0, 'mission response')
            self.assertTrue(future.result().accepted, future.result().message)
            spin_until(lambda: latest.get('complete') is True, 85.0, 'square mission')
            settle_deadline = time.monotonic() + 2.0
            while time.monotonic() < settle_deadline:
                rclpy.spin_once(node, timeout_sec=0.02)

            self.assertEqual(indices, [0, 1, 2, 3])
            self.assertEqual(len(latest['goals'].poses), 4)
            for actual, expected in zip(latest['goals'].poses, GOALS):
                yaw = math.atan2(
                    2.0 * actual.orientation.w * actual.orientation.z,
                    1.0 - 2.0 * actual.orientation.z * actual.orientation.z)
                self.assertLess(abs(math.remainder(yaw - expected[3], 2.0 * math.pi)), 1e-6)
            final_error = math.dist(latest['position'], GOALS[-1][:3])
            self.assertLess(final_error, 0.10)
            self.assertLess(latest['speed'], 0.08)
            self.assertLess(maximum_consecutive_saturation_samples, 200)
            self.assertEqual(consecutive_saturation_samples, 0)
            self.assertEqual(node.get_node_names().count('waypoint_manager_node'), 1)
            self.assertNotIn('trajectory_mission_node', node.get_node_names())
            output = b''.join(event.text for event in proc_output)
            self.assertNotIn(b'collision', output.lower())
            print(
                'assessment_basic_multi_e2e: '
                f'indices={indices} final_error={final_error:.6f} '
                f'final_speed={latest["speed"]:.6f} '
                f'max_consecutive_saturation_samples='
                f'{maximum_consecutive_saturation_samples}', flush=True)
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_client(client)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestAssessmentBasicMultiShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
