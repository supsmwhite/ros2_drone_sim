#!/usr/bin/env python3

import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '131'

from ament_index_python.packages import get_package_share_directory
from drone_msgs.msg import ControllerDiagnostics, MotorRPM
from drone_msgs.srv import ExecuteGoalSequence
from geometry_msgs.msg import Pose
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.markers
from nav_msgs.msg import Odometry, Path
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


TARGETS = ((0.0, 0.0, 1.5), (2.0, 1.0, 1.5))


def marker_labels(message):
    return {
        marker.text.splitlines()[0] for marker in message.markers
        if marker.type == Marker.TEXT_VIEW_FACING
    }


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


class TestAssessmentBasicSingle(unittest.TestCase):

    def test_hover_then_single_goal(self):
        rclpy.init()
        node = rclpy.create_node('assessment_basic_single_e2e_test')
        latest = {}
        health_errors = []
        rpm_samples = 0
        consecutive_saturation_samples = 0
        maximum_consecutive_saturation_samples = 0
        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        def on_odom(message):
            values = (
                message.pose.pose.position.x, message.pose.pose.position.y,
                message.pose.pose.position.z, message.pose.pose.orientation.x,
                message.pose.pose.orientation.y, message.pose.pose.orientation.z,
                message.pose.pose.orientation.w, message.twist.twist.linear.x,
                message.twist.twist.linear.y, message.twist.twist.linear.z)
            if not all(math.isfinite(value) for value in values):
                health_errors.append('non-finite Odom')
                return
            latest['position'] = values[:3]
            latest['speed'] = math.sqrt(sum(value * value for value in values[7:10]))

        def on_rpm(message):
            nonlocal rpm_samples
            values = (
                message.m1_front_left_ccw_rpm, message.m2_rear_left_cw_rpm,
                message.m3_rear_right_ccw_rpm, message.m4_front_right_cw_rpm)
            rpm_samples += 1
            if not all(math.isfinite(value) and 0.0 <= value <= 20000.0
                       for value in values):
                health_errors.append('invalid RPM')

        def on_diagnostics(message):
            nonlocal consecutive_saturation_samples
            nonlocal maximum_consecutive_saturation_samples
            values = tuple(message.motor_rpm) + (
                message.horizontal_acceleration_x,
                message.horizontal_acceleration_y, message.collective_thrust)
            if not all(math.isfinite(value) for value in values):
                health_errors.append('non-finite controller output')
            if (message.horizontal_saturated or message.altitude_saturated or
                    message.attitude_saturated or message.mixer_saturated):
                consecutive_saturation_samples += 1
                maximum_consecutive_saturation_samples = max(
                    maximum_consecutive_saturation_samples,
                    consecutive_saturation_samples)
            else:
                consecutive_saturation_samples = 0

        subscriptions = [
            node.create_subscription(Odometry, '/drone/odom', on_odom, 20),
            node.create_subscription(MotorRPM, '/drone/motor_rpm_cmd', on_rpm, 20),
            node.create_subscription(
                ControllerDiagnostics, '/drone/controller/diagnostics',
                on_diagnostics, 20),
            node.create_subscription(
                Path, '/drone/path', lambda msg: latest.__setitem__('path', msg), 10),
            node.create_subscription(
                MarkerArray, '/drone/mission/goal_markers',
                lambda msg: latest.__setitem__('markers', msg), state_qos),
            node.create_subscription(
                Bool, '/drone/mission/complete',
                lambda msg: latest.__setitem__('complete', msg.data), state_qos),
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

        def execute_target(target):
            request = ExecuteGoalSequence.Request()
            request.goals.header.frame_id = 'map'
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = target
            pose.orientation.w = 1.0
            request.goals.poses = [pose]
            future = client.call_async(request)
            spin_until(future.done, 5.0, f'mission response for {target}')
            self.assertTrue(future.result().accepted, future.result().message)
            spin_until(
                lambda: latest.get('complete') is False and
                marker_labels(latest.get('markers', MarkerArray())) == {'P1 CURRENT'},
                5.0, f'P1 CURRENT for {target}')
            spin_until(
                lambda: latest.get('complete') is True and
                marker_labels(latest.get('markers', MarkerArray())) == {'P1 DONE'},
                35.0, f'P1 DONE for {target}')
            stable_since = None

            def stable():
                nonlocal stable_since
                within = (math.dist(latest['position'], target) < 0.10 and
                          latest['speed'] < 0.08)
                if not within:
                    stable_since = None
                    return False
                stable_since = stable_since or time.monotonic()
                return time.monotonic() - stable_since >= 1.5

            spin_until(stable, 10.0, f'strict stable target {target}')
            return math.dist(latest['position'], target), latest['speed']

        try:
            spin_until(
                lambda: 'position' in latest and
                {'quadrotor_dynamics_node', 'position_controller_node',
                 'waypoint_manager_node', 'goal_visualizer_node'}.issubset(
                    set(node.get_node_names())),
                8.0, 'formal basic graph')
            self.assertTrue(client.wait_for_service(timeout_sec=8.0))
            self.assertEqual(node.get_node_names().count('waypoint_manager_node'), 1)
            results = [execute_target(target) for target in TARGETS]
            self.assertEqual(marker_labels(latest['markers']), {'P1 DONE'})
            self.assertGreater(len(latest.get('path', Path()).poses), 100)
            self.assertGreater(rpm_samples, 100)
            self.assertLess(maximum_consecutive_saturation_samples, 200)
            self.assertEqual(consecutive_saturation_samples, 0)
            self.assertFalse(health_errors, health_errors)
            for error, speed in results:
                self.assertLess(error, 0.10)
                self.assertLess(speed, 0.08)
            print(
                'assessment_basic_single_e2e: '
                f'hover_error={results[0][0]:.6f} hover_speed={results[0][1]:.6f} '
                f'single_error={results[1][0]:.6f} single_speed={results[1][1]:.6f} '
                f'path_points={len(latest["path"].poses)} rpm_samples={rpm_samples} '
                f'max_consecutive_saturation_samples='
                f'{maximum_consecutive_saturation_samples}',
                flush=True)
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_client(client)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestAssessmentBasicSingleShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
