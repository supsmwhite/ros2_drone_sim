#!/usr/bin/env python3

import importlib.util
import math
import os
from pathlib import Path
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '134'

from ament_index_python.packages import get_package_share_directory
from drone_msgs.msg import ControllerDiagnostics, MotorRPM
from geometry_msgs.msg import WrenchStamped
import launch
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rcl_interfaces.srv import GetParameters
from visualization_msgs.msg import Marker, MarkerArray


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('drone_bringup'), 'launch',
        'assessment_disturbance_sim.launch.py')
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={
                'start_delay': '1.0', 'recovery_duration': '5.0',
                'use_rviz': 'false'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestAssessmentDisturbanceLaunch(unittest.TestCase):

    def test_persistent_release_defaults_to_ten_seconds(self):
        launch_path = (
            Path(get_package_share_directory('drone_bringup')) / 'launch' /
            'disturbance_visual_demo.launch.py')
        spec = importlib.util.spec_from_file_location(
            'disturbance_visual_demo_launch', launch_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        duration = next(
            action for action in description.entities
            if isinstance(action, DeclareLaunchArgument) and
            action.name == 'disturbance_duration')
        context = LaunchContext()
        context.launch_configurations['profile'] = 'persistent_release'
        resolved = ''.join(
            substitution.perform(context) for substitution in duration.default_value)
        self.assertEqual(resolved, '10.0')

    def test_default_short_gust_is_explicit_external_wrench_opt_in(self):
        rclpy.init()
        node = rclpy.create_node('assessment_disturbance_launch_test')

        def parameters(remote_node, names):
            client = node.create_client(GetParameters, f'{remote_node}/get_parameters')
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline and not client.service_is_ready():
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertTrue(client.service_is_ready())
            request = GetParameters.Request()
            request.names = names
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            self.assertTrue(future.done())
            return future.result().values

        latest = {}
        applied_forces = []
        marker_messages = []
        health_errors = []
        consecutive_saturation = 0
        maximum_consecutive_saturation = 0

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
            nonlocal consecutive_saturation, maximum_consecutive_saturation
            latest['integral_x'] = message.horizontal_i_acceleration_x
            saturated = (
                message.horizontal_saturated or message.altitude_saturated or
                message.attitude_saturated or message.mixer_saturated)
            consecutive_saturation = consecutive_saturation + 1 if saturated else 0
            maximum_consecutive_saturation = max(
                maximum_consecutive_saturation, consecutive_saturation)

        marker_qos = rclpy.qos.QoSProfile(
            depth=1, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = [
            node.create_subscription(Odometry, '/drone/odom', on_odom, 20),
            node.create_subscription(MotorRPM, '/drone/motor_rpm_cmd', on_rpm, 20),
            node.create_subscription(
                ControllerDiagnostics, '/drone/controller/diagnostics',
                on_diagnostics, 20),
            node.create_subscription(
                WrenchStamped, '/drone/external_wrench/applied',
                applied_forces.append, 20),
            node.create_subscription(
                MarkerArray, '/drone/disturbance/markers',
                marker_messages.append, marker_qos),
        ]

        def marker_texts():
            return [marker.text for array in marker_messages for marker in array.markers
                    if marker.type == Marker.TEXT_VIEW_FACING and
                    marker.action == Marker.ADD]

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
            dynamics = parameters(
                '/quadrotor_dynamics_node', ['enable_external_wrench'])
            demo = parameters(
                '/disturbance_demo_node',
                ['profile', 'disturbance_duration', 'force_x', 'force_y', 'force_z'])
            self.assertTrue(dynamics[0].bool_value)
            self.assertEqual(demo[0].string_value, 'short_gust')
            self.assertAlmostEqual(demo[1].double_value, 2.0)
            self.assertGreater(demo[2].double_value, 0.0)
            self.assertAlmostEqual(demo[3].double_value, 0.0)
            self.assertAlmostEqual(demo[4].double_value, 0.0)
            spin_until(
                lambda: any(message.wrench.force.x > 0.29 for message in applied_forces),
                12.0, 'applied short gust')
            spin_until(
                lambda: any(
                    marker.ns == 'equivalent_external_force' and
                    marker.action == Marker.ADD and len(marker.points) == 2 and
                    marker.points[1].x > marker.points[0].x
                    for array in marker_messages for marker in array.markers),
                1.0, 'positive-X external-force marker')
            spin_until(
                lambda: latest.get('integral_x', 0.0) < -0.01 and any(
                    marker.ns == 'horizontal_integral_acceleration' and
                    marker.action == Marker.ADD and len(marker.points) == 2 and
                    marker.points[1].x < marker.points[0].x
                    for array in marker_messages for marker in array.markers),
                4.0, 'negative-X integral compensation')
            spin_until(
                lambda: any(text.startswith('COMPLETE') for text in marker_texts()),
                9.0, 'disturbance recovery complete')
            self.assertTrue(all(
                abs(message.wrench.force.x) < 1.0e-9
                for message in applied_forces[-5:]))
            final_horizontal_error = math.hypot(
                latest['position'][0], latest['position'][1])
            self.assertLess(final_horizontal_error, 0.15)
            self.assertLess(latest['speed'], 0.10)
            self.assertLess(maximum_consecutive_saturation, 200)
            self.assertEqual(consecutive_saturation, 0)
            print(
                'assessment_disturbance_e2e: '
                f'final_horizontal_error={final_horizontal_error:.6f} '
                f'final_speed={latest["speed"]:.6f} '
                f'max_consecutive_saturation_samples='
                f'{maximum_consecutive_saturation}', flush=True)
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestAssessmentDisturbanceShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        process_names = proc_info.process_names()
        for expected in (
                'quadrotor_dynamics_node', 'position_controller_node',
                'robot_state_publisher', 'disturbance_demo_node'):
            self.assertTrue(any(expected in name for name in process_names))
        self.assertFalse(any('rviz2' in name for name in process_names))
        launch_testing.asserts.assertExitCodes(proc_info)
