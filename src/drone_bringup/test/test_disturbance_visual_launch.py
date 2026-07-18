#!/usr/bin/env python3
# flake8: noqa: E402

import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '122'

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rcl_interfaces.srv import GetParameters


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('drone_bringup'), 'launch',
        'disturbance_visual_demo.launch.py')
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'profile': 'persistent_release',
            'start_delay': '60.0',
            'use_rviz': 'false',
        }.items())
    return launch.LaunchDescription([
        demo,
        launch_testing.actions.ReadyToTest(),
    ])


class TestDisturbanceVisualLaunch(unittest.TestCase):

    def test_demo_is_the_explicit_external_wrench_opt_in(self):
        rclpy.init()
        node = rclpy.create_node('disturbance_visual_launch_test')

        def get_parameters(remote_node, names):
            client = node.create_client(GetParameters, f'{remote_node}/get_parameters')
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline and not client.service_is_ready():
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertTrue(client.service_is_ready(), f'parameter service for {remote_node}')
            request = GetParameters.Request()
            request.names = names
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            self.assertTrue(future.done())
            return future.result().values

        try:
            dynamics = get_parameters(
                '/quadrotor_dynamics_node', ['enable_external_wrench'])
            demo = get_parameters(
                '/disturbance_demo_node',
                ['profile', 'disturbance_duration', 'force_z', 'force_publish_rate'])
            self.assertTrue(dynamics[0].bool_value)
            self.assertEqual(demo[0].string_value, 'persistent_release')
            self.assertAlmostEqual(demo[1].double_value, 10.0)
            self.assertAlmostEqual(demo[2].double_value, 0.0)
            self.assertGreaterEqual(demo[3].double_value, 10.0)
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestDisturbanceVisualLaunchShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
