#!/usr/bin/env python3

import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '118'

from ament_index_python.packages import get_package_share_directory
from drone_msgs.msg import MotorRPM, TrajectorySetpoint
from drone_msgs.srv import ExecuteGoalSequence
from geometry_msgs.msg import Pose
import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    bringup = get_package_share_directory('drone_bringup')
    dynamics = Node(
        package='drone_dynamics',
        executable='quadrotor_dynamics_node',
        name='quadrotor_dynamics_node',
        output='screen',
        parameters=[os.path.join(bringup, 'config', 'dynamics.yaml')],
    )
    controller = Node(
        package='drone_controller',
        executable='position_controller_node',
        name='position_controller_node',
        output='screen',
        parameters=[
            os.path.join(bringup, 'config', 'controller.yaml'),
            {'setpoint_source': 'trajectory'},
        ],
    )
    executor = Node(
        package='drone_planning',
        executable='multi_goal_static_avoidance_node',
        name='multi_goal_static_avoidance_node',
        output='screen',
        parameters=[
            os.path.join(bringup, 'config', 'environment.yaml'),
            os.path.join(bringup, 'config', 'astar.yaml'),
            os.path.join(bringup, 'config', 'planned_trajectory.yaml'),
            os.path.join(bringup, 'config', 'interactive_goal_executor.yaml'),
            {
                # Test-only closed wall: both endpoints are geometrically legal,
                # but no complete START -> P1 route exists.
                'workspace': [-1.0, 5.0, -2.5, 2.5, -0.5, 5.0],
                'obstacles': [2.0, 0.0, 2.35, 0.8, 5.0, 4.7],
                'interactive_mission_odom_wait_timeout': 2.0,
            },
        ],
    )
    return launch.LaunchDescription([
        dynamics,
        controller,
        executor,
        launch_testing.actions.ReadyToTest(),
    ])


class TestInteractivePreflightFailure(unittest.TestCase):

    def test_no_path_rejection_publishes_no_flight_setpoint(self):
        rclpy.init()
        node = rclpy.create_node('interactive_preflight_failure_test')
        qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        latest = {}
        setpoint_count = 0
        rpm_count = 0
        maximum_command_rpm = 0.0
        initial_xy = None
        maximum_horizontal_displacement = 0.0
        maximum_absolute_z = 0.0
        health_errors = []
        complete_values = []

        def on_setpoint(_message):
            nonlocal setpoint_count
            setpoint_count += 1

        def on_rpm(message):
            nonlocal rpm_count, maximum_command_rpm
            values = (
                message.m1_front_left_ccw_rpm,
                message.m2_rear_left_cw_rpm,
                message.m3_rear_right_ccw_rpm,
                message.m4_front_right_cw_rpm,
            )
            rpm_count += 1
            if not all(math.isfinite(value) for value in values):
                health_errors.append('non-finite motor RPM command')
            maximum_command_rpm = max(maximum_command_rpm, *map(abs, values))

        def on_odom(message):
            nonlocal initial_xy, maximum_horizontal_displacement, maximum_absolute_z
            position = (
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            )
            velocity = (
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.linear.z,
            )
            if not all(math.isfinite(value) for value in position + velocity):
                health_errors.append('non-finite Odom')
            if initial_xy is None:
                initial_xy = position[:2]
            maximum_horizontal_displacement = max(
                maximum_horizontal_displacement,
                math.dist(position[:2], initial_xy))
            maximum_absolute_z = max(maximum_absolute_z, abs(position[2]))
            latest['position'] = position

        def on_complete(message):
            complete_values.append(message.data)
            latest['complete'] = message.data

        subscriptions = [
            node.create_subscription(
                Bool, '/drone/interactive_mission/active',
                lambda msg: latest.__setitem__('active', msg.data), qos),
            node.create_subscription(
                Bool, '/drone/multi_goal/success',
                lambda msg: latest.__setitem__('success', msg.data), 10),
            node.create_subscription(
                String, '/drone/interactive_mission/status',
                lambda msg: latest.__setitem__('status', msg.data), qos),
            node.create_subscription(
                TrajectorySetpoint, '/drone/trajectory_setpoint', on_setpoint, 10),
            node.create_subscription(MotorRPM, '/drone/motor_rpm_cmd', on_rpm, 20),
            node.create_subscription(Odometry, '/drone/odom', on_odom, 20),
            node.create_subscription(
                Bool, '/drone/multi_goal/complete', on_complete, 10),
        ]
        client = node.create_client(
            ExecuteGoalSequence, '/drone/interactive_goals/execute')

        def spin_until(predicate, timeout, description):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if predicate():
                    return
            self.fail(f'timed out waiting for {description}; latest={latest}')

        try:
            self.assertTrue(client.wait_for_service(timeout_sec=8.0))
            spin_until(
                lambda: latest.get('active') is False and
                latest.get('status') == 'WAITING FOR VALIDATED MISSION' and
                'position' in latest and rpm_count > 0,
                5.0, 'grounded waiting state')
            warmup_deadline = time.monotonic() + 1.0
            while time.monotonic() < warmup_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)

            request = ExecuteGoalSequence.Request()
            request.goals.header.frame_id = 'map'
            goal = Pose()
            request.goals.poses = [goal]
            goal.position.x = 4.0
            goal.position.y = 0.0
            goal.position.z = 1.5
            goal.orientation.w = 1.0
            request.draft_revision = 7
            future = client.call_async(request)
            spin_until(future.done, 4.0, 'service response')
            self.assertTrue(future.result().accepted)
            spin_until(
                lambda: latest.get('active') is False and
                latest.get('success') is False and
                latest.get('status', '').startswith(
                    'MISSION FAILED: REJECTED: preflight segment'),
                5.0, 'preflight no-path rejection')
            observation_deadline = time.monotonic() + 3.0
            while time.monotonic() < observation_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)

            self.assertEqual(setpoint_count, 0)
            self.assertGreater(rpm_count, 100)
            self.assertLess(maximum_command_rpm, 1.0e-6)
            self.assertLess(maximum_absolute_z, 0.02)
            self.assertLess(maximum_horizontal_displacement, 0.02)
            self.assertFalse(any(complete_values))
            self.assertFalse(latest.get('complete', True))
            self.assertFalse(latest.get('active', True))
            self.assertFalse(latest.get('success', True))
            self.assertIn('START -> P1 A* failed', latest['status'])
            self.assertFalse(health_errors, health_errors)
            print(
                'interactive_preflight_failure: '
                f'setpoints={setpoint_count}, rpm_messages={rpm_count}, '
                f'max_command_rpm={maximum_command_rpm:.3e}, '
                f'max_abs_z={maximum_absolute_z:.6f} m, '
                f'max_horizontal_displacement={maximum_horizontal_displacement:.6f} m')
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_client(client)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestInteractivePreflightFailureShutdown(unittest.TestCase):

    def test_process_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
