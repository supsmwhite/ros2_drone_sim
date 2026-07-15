#!/usr/bin/env python3

import math
import os
import time
import unittest

# Isolate this system test from both a developer's ROS graph and the single-goal
# launch test, which deliberately uses a different fixed domain.
os.environ['ROS_DOMAIN_ID'] = '94'

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
import rclpy
from std_msgs.msg import Bool, UInt32


EXPECTED_INDICES = [0, 1, 2, 3, 4]
EXPECTED_GOALS = [
    (0.0, 0.0, 1.5, 0.0),
    (2.0, 0.0, 1.5, 0.0),
    (2.0, 1.5, 2.0, math.pi / 2.0),
    (0.0, 1.5, 1.5, math.pi),
    (0.0, 0.0, 1.5, 0.0),
]
FINAL_TARGET = EXPECTED_GOALS[-1][:3]
MISSION_TIMEOUT = 100.0
DISCOVERY_TIMEOUT = 8.0
POST_COMPLETE_OBSERVATION = 2.0
FINAL_POSITION_TOLERANCE = 0.20
FINAL_SPEED_TOLERANCE = 0.15
GOAL_TOLERANCE = 1.0e-6


def shortest_yaw_error(target, current):
    return math.remainder(target - current, 2.0 * math.pi)


def goals_match(actual, expected, tolerance=GOAL_TOLERANCE):
    return (
        all(abs(actual[index] - expected[index]) <= tolerance for index in range(3))
        and abs(shortest_yaw_error(expected[3], actual[3])) <= tolerance
    )


def quaternion_yaw(quaternion):
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('quaternion is not finite')
    norm_squared = sum(value * value for value in values)
    if not math.isfinite(norm_squared) or norm_squared <= 1.0e-12:
        raise ValueError('quaternion norm is invalid')
    inverse_norm = 1.0 / math.sqrt(norm_squared)
    x, y, z, w = (value * inverse_norm for value in values)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    if not math.isfinite(yaw):
        raise ValueError('yaw is not finite')
    return yaw


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    mission_sim = os.path.join(
        get_package_share_directory('drone_bringup'),
        'launch',
        'mission_sim.launch.py',
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mission_sim),
        launch_arguments={'use_rviz': 'false'}.items(),
    )
    return launch.LaunchDescription([
        simulation,
        launch_testing.actions.ReadyToTest(),
    ])


class TestWaypointMissionEndToEnd(unittest.TestCase):

    def test_waypoint_mission_completes_in_order(self):
        rclpy.init()
        node = rclpy.create_node('waypoint_mission_e2e_test')
        test_start = time.monotonic()
        observed_indices = []
        observed_goal_sequence = []
        switch_times = []
        complete_time = None
        latest_position = None
        latest_speed = math.inf
        latest_goal = None
        odom_samples = 0
        goal_samples = 0
        goal_samples_after_complete = 0
        last_odom_time = None
        health_errors = []

        def on_index(message):
            value = int(message.data)
            if not observed_indices or value != observed_indices[-1]:
                observed_indices.append(value)
                switch_times.append(time.monotonic() - test_start)
                expected_prefix = EXPECTED_INDICES[:len(observed_indices)]
                if observed_indices != expected_prefix:
                    health_errors.append(
                        f'waypoint index skipped or regressed: {observed_indices}')

        def on_complete(message):
            nonlocal complete_time
            if message.data and complete_time is None:
                complete_time = time.monotonic()
            elif not message.data and complete_time is not None:
                health_errors.append('mission complete state regressed to false')

        def on_goal(message):
            nonlocal latest_goal, goal_samples, goal_samples_after_complete
            if message.header.frame_id != 'map':
                health_errors.append(
                    f'invalid /drone/goal frame: {message.header.frame_id!r}')
                return
            try:
                yaw = quaternion_yaw(message.pose.orientation)
            except ValueError as error:
                health_errors.append(f'invalid goal quaternion: {error}')
                return
            values = (
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            )
            if not all(math.isfinite(value) for value in values):
                health_errors.append('non-finite value in /drone/goal')
                return
            goal = (*values, yaw)
            if (not observed_goal_sequence or
                    not goals_match(goal, observed_goal_sequence[-1])):
                observed_goal_sequence.append(goal)
                sequence_index = len(observed_goal_sequence) - 1
                if (sequence_index >= len(EXPECTED_GOALS) or
                        not goals_match(goal, EXPECTED_GOALS[sequence_index])):
                    health_errors.append(
                        'goal sequence skipped, regressed, or contained an unexpected target: '
                        f'{observed_goal_sequence}')
            latest_goal = message
            goal_samples += 1
            if complete_time is not None:
                goal_samples_after_complete += 1

        def on_odometry(message):
            nonlocal latest_position, latest_speed, odom_samples, last_odom_time
            odom_samples += 1
            last_odom_time = time.monotonic()
            pose = message.pose.pose
            twist = message.twist.twist
            values = (
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w,
                twist.linear.x, twist.linear.y, twist.linear.z,
                twist.angular.x, twist.angular.y, twist.angular.z,
            )
            if not all(math.isfinite(value) for value in values):
                health_errors.append('non-finite value in /drone/odom')
                return
            try:
                quaternion_yaw(pose.orientation)
            except ValueError as error:
                health_errors.append(f'invalid odometry quaternion: {error}')
                return
            latest_position = (pose.position.x, pose.position.y, pose.position.z)
            latest_speed = math.sqrt(
                twist.linear.x ** 2 + twist.linear.y ** 2 + twist.linear.z ** 2)

        index_subscription = node.create_subscription(
            UInt32, '/drone/mission/current_waypoint_index', on_index, 10)
        complete_subscription = node.create_subscription(
            Bool, '/drone/mission/complete', on_complete, 10)
        goal_subscription = node.create_subscription(
            PoseStamped, '/drone/goal', on_goal, 10)
        odom_subscription = node.create_subscription(
            Odometry, '/drone/odom', on_odometry, 10)

        try:
            required_nodes = {
                'quadrotor_dynamics_node',
                'position_controller_node',
                'waypoint_manager_node',
            }
            discovery_deadline = time.monotonic() + DISCOVERY_TIMEOUT
            while time.monotonic() < discovery_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if (required_nodes.issubset(set(node.get_node_names())) and
                        latest_position is not None and observed_indices == [0] and
                        latest_goal is not None):
                    break
            else:
                self.fail(
                    f'mission ROS graph was not ready; nodes={sorted(node.get_node_names())}, '
                    f'indices={observed_indices}, odom_samples={odom_samples}, '
                    f'goal_sequence={observed_goal_sequence}, goal_samples={goal_samples}')

            mission_deadline = test_start + MISSION_TIMEOUT
            while time.monotonic() < mission_deadline and complete_time is None:
                rclpy.spin_once(node, timeout_sec=0.02)
                now = time.monotonic()
                if health_errors:
                    self.fail(health_errors[0])
                if last_odom_time is not None and now - last_odom_time > 2.0:
                    self.fail('/drone/odom stopped for more than 2 s during mission')
                if not required_nodes.issubset(set(node.get_node_names())):
                    self.fail(
                        'a required node exited during mission; '
                        f'nodes={sorted(node.get_node_names())}')

            self.assertIsNotNone(
                complete_time,
                f'mission did not complete within {MISSION_TIMEOUT:.1f}s; '
                f'indices={observed_indices}, position={latest_position}, speed={latest_speed}')
            observation_deadline = complete_time + POST_COMPLETE_OBSERVATION
            while time.monotonic() < observation_deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                if health_errors:
                    self.fail(health_errors[0])
                self.assertEqual(observed_indices, EXPECTED_INDICES)
                self.assertTrue(required_nodes.issubset(set(node.get_node_names())))

            final_error = math.sqrt(sum(
                (latest_position[index] - FINAL_TARGET[index]) ** 2
                for index in range(3)))
            final_goal_position = (
                latest_goal.pose.position.x,
                latest_goal.pose.position.y,
                latest_goal.pose.position.z,
            )
            final_goal_error = math.sqrt(sum(
                (final_goal_position[index] - FINAL_TARGET[index]) ** 2
                for index in range(3)))
            final_goal_yaw = quaternion_yaw(latest_goal.pose.orientation)
            summary = (
                'waypoint_mission_e2e: '
                f'indices={observed_indices} '
                f'goals={observed_goal_sequence} '
                f'switch_times={[round(value, 3) for value in switch_times]} '
                f'mission_complete_time={complete_time - test_start:.3f}s '
                f'final_position={latest_position} final_error={final_error:.6f} '
                f'final_speed={latest_speed:.6f} '
                f'goal_samples_after_complete={goal_samples_after_complete} '
                f'odom_samples={odom_samples} goal_samples={goal_samples}'
            )
            print(summary, flush=True)

            metrics = (
                f'indices={observed_indices}, goals={observed_goal_sequence}, '
                f'switch_times={switch_times}, '
                f'final_position={latest_position}, final_error={final_error:.6f}, '
                f'final_speed={latest_speed:.6f}, odom_samples={odom_samples}, '
                f'goal_samples={goal_samples}, '
                f'goal_samples_after_complete={goal_samples_after_complete}'
            )
            self.assertEqual(observed_indices, EXPECTED_INDICES, metrics)
            self.assertEqual(len(observed_goal_sequence), len(EXPECTED_GOALS), metrics)
            for actual, expected in zip(observed_goal_sequence, EXPECTED_GOALS):
                self.assertTrue(goals_match(actual, expected), metrics)
            self.assertLess(final_error, FINAL_POSITION_TOLERANCE, metrics)
            self.assertLess(latest_speed, FINAL_SPEED_TOLERANCE, metrics)
            self.assertGreater(odom_samples, 500, metrics)
            self.assertGreater(goal_samples_after_complete, 10, metrics)
            self.assertLess(final_goal_error, 1.0e-9, metrics)
            self.assertLess(abs(final_goal_yaw), 1.0e-9, metrics)
            self.assertFalse(health_errors, '; '.join(health_errors))
        finally:
            node.destroy_subscription(index_subscription)
            node.destroy_subscription(complete_subscription)
            node.destroy_subscription(goal_subscription)
            node.destroy_subscription(odom_subscription)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestWaypointMissionShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        process_names = proc_info.process_names()
        for expected in (
                'quadrotor_dynamics_node',
                'position_controller_node',
                'waypoint_manager_node'):
            self.assertTrue(
                any(expected in name for name in process_names),
                f'{expected} was not launched: {process_names}')
        launch_testing.asserts.assertExitCodes(proc_info)
