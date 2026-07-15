#!/usr/bin/env python3

import math
import os
import time
import unittest

# Keep this launch test independent from ROS nodes which may already be running
# on a developer workstation.  This is set before either rclpy or the launched
# child processes create a DDS participant, so every participant uses domain 93.
os.environ['ROS_DOMAIN_ID'] = '93'

from ament_index_python.packages import get_package_share_directory
from drone_msgs.msg import MotorRPM
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


TARGET = (2.0, 1.0, 1.5)
POSITION_TOLERANCE = 0.30
HORIZONTAL_SPEED_TOLERANCE = 0.15
VERTICAL_SPEED_TOLERANCE = 0.10
STABLE_DURATION = 2.0
POST_STABLE_OBSERVATION = 3.0
FINAL_POSITION_TOLERANCE = 0.10
FINAL_HORIZONTAL_SPEED_TOLERANCE = 0.08
FINAL_VERTICAL_SPEED_TOLERANCE = 0.05
DISCOVERY_TIMEOUT = 8.0
FLIGHT_TIMEOUT = 40.0
GOAL_PUBLISH_DURATION = 1.0
RPM_STARTUP_GRACE = 2.0
MIN_ODOM_SAMPLES = 100
MIN_RPM_SAMPLES = 50
RPM_LOWER_BOUND = 0.0
RPM_UPPER_BOUND = 20000.0
RPM_BOUNDARY_EPSILON = 1.0e-6


def rotate_body_vector_to_world(quaternion, vector_body):
    """Rotate a base_link vector into map using a ROS (x, y, z, w) quaternion."""
    q = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    v = (vector_body.x, vector_body.y, vector_body.z)
    if not all(math.isfinite(value) for value in q + v):
        raise ValueError('quaternion and vector components must be finite')

    norm_squared = sum(value * value for value in q)
    if not math.isfinite(norm_squared) or norm_squared <= 1.0e-12:
        raise ValueError('quaternion norm is invalid')
    inverse_norm = 1.0 / math.sqrt(norm_squared)
    qx, qy, qz, qw = (value * inverse_norm for value in q)
    vx, vy, vz = v

    # Unit-quaternion vector rotation: v' = v + 2*(qw*(q_xyz x v)
    # + q_xyz x (q_xyz x v)).
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    rotated = (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )
    if not all(math.isfinite(value) for value in rotated):
        raise ValueError('rotated vector is not finite')
    return rotated


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    basic_sim = os.path.join(
        get_package_share_directory('drone_bringup'),
        'launch',
        'basic_sim.launch.py',
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(basic_sim),
        launch_arguments={'use_rviz': 'false'}.items(),
    )
    return launch.LaunchDescription([
        simulation,
        launch_testing.actions.ReadyToTest(),
    ])


class TestSingleGoalEndToEnd(unittest.TestCase):

    def test_quaternion_rotation_helper(self):
        quaternion = type('Quaternion', (), {
            'x': 0.0,
            'y': math.sin(math.pi / 4.0),
            'z': 0.0,
            'w': math.cos(math.pi / 4.0),
        })()
        vector = type('Vector', (), {'x': 1.0, 'y': 0.0, 'z': 0.0})()
        rotated = rotate_body_vector_to_world(quaternion, vector)
        self.assertAlmostEqual(rotated[0], 0.0, places=12)
        self.assertAlmostEqual(rotated[1], 0.0, places=12)
        self.assertAlmostEqual(rotated[2], -1.0, places=12)
        quaternion.w = float('nan')
        with self.assertRaises(ValueError):
            rotate_body_vector_to_world(quaternion, vector)
        quaternion.w = 0.0
        quaternion.y = 0.0
        with self.assertRaises(ValueError):
            rotate_body_vector_to_world(quaternion, vector)

    def test_single_goal_reaches_stable_hover(self):
        rclpy.init()
        node = rclpy.create_node('single_goal_e2e_test')
        first_goal_publish_time = None
        first_position_tolerance_time = None
        stable_start_time = None
        acceptance_time = None
        observation_end_time = None
        post_stable_deadline = None
        latest_position = None
        latest_velocity_world = None
        latest_position_error = math.inf
        latest_horizontal_speed = math.inf
        latest_vertical_speed = math.inf
        last_odom_reception_time = None
        last_rpm_reception_time = None
        odom_samples = 0
        rpm_samples = 0
        minimum_rpm_all = math.inf
        maximum_rpm_all = -math.inf
        minimum_rpm_after_grace = math.inf
        maximum_rpm_after_grace = -math.inf
        rpm_samples_after_grace = 0
        health_errors = []

        def on_odometry(message):
            nonlocal latest_position, latest_velocity_world
            nonlocal latest_position_error, latest_horizontal_speed, latest_vertical_speed
            nonlocal last_odom_reception_time, odom_samples
            odom_samples += 1
            last_odom_reception_time = time.monotonic()
            pose = message.pose.pose
            twist = message.twist.twist
            critical_values = (
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w,
                twist.linear.x, twist.linear.y, twist.linear.z,
                twist.angular.x, twist.angular.y, twist.angular.z,
            )
            if not all(math.isfinite(value) for value in critical_values):
                health_errors.append('non-finite value in /drone/odom')
                return
            try:
                velocity_world = rotate_body_vector_to_world(
                    pose.orientation, twist.linear)
            except ValueError as error:
                health_errors.append(f'invalid /drone/odom quaternion: {error}')
                return

            position = (pose.position.x, pose.position.y, pose.position.z)
            latest_position = position
            latest_velocity_world = velocity_world
            latest_position_error = math.sqrt(sum(
                (position[index] - TARGET[index]) ** 2 for index in range(3)))
            latest_horizontal_speed = math.hypot(velocity_world[0], velocity_world[1])
            latest_vertical_speed = abs(velocity_world[2])

        def on_motor_rpm(message):
            nonlocal last_rpm_reception_time, rpm_samples
            nonlocal minimum_rpm_all, maximum_rpm_all
            nonlocal minimum_rpm_after_grace, maximum_rpm_after_grace
            nonlocal rpm_samples_after_grace
            rpm_samples += 1
            now = time.monotonic()
            last_rpm_reception_time = now
            values = (
                message.m1_front_left_ccw_rpm,
                message.m2_rear_left_cw_rpm,
                message.m3_rear_right_ccw_rpm,
                message.m4_front_right_cw_rpm,
            )
            if not all(math.isfinite(value) for value in values):
                health_errors.append('non-finite value in /drone/motor_rpm_cmd')
                return
            minimum_rpm_all = min(minimum_rpm_all, *values)
            maximum_rpm_all = max(maximum_rpm_all, *values)
            if (first_goal_publish_time is not None and
                    now - first_goal_publish_time >= RPM_STARTUP_GRACE):
                rpm_samples_after_grace += 1
                minimum_rpm_after_grace = min(minimum_rpm_after_grace, *values)
                maximum_rpm_after_grace = max(maximum_rpm_after_grace, *values)
                if any(value <= RPM_LOWER_BOUND + RPM_BOUNDARY_EPSILON for value in values):
                    health_errors.append(
                        f'zero/boundary RPM after startup grace at t='
                        f'{now - first_goal_publish_time:.3f}s: {values}')
                if any(value >= RPM_UPPER_BOUND - RPM_BOUNDARY_EPSILON for value in values):
                    health_errors.append(
                        f'max/boundary RPM after startup grace at t='
                        f'{now - first_goal_publish_time:.3f}s: {values}')

        odom_subscription = node.create_subscription(
            Odometry, '/drone/odom', on_odometry, 10)
        rpm_subscription = node.create_subscription(
            MotorRPM, '/drone/motor_rpm_cmd', on_motor_rpm, 10)
        goal_publisher = node.create_publisher(PoseStamped, '/drone/goal', 10)

        try:
            discovery_deadline = time.monotonic() + DISCOVERY_TIMEOUT
            required_nodes = {'quadrotor_dynamics_node', 'position_controller_node'}
            while time.monotonic() < discovery_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                discovered_nodes = set(node.get_node_names())
                if (latest_position is not None and
                        goal_publisher.get_subscription_count() > 0 and
                        required_nodes.issubset(discovered_nodes)):
                    break
            else:
                self.fail(
                    'ROS graph did not become ready within '
                    f'{DISCOVERY_TIMEOUT:.1f}s; nodes={sorted(node.get_node_names())}, '
                    f'goal_subscribers={goal_publisher.get_subscription_count()}, '
                    f'odom_samples={odom_samples}')

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = TARGET
            goal.pose.orientation.w = 1.0
            first_goal_publish_time = time.monotonic()
            publish_until = first_goal_publish_time + GOAL_PUBLISH_DURATION
            next_publish_time = first_goal_publish_time
            while time.monotonic() < publish_until:
                now = time.monotonic()
                if now >= next_publish_time:
                    goal.header.stamp = node.get_clock().now().to_msg()
                    goal_publisher.publish(goal)
                    next_publish_time = now + 0.10
                rclpy.spin_once(node, timeout_sec=0.02)

            flight_deadline = first_goal_publish_time + FLIGHT_TIMEOUT
            while time.monotonic() < flight_deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                now = time.monotonic()
                if health_errors:
                    self.fail(health_errors[0])
                if (last_odom_reception_time is not None and
                        now - last_odom_reception_time > 2.0):
                    self.fail('the /drone/odom stream stopped for more than 2 s')
                if (last_rpm_reception_time is not None and
                        now - last_rpm_reception_time > 2.0):
                    self.fail('the /drone/motor_rpm_cmd stream stopped for more than 2 s')
                if not required_nodes.issubset(set(node.get_node_names())):
                    self.fail(
                        'a required node exited before stable hover; '
                        f'nodes={sorted(node.get_node_names())}')

                within_position = latest_position_error < POSITION_TOLERANCE
                stable = (
                    within_position and
                    latest_horizontal_speed < HORIZONTAL_SPEED_TOLERANCE and
                    latest_vertical_speed < VERTICAL_SPEED_TOLERANCE
                )
                if within_position and first_position_tolerance_time is None:
                    first_position_tolerance_time = now
                if acceptance_time is None:
                    if stable:
                        if stable_start_time is None:
                            stable_start_time = now
                        if now - stable_start_time >= STABLE_DURATION:
                            acceptance_time = now
                            post_stable_deadline = now + POST_STABLE_OBSERVATION
                    else:
                        stable_start_time = None
                else:
                    if not stable:
                        elapsed_observation = now - acceptance_time
                        self.fail(
                            'vehicle left the stable acceptance region during '
                            'post-stable observation; '
                            f'position error={latest_position_error:.6f}, '
                            f'horizontal speed={latest_horizontal_speed:.6f}, '
                            f'vertical speed={latest_vertical_speed:.6f}, '
                            f'elapsed observation time={elapsed_observation:.3f}s')
                    if now >= post_stable_deadline:
                        observation_end_time = now
                        break

            total_stable_hold = (
                observation_end_time - stable_start_time
                if observation_end_time is not None and stable_start_time is not None else 0.0)
            first_position_text = (
                f'{first_position_tolerance_time - first_goal_publish_time:.3f}s'
                if first_position_tolerance_time is not None else 'n/a')
            stable_start_text = (
                f'{stable_start_time - first_goal_publish_time:.3f}s'
                if stable_start_time is not None else 'n/a')
            acceptance_text = (
                f'{acceptance_time - first_goal_publish_time:.3f}s'
                if acceptance_time is not None else 'n/a')
            observation_end_text = (
                f'{observation_end_time - first_goal_publish_time:.3f}s'
                if observation_end_time is not None else 'n/a')
            summary = (
                'single_goal_e2e: '
                f'target={list(TARGET)} '
                f'reached={observation_end_time is not None} '
                f'first_position_tolerance_time={first_position_text} '
                f'stable_start_time={stable_start_text} '
                f'acceptance_time={acceptance_text} '
                f'observation_end_time={observation_end_text} '
                f'post_stable_observation={POST_STABLE_OBSERVATION:.3f}s '
                f'total_stable_hold={total_stable_hold:.3f}s '
                f'observation_end_position={latest_position} '
                f'observation_end_error={latest_position_error:.6f} '
                f'observation_end_velocity_world={latest_velocity_world} '
                f'rpm_range_all=[{minimum_rpm_all:.3f}, {maximum_rpm_all:.3f}] '
                f'rpm_range_after_grace=['
                f'{minimum_rpm_after_grace:.3f}, {maximum_rpm_after_grace:.3f}] '
                f'odom_samples={odom_samples} rpm_samples={rpm_samples} '
                f'rpm_samples_after_grace={rpm_samples_after_grace}'
            )
            print(summary, flush=True)

            failure_metrics = (
                f'observation end position error={latest_position_error:.6f}, '
                f'observation end horizontal speed={latest_horizontal_speed:.6f}, '
                f'observation end vertical speed={latest_vertical_speed:.6f}, '
                f'rpm range all=[{minimum_rpm_all:.3f}, {maximum_rpm_all:.3f}], '
                f'rpm range after grace=['
                f'{minimum_rpm_after_grace:.3f}, {maximum_rpm_after_grace:.3f}], '
                f'odom samples={odom_samples}, rpm samples={rpm_samples}'
            )
            self.assertIsNotNone(
                acceptance_time,
                f'target was not accepted within {FLIGHT_TIMEOUT:.1f}s; '
                + failure_metrics)
            self.assertIsNotNone(
                observation_end_time,
                f'post-stable observation did not finish within {FLIGHT_TIMEOUT:.1f}s; '
                + failure_metrics)
            self.assertGreater(odom_samples, MIN_ODOM_SAMPLES, failure_metrics)
            self.assertGreater(rpm_samples, MIN_RPM_SAMPLES, failure_metrics)
            self.assertGreater(rpm_samples_after_grace, 0, failure_metrics)
            self.assertGreater(
                minimum_rpm_after_grace,
                RPM_LOWER_BOUND + RPM_BOUNDARY_EPSILON,
                failure_metrics)
            self.assertLess(
                maximum_rpm_after_grace,
                RPM_UPPER_BOUND - RPM_BOUNDARY_EPSILON,
                failure_metrics)
            self.assertLess(latest_position_error, FINAL_POSITION_TOLERANCE, failure_metrics)
            self.assertLess(
                latest_horizontal_speed, FINAL_HORIZONTAL_SPEED_TOLERANCE, failure_metrics)
            self.assertLess(
                latest_vertical_speed, FINAL_VERTICAL_SPEED_TOLERANCE, failure_metrics)
            self.assertGreaterEqual(
                total_stable_hold,
                STABLE_DURATION + POST_STABLE_OBSERVATION,
                failure_metrics)
            self.assertFalse(health_errors, '; '.join(health_errors))
        finally:
            node.destroy_subscription(odom_subscription)
            node.destroy_subscription(rpm_subscription)
            node.destroy_publisher(goal_publisher)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestSingleGoalEndToEndShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        process_names = proc_info.process_names()
        self.assertTrue(
            any('quadrotor_dynamics_node' in name for name in process_names),
            f'dynamics process was not launched: {process_names}')
        self.assertTrue(
            any('position_controller_node' in name for name in process_names),
            f'controller process was not launched: {process_names}')
        launch_testing.asserts.assertExitCodes(proc_info)
