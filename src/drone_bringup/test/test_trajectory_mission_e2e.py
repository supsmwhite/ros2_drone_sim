#!/usr/bin/env python3

import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '95'

from ament_index_python.packages import get_package_share_directory
from drone_msgs.msg import TrajectorySetpoint
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
from std_msgs.msg import Bool, UInt32


EXPECTED_SEGMENTS = [0, 1, 2, 3]
INTERMEDIATE_WAYPOINTS = [
    (2.0, 0.0, 1.5),
    (2.0, 1.5, 2.0),
    (0.0, 1.5, 1.5),
]
FINAL_TARGET = (0.0, 0.0, 1.5)
MISSION_TIMEOUT = 70.0
DISCOVERY_TIMEOUT = 8.0
POST_COMPLETE_OBSERVATION = 4.0
FINAL_POSITION_TOLERANCE = 0.20
FINAL_SPEED_TOLERANCE = 0.15
MAX_SETPOINT_POSITION_STEP = 0.10
MAX_REFERENCE_SPEED_LIMIT = 1.0
MAX_REFERENCE_ACCELERATION_LIMIT = 1.0
MAX_VELOCITY_STEP = 0.10
MAX_ACCELERATION_STEP = 0.20


def norm3(values):
    return math.sqrt(sum(value * value for value in values))


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    trajectory_sim = os.path.join(
        get_package_share_directory('drone_bringup'),
        'launch',
        'trajectory_sim.launch.py',
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(trajectory_sim),
        launch_arguments={'use_rviz': 'false'}.items(),
    )
    return launch.LaunchDescription([
        simulation,
        launch_testing.actions.ReadyToTest(),
    ])


class TestTrajectoryMissionEndToEnd(unittest.TestCase):

    def test_trajectory_mission_tracks_continuously(self):
        rclpy.init()
        node = rclpy.create_node('trajectory_mission_e2e_test')
        test_start = time.monotonic()
        observed_segments = []
        segment_switch_times = []
        trajectory_start_time = None
        complete_time = None
        latest_setpoint = None
        latest_position = None
        latest_speed = math.inf
        last_odom_time = None
        setpoint_samples = 0
        odom_samples = 0
        setpoints_after_complete = 0
        reference_path_samples = 0
        max_reference_speed = 0.0
        max_reference_acceleration = 0.0
        max_tracking_error = 0.0
        max_position_step = 0.0
        max_velocity_step = 0.0
        max_acceleration_step = 0.0
        intermediate_speeds = [0.0, 0.0, 0.0]
        health_errors = []

        def on_segment(message):
            value = int(message.data)
            if not observed_segments or value != observed_segments[-1]:
                observed_segments.append(value)
                segment_switch_times.append(time.monotonic() - test_start)
                if observed_segments != EXPECTED_SEGMENTS[:len(observed_segments)]:
                    health_errors.append(
                        f'trajectory segment skipped or regressed: {observed_segments}')

        def on_complete(message):
            nonlocal complete_time
            if message.data and complete_time is None:
                complete_time = time.monotonic()
            elif not message.data and complete_time is not None:
                health_errors.append('trajectory complete state regressed to false')

        def on_reference_path(message):
            nonlocal reference_path_samples
            if message.header.frame_id != 'map':
                health_errors.append(
                    f'invalid reference path frame: {message.header.frame_id!r}')
            reference_path_samples = max(reference_path_samples, len(message.poses))

        def on_setpoint(message):
            nonlocal latest_setpoint, setpoint_samples, setpoints_after_complete
            nonlocal trajectory_start_time, max_reference_speed
            nonlocal max_reference_acceleration, max_position_step
            nonlocal max_velocity_step, max_acceleration_step
            values = (
                message.position.x, message.position.y, message.position.z,
                message.velocity.x, message.velocity.y, message.velocity.z,
                message.acceleration.x, message.acceleration.y,
                message.acceleration.z, message.yaw,
            )
            if message.header.frame_id != 'map':
                health_errors.append(
                    f'invalid trajectory setpoint frame: {message.header.frame_id!r}')
                return
            if not all(math.isfinite(value) for value in values):
                health_errors.append('non-finite trajectory setpoint')
                return
            position = values[0:3]
            velocity = values[3:6]
            acceleration = values[6:9]
            speed = norm3(velocity)
            acceleration_norm = norm3(acceleration)
            if latest_setpoint is not None:
                previous_position = (
                    latest_setpoint.position.x,
                    latest_setpoint.position.y,
                    latest_setpoint.position.z,
                )
                previous_velocity = (
                    latest_setpoint.velocity.x,
                    latest_setpoint.velocity.y,
                    latest_setpoint.velocity.z,
                )
                previous_acceleration = (
                    latest_setpoint.acceleration.x,
                    latest_setpoint.acceleration.y,
                    latest_setpoint.acceleration.z,
                )
                max_position_step = max(
                    max_position_step,
                    norm3(tuple(position[i] - previous_position[i] for i in range(3))))
                max_velocity_step = max(
                    max_velocity_step,
                    norm3(tuple(velocity[i] - previous_velocity[i] for i in range(3))))
                max_acceleration_step = max(
                    max_acceleration_step,
                    norm3(tuple(
                        acceleration[i] - previous_acceleration[i] for i in range(3))))
            if trajectory_start_time is None and (speed > 1.0e-4 or acceleration_norm > 1.0e-4):
                trajectory_start_time = time.monotonic()
            max_reference_speed = max(max_reference_speed, speed)
            max_reference_acceleration = max(max_reference_acceleration, acceleration_norm)
            for index, waypoint in enumerate(INTERMEDIATE_WAYPOINTS):
                distance = norm3(tuple(position[axis] - waypoint[axis] for axis in range(3)))
                if distance < 0.04:
                    intermediate_speeds[index] = max(intermediate_speeds[index], speed)
            latest_setpoint = message
            setpoint_samples += 1
            if complete_time is not None:
                setpoints_after_complete += 1

        def on_odometry(message):
            nonlocal latest_position, latest_speed, last_odom_time, odom_samples
            nonlocal max_tracking_error
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
                health_errors.append('non-finite odometry')
                return
            latest_position = values[0:3]
            latest_speed = norm3(values[7:10])
            last_odom_time = time.monotonic()
            odom_samples += 1
            if trajectory_start_time is not None and latest_setpoint is not None:
                reference_position = (
                    latest_setpoint.position.x,
                    latest_setpoint.position.y,
                    latest_setpoint.position.z,
                )
                tracking_error = norm3(tuple(
                    latest_position[index] - reference_position[index]
                    for index in range(3)))
                max_tracking_error = max(max_tracking_error, tracking_error)

        segment_subscription = node.create_subscription(
            UInt32, '/drone/trajectory/current_segment', on_segment, 10)
        complete_subscription = node.create_subscription(
            Bool, '/drone/trajectory/complete', on_complete, 10)
        setpoint_subscription = node.create_subscription(
            TrajectorySetpoint, '/drone/trajectory_setpoint', on_setpoint, 10)
        odom_subscription = node.create_subscription(
            Odometry, '/drone/odom', on_odometry, 10)
        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        path_subscription = node.create_subscription(
            Path, '/drone/reference_path', on_reference_path, path_qos)

        try:
            required_nodes = {
                'quadrotor_dynamics_node',
                'position_controller_node',
                'trajectory_mission_node',
            }
            discovery_deadline = time.monotonic() + DISCOVERY_TIMEOUT
            while time.monotonic() < discovery_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if (required_nodes.issubset(set(node.get_node_names())) and
                        latest_position is not None and latest_setpoint is not None and
                        observed_segments == [0] and reference_path_samples > 100):
                    break
            else:
                self.fail(
                    f'trajectory ROS graph was not ready; nodes={sorted(node.get_node_names())}, '
                    f'segments={observed_segments}, odom={odom_samples}, '
                    f'setpoints={setpoint_samples}, path={reference_path_samples}')

            mission_deadline = test_start + MISSION_TIMEOUT
            while time.monotonic() < mission_deadline and complete_time is None:
                rclpy.spin_once(node, timeout_sec=0.01)
                now = time.monotonic()
                if health_errors:
                    self.fail(health_errors[0])
                if last_odom_time is not None and now - last_odom_time > 2.0:
                    self.fail('/drone/odom stopped for more than 2 s')
                if not required_nodes.issubset(set(node.get_node_names())):
                    self.fail(f'a required node exited: {sorted(node.get_node_names())}')

            self.assertIsNotNone(
                complete_time,
                f'trajectory did not complete; segments={observed_segments}, '
                f'position={latest_position}, speed={latest_speed}')
            observation_deadline = complete_time + POST_COMPLETE_OBSERVATION
            while time.monotonic() < observation_deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
                if health_errors:
                    self.fail(health_errors[0])
                self.assertTrue(required_nodes.issubset(set(node.get_node_names())))

            final_error = norm3(tuple(
                latest_position[index] - FINAL_TARGET[index] for index in range(3)))
            final_setpoint_position = (
                latest_setpoint.position.x,
                latest_setpoint.position.y,
                latest_setpoint.position.z,
            )
            final_setpoint_error = norm3(tuple(
                final_setpoint_position[index] - FINAL_TARGET[index]
                for index in range(3)))
            summary = (
                'trajectory_mission_e2e: '
                f'trajectory_start_time={trajectory_start_time - test_start:.3f}s '
                f'segments={observed_segments} '
                f'segment_switch_times={[round(value, 3) for value in segment_switch_times]} '
                f'mission_complete_time={complete_time - test_start:.3f}s '
                f'max_reference_speed={max_reference_speed:.6f} '
                f'max_reference_acceleration={max_reference_acceleration:.6f} '
                f'max_tracking_error_sampled={max_tracking_error:.6f} '
                f'max_position_step={max_position_step:.6f} '
                f'max_velocity_step={max_velocity_step:.6f} '
                f'max_acceleration_step={max_acceleration_step:.6f} '
                f'intermediate_speeds={[round(value, 6) for value in intermediate_speeds]} '
                f'final_position={latest_position} final_error={final_error:.6f} '
                f'final_speed={latest_speed:.6f} odom_samples={odom_samples} '
                f'setpoint_samples={setpoint_samples} '
                f'setpoints_after_complete={setpoints_after_complete}'
            )
            print(summary, flush=True)

            self.assertIsNotNone(trajectory_start_time, summary)
            self.assertEqual(observed_segments, EXPECTED_SEGMENTS, summary)
            self.assertTrue(all(speed > 0.10 for speed in intermediate_speeds), summary)
            self.assertLess(max_position_step, MAX_SETPOINT_POSITION_STEP, summary)
            self.assertLess(max_tracking_error, 0.10, summary)
            self.assertLess(max_reference_speed, MAX_REFERENCE_SPEED_LIMIT, summary)
            self.assertLess(
                max_reference_acceleration, MAX_REFERENCE_ACCELERATION_LIMIT, summary)
            self.assertLess(max_velocity_step, MAX_VELOCITY_STEP, summary)
            self.assertLess(max_acceleration_step, MAX_ACCELERATION_STEP, summary)
            self.assertLess(final_setpoint_error, 1.0e-9, summary)
            self.assertLess(final_error, FINAL_POSITION_TOLERANCE, summary)
            self.assertLess(latest_speed, FINAL_SPEED_TOLERANCE, summary)
            self.assertGreater(setpoints_after_complete, 100, summary)
            self.assertGreater(reference_path_samples, 100, summary)
            self.assertGreater(odom_samples, 3000, summary)
            self.assertGreater(setpoint_samples, 1000, summary)
            self.assertFalse(health_errors, '; '.join(health_errors))
        finally:
            node.destroy_subscription(segment_subscription)
            node.destroy_subscription(complete_subscription)
            node.destroy_subscription(setpoint_subscription)
            node.destroy_subscription(odom_subscription)
            node.destroy_subscription(path_subscription)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestTrajectoryMissionShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        process_names = proc_info.process_names()
        for expected in (
                'quadrotor_dynamics_node',
                'position_controller_node',
                'trajectory_mission_node'):
            self.assertTrue(
                any(expected in name for name in process_names),
                f'{expected} was not launched: {process_names}')
        launch_testing.asserts.assertExitCodes(proc_info)
