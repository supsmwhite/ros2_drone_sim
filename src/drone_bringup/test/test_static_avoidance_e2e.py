#!/usr/bin/env python3

import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '99'

from ament_index_python.packages import get_package_share_directory
from drone_msgs.msg import MotorRPM, TrajectorySetpoint
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


FINAL_TARGET = (13.2, 5.5, 1.5)
YAW_MODE = os.environ.get('STATIC_AVOIDANCE_YAW_MODE', 'fixed')
DISCOVERY_TIMEOUT = 8.0
MISSION_TIMEOUT = 85.0
POST_COMPLETE_OBSERVATION = 3.0
BASE_INFLATED_OBSTACLES = (
    ((1.95, -2.75, -0.25), (3.25, 1.75, 4.95)),
    ((3.95, 1.55, -0.25), (5.25, 6.75, 4.95)),
    ((6.05, -1.05, -0.25), (7.35, 2.65, 4.95)),
    ((8.25, 0.75, -0.25), (9.55, 6.75, 4.95)),
    ((10.75, -1.75, -0.25), (12.05, 0.05, 4.95)),
    ((10.75, 1.45, -0.25), (12.05, 4.45, 4.95)),
)


def norm3(values):
    return math.sqrt(sum(value * value for value in values))


def distance_to_box(point, lower, upper):
    offsets = tuple(
        max(lower[index] - point[index], 0.0, point[index] - upper[index])
        for index in range(3)
    )
    return norm3(offsets)


def inside_closed_box(point, lower, upper):
    return all(lower[index] <= point[index] <= upper[index] for index in range(3))


def segment_intersects_closed_box(start, end, lower, upper):
    interval_min = 0.0
    interval_max = 1.0
    for index in range(3):
        delta = end[index] - start[index]
        if delta == 0.0:
            if start[index] < lower[index] or start[index] > upper[index]:
                return False
            continue
        axis_min = (lower[index] - start[index]) / delta
        axis_max = (upper[index] - start[index]) / delta
        if axis_min > axis_max:
            axis_min, axis_max = axis_max, axis_min
        interval_min = max(interval_min, axis_min)
        interval_max = min(interval_max, axis_max)
        if interval_min > interval_max:
            return False
    return True


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    static_avoidance_sim = os.path.join(
        get_package_share_directory('drone_bringup'),
        'launch',
        'static_avoidance_sim.launch.py',
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(static_avoidance_sim),
        launch_arguments={'use_rviz': 'false', 'yaw_mode': YAW_MODE}.items(),
    )
    return launch.LaunchDescription([
        simulation,
        launch_testing.actions.ReadyToTest(),
    ])


class TestStaticAvoidanceEndToEnd(unittest.TestCase):

    def test_closed_box_segment_helper(self):
        lower = (1.0, 2.0, 3.0)
        upper = (2.0, 4.0, 6.0)
        self.assertTrue(segment_intersects_closed_box(
            (0.0, 3.0, 4.0), (3.0, 3.0, 4.0), lower, upper))
        self.assertTrue(segment_intersects_closed_box(
            (0.0, 1.0, 2.0), (1.0, 2.0, 3.0), lower, upper))
        self.assertFalse(segment_intersects_closed_box(
            (0.0, 1.0, 4.0), (3.0, 1.0, 4.0), lower, upper))
        self.assertFalse(segment_intersects_closed_box(
            (0.0, 3.0, 7.0), (3.0, 3.0, 7.0), lower, upper))

    def test_planned_trajectory_executes_safely(self, proc_output):
        rclpy.init()
        node = rclpy.create_node('static_avoidance_e2e_test')
        test_start = time.monotonic()
        path_planning_success = None
        trajectory_generation_success = None
        expected_segments = None
        observed_segments = []
        segment_switch_times = []
        preparation_complete_time = None
        trajectory_start_time = None
        mission_complete_time = None
        latest_setpoint = None
        latest_position = None
        previous_odom_position = None
        latest_speed = math.inf
        last_odom_time = None
        last_setpoint_time = None
        last_rpm_time = None
        odom_samples = 0
        setpoint_samples = 0
        rpm_samples = 0
        setpoints_after_complete = 0
        odom_after_complete = 0
        rpm_after_complete = 0
        collision_samples = 0
        max_tracking_error = 0.0
        minimum_sampled_clearance = math.inf
        health_errors = []
        previous_yaw = None
        previous_yaw_stamp = None
        max_adjacent_yaw_jump = 0.0
        max_yaw_reference_rate = 0.0
        tangent_error_sum = 0.0
        tangent_error_samples = 0

        def on_planning_success(message):
            nonlocal path_planning_success
            path_planning_success = bool(message.data)

        def on_generation_success(message):
            nonlocal trajectory_generation_success
            trajectory_generation_success = bool(message.data)

        def on_simplified_path(message):
            nonlocal expected_segments
            if message.header.frame_id != 'map' or len(message.poses) < 2:
                health_errors.append('simplified path is invalid')
                return
            expected_segments = list(range(len(message.poses) - 1))

        def on_segment(message):
            value = int(message.data)
            if not observed_segments or value != observed_segments[-1]:
                observed_segments.append(value)
                segment_switch_times.append(time.monotonic() - test_start)
                if observed_segments != list(range(len(observed_segments))):
                    health_errors.append(
                        f'planned trajectory segment skipped or regressed: {observed_segments}')

        def on_complete(message):
            nonlocal mission_complete_time
            if message.data and mission_complete_time is None:
                mission_complete_time = time.monotonic()
            elif not message.data and mission_complete_time is not None:
                health_errors.append('mission complete state regressed to false')

        def on_collision(message):
            nonlocal collision_samples
            collision_samples += 1
            if message.data:
                health_errors.append('/drone/environment/in_collision became true')

        def on_setpoint(message):
            nonlocal latest_setpoint, last_setpoint_time, setpoint_samples
            nonlocal setpoints_after_complete, preparation_complete_time
            nonlocal trajectory_start_time
            nonlocal previous_yaw, previous_yaw_stamp
            nonlocal max_adjacent_yaw_jump, max_yaw_reference_rate
            nonlocal tangent_error_sum, tangent_error_samples
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
            speed = norm3(values[3:6])
            acceleration = norm3(values[6:9])
            if trajectory_start_time is None and (speed > 1.0e-4 or acceleration > 1.0e-4):
                trajectory_start_time = time.monotonic()
                preparation_complete_time = trajectory_start_time
            latest_setpoint = message
            stamp = message.header.stamp.sec + 1.0e-9 * message.header.stamp.nanosec
            if previous_yaw is not None:
                jump = abs(message.yaw - previous_yaw)
                max_adjacent_yaw_jump = max(max_adjacent_yaw_jump, jump)
                stamp_dt = stamp - previous_yaw_stamp
                if stamp_dt > 0.0:
                    max_yaw_reference_rate = max(
                        max_yaw_reference_rate, jump / stamp_dt)
            previous_yaw = message.yaw
            previous_yaw_stamp = stamp
            horizontal_speed = math.hypot(message.velocity.x, message.velocity.y)
            remaining = math.dist(values[0:3], FINAL_TARGET)
            if horizontal_speed >= 0.15 and remaining >= 0.8:
                tangent = math.atan2(message.velocity.y, message.velocity.x)
                tangent_error_sum += abs(math.remainder(
                    message.yaw - tangent, 2.0 * math.pi))
                tangent_error_samples += 1
            last_setpoint_time = time.monotonic()
            setpoint_samples += 1
            if mission_complete_time is not None:
                setpoints_after_complete += 1

        def on_odometry(message):
            nonlocal latest_position, latest_speed, last_odom_time, odom_samples
            nonlocal odom_after_complete, max_tracking_error
            nonlocal minimum_sampled_clearance, previous_odom_position
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
                previous_odom_position = None
                health_errors.append('non-finite odometry')
                return
            position = values[0:3]
            latest_position = position
            latest_speed = norm3(values[7:10])
            last_odom_time = time.monotonic()
            odom_samples += 1
            if mission_complete_time is not None:
                odom_after_complete += 1
            for lower, upper in BASE_INFLATED_OBSTACLES:
                if inside_closed_box(position, lower, upper):
                    health_errors.append(
                        f'actual Odom entered base-inflated obstacle at {position}')
                if (previous_odom_position is not None and
                        segment_intersects_closed_box(
                            previous_odom_position, position, lower, upper)):
                    health_errors.append(
                        'actual Odom segment intersected base-inflated obstacle: '
                        f'{previous_odom_position} -> {position}')
                minimum_sampled_clearance = min(
                    minimum_sampled_clearance,
                    distance_to_box(position, lower, upper),
                )
            if (trajectory_start_time is not None and
                    mission_complete_time is None and latest_setpoint is not None):
                reference = (
                    latest_setpoint.position.x,
                    latest_setpoint.position.y,
                    latest_setpoint.position.z,
                )
                max_tracking_error = max(
                    max_tracking_error,
                    norm3(tuple(position[index] - reference[index] for index in range(3))),
                )
            previous_odom_position = position

        def on_motor_rpm(message):
            nonlocal last_rpm_time, rpm_samples, rpm_after_complete
            values = (
                message.m1_front_left_ccw_rpm,
                message.m2_rear_left_cw_rpm,
                message.m3_rear_right_ccw_rpm,
                message.m4_front_right_cw_rpm,
            )
            if not all(math.isfinite(value) for value in values):
                health_errors.append('non-finite motor RPM command')
                return
            last_rpm_time = time.monotonic()
            rpm_samples += 1
            if mission_complete_time is not None:
                rpm_after_complete += 1

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        subscriptions = [
            node.create_subscription(
                Bool, '/drone/planning/success', on_planning_success, latched_qos),
            node.create_subscription(
                Bool, '/drone/trajectory_generation/success',
                on_generation_success, latched_qos),
            node.create_subscription(
                Path, '/drone/simplified_path', on_simplified_path, latched_qos),
            node.create_subscription(
                UInt32, '/drone/planned_trajectory/current_segment', on_segment, 10),
            node.create_subscription(
                Bool, '/drone/planned_trajectory/complete', on_complete, 10),
            node.create_subscription(
                Bool, '/drone/environment/in_collision', on_collision, 10),
            node.create_subscription(
                TrajectorySetpoint, '/drone/trajectory_setpoint', on_setpoint, 10),
            node.create_subscription(Odometry, '/drone/odom', on_odometry, 10),
            node.create_subscription(
                MotorRPM, '/drone/motor_rpm_cmd', on_motor_rpm, 10),
        ]

        required_nodes = {
            'quadrotor_dynamics_node',
            'position_controller_node',
            'robot_state_publisher',
            'static_environment_node',
            'astar_planner_node',
            'planned_trajectory_node',
        }

        try:
            discovery_deadline = time.monotonic() + DISCOVERY_TIMEOUT
            while time.monotonic() < discovery_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if (required_nodes.issubset(set(node.get_node_names())) and
                        path_planning_success is not None and
                        trajectory_generation_success is not None and
                        expected_segments is not None and
                        latest_position is not None and latest_setpoint is not None and
                        rpm_samples > 0 and collision_samples > 0):
                    break
            else:
                self.fail(
                    f'static avoidance ROS graph was not ready; '
                    f'nodes={sorted(node.get_node_names())}, planning={path_planning_success}, '
                    f'generation={trajectory_generation_success}, odom={odom_samples}, '
                    f'setpoints={setpoint_samples}, rpm={rpm_samples}, '
                    f'collision={collision_samples}')

            self.assertTrue(path_planning_success, 'A* planning reported failure')
            self.assertTrue(
                trajectory_generation_success, 'planned trajectory generation reported failure')
            self.assertEqual(node.count_publishers('/drone/reference_path'), 1)
            self.assertEqual(node.count_publishers('/drone/trajectory_setpoint'), 1)

            mission_deadline = test_start + MISSION_TIMEOUT
            while time.monotonic() < mission_deadline and mission_complete_time is None:
                rclpy.spin_once(node, timeout_sec=0.01)
                now = time.monotonic()
                if health_errors:
                    self.fail(health_errors[0])
                if last_odom_time is not None and now - last_odom_time > 1.0:
                    self.fail('/drone/odom stopped')
                if last_setpoint_time is not None and now - last_setpoint_time > 1.0:
                    self.fail('/drone/trajectory_setpoint stopped')
                if last_rpm_time is not None and now - last_rpm_time > 1.0:
                    self.fail('/drone/motor_rpm_cmd stopped')
                if not required_nodes.issubset(set(node.get_node_names())):
                    self.fail(f'a required node exited: {sorted(node.get_node_names())}')

            self.assertIsNotNone(
                mission_complete_time,
                f'mission did not complete; segments={observed_segments}, '
                f'position={latest_position}, speed={latest_speed}')

            observation_deadline = mission_complete_time + POST_COMPLETE_OBSERVATION
            while time.monotonic() < observation_deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
                if health_errors:
                    self.fail(health_errors[0])
                self.assertTrue(required_nodes.issubset(set(node.get_node_names())))

            now = time.monotonic()
            self.assertLess(now - last_odom_time, 1.0)
            self.assertLess(now - last_setpoint_time, 1.0)
            self.assertLess(now - last_rpm_time, 1.0)
            final_setpoint_position = (
                latest_setpoint.position.x,
                latest_setpoint.position.y,
                latest_setpoint.position.z,
            )
            final_setpoint_velocity = (
                latest_setpoint.velocity.x,
                latest_setpoint.velocity.y,
                latest_setpoint.velocity.z,
            )
            final_setpoint_acceleration = (
                latest_setpoint.acceleration.x,
                latest_setpoint.acceleration.y,
                latest_setpoint.acceleration.z,
            )
            final_error = norm3(tuple(
                latest_position[index] - FINAL_TARGET[index] for index in range(3)))
            controller_output = b''.join(event.text for event in proc_output)
            saturation_true_count = controller_output.count(b'saturated=true')
            summary = (
                'static_avoidance_e2e: '
                f'preparation_complete_time={preparation_complete_time - test_start:.3f}s '
                f'trajectory_start_time={trajectory_start_time - test_start:.3f}s '
                f'segments={observed_segments} '
                f'segment_switch_times={[round(value, 3) for value in segment_switch_times]} '
                f'mission_complete_time={mission_complete_time - test_start:.3f}s '
                f'max_tracking_error={max_tracking_error:.6f}m '
                f'minimum_sampled_clearance={minimum_sampled_clearance:.6f}m '
                f'controller_saturated_true_logs={saturation_true_count} '
                f'yaw_mode={YAW_MODE} max_adjacent_yaw_jump={max_adjacent_yaw_jump:.6f}rad '
                f'max_yaw_reference_rate={max_yaw_reference_rate:.6f}rad/s '
                f'mean_tangent_yaw_error='
                f'{tangent_error_sum / max(tangent_error_samples, 1):.6f}rad '
                f'final_yaw_error='
                f'{abs(math.remainder(latest_setpoint.yaw, 2.0 * math.pi)):.6f}rad '
                f'final_position_error={final_error:.6f}m '
                f'final_speed={latest_speed:.6f}m/s '
                f'odom_samples={odom_samples} setpoint_samples={setpoint_samples} '
                f'rpm_samples={rpm_samples} collision_samples={collision_samples} '
                f'post_complete_samples=(odom={odom_after_complete}, '
                f'setpoint={setpoints_after_complete}, rpm={rpm_after_complete})'
            )
            print(summary, flush=True)

            self.assertIsNotNone(preparation_complete_time, summary)
            self.assertIsNotNone(trajectory_start_time, summary)
            self.assertEqual(observed_segments, expected_segments, summary)
            self.assertLess(max_tracking_error, 0.10, summary)
            self.assertGreater(minimum_sampled_clearance, 0.05, summary)
            self.assertEqual(saturation_true_count, 0, summary)
            self.assertLess(max_adjacent_yaw_jump, 0.10, summary)
            self.assertLessEqual(max_yaw_reference_rate, 0.82, summary)
            self.assertLess(
                abs(math.remainder(latest_setpoint.yaw, 2.0 * math.pi)), 0.05, summary)
            if YAW_MODE == 'path_tangent':
                self.assertGreater(tangent_error_samples, 100, summary)
                self.assertLess(tangent_error_sum / tangent_error_samples, 0.35, summary)
            else:
                self.assertLess(max_adjacent_yaw_jump, 1.0e-9, summary)
            self.assertEqual(latest_setpoint.header.frame_id, 'map', summary)
            self.assertLess(
                norm3(tuple(
                    final_setpoint_position[index] - FINAL_TARGET[index]
                    for index in range(3))),
                1.0e-9,
                summary,
            )
            self.assertLess(norm3(final_setpoint_velocity), 1.0e-9, summary)
            self.assertLess(norm3(final_setpoint_acceleration), 1.0e-9, summary)
            self.assertLess(final_error, 0.20, summary)
            self.assertLess(latest_speed, 0.15, summary)
            self.assertGreater(setpoints_after_complete, 100, summary)
            self.assertGreater(odom_after_complete, 300, summary)
            self.assertGreater(rpm_after_complete, 200, summary)
            self.assertFalse(health_errors, '; '.join(health_errors))
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestStaticAvoidanceShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        process_names = proc_info.process_names()
        for expected in (
                'quadrotor_dynamics_node',
                'position_controller_node',
                'robot_state_publisher',
                'static_environment_node',
                'astar_planner_node',
                'planned_trajectory_node'):
            self.assertTrue(
                any(expected in name for name in process_names),
                f'{expected} was not launched: {process_names}')
        launch_testing.asserts.assertExitCodes(proc_info)
