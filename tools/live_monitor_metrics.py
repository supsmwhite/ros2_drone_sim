#!/usr/bin/env python3
"""ROS-independent calculations shared by the live assessment monitor tests."""

import math


def quaternion_yaw(quaternion):
    """Return normalized quaternion yaw, or None for an invalid quaternion."""
    x, y, z, w = map(float, quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-12:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def vector_error(actual, target):
    """Return target-minus-actual components, 3D distance, and XY distance."""
    if actual is None or target is None:
        return None
    error = tuple(float(target[i]) - float(actual[i]) for i in range(3))
    if not all(math.isfinite(value) for value in error):
        return None
    return {
        "vector": error,
        "distance": math.sqrt(sum(value * value for value in error)),
        "horizontal_distance": math.hypot(error[0], error[1]),
    }


def wrapped_error(target, actual):
    """Return target-minus-actual angular error wrapped to [-pi, pi]."""
    if target is None or actual is None:
        return None
    target, actual = float(target), float(actual)
    if not math.isfinite(target) or not math.isfinite(actual):
        return None
    return math.remainder(target - actual, 2.0 * math.pi)


def obstacle_boxes(flat_obstacles):
    """Convert [center, size] obstacle groups to axis-aligned boxes."""
    values = list(map(float, flat_obstacles))
    if len(values) % 6:
        raise ValueError("obstacles must contain groups of six values")
    boxes = []
    for index in range(0, len(values), 6):
        x, y, z, sx, sy, sz = values[index:index + 6]
        boxes.append(((x - sx / 2.0, y - sy / 2.0, z - sz / 2.0),
                      (x + sx / 2.0, y + sy / 2.0, z + sz / 2.0)))
    return boxes


def point_box_distance(point, box):
    lower, upper = box
    return math.sqrt(sum(
        max(lower[index] - point[index], 0.0, point[index] - upper[index]) ** 2
        for index in range(3)))


def safety_clearance(position, boxes, safety_radius):
    """Match Recorder semantics: raw obstacle distance minus safety radius."""
    if position is None or not boxes:
        return None
    raw_distance = min(point_box_distance(position, box) for box in boxes)
    return raw_distance, raw_distance - float(safety_radius)


def format_duration(seconds):
    if seconds is None or not math.isfinite(seconds) or seconds < 0.0:
        return "--"
    minutes, remainder = divmod(seconds, 60.0)
    if minutes < 1.0:
        return f"{remainder:5.1f} s"
    return f"{int(minutes):02d}:{remainder:04.1f}"


class OnlineExtrema:
    """Accumulate the live extrema reported by the formal assessment tooling."""

    def __init__(self):
        self.sample_count = 0
        self.maximum_goal_error = None
        self.maximum_horizontal_error = None
        self.maximum_tracking_error = None
        self.maximum_absolute_yaw_error = None
        self.maximum_speed = None
        self.minimum_safety_clearance = None
        self.maximum_force = None
        self.saturation_observed = [False, False, False, False]

    @staticmethod
    def _maximum(current, value):
        if value is None or not math.isfinite(value):
            return current
        return value if current is None else max(current, value)

    @staticmethod
    def _minimum(current, value):
        if value is None or not math.isfinite(value):
            return current
        return value if current is None else min(current, value)

    def observe(self, *, goal_error=None, horizontal_error=None, tracking_error=None,
                yaw_error=None, speed=None, safety_clearance_m=None,
                force_magnitude=None, saturation=None):
        self.sample_count += 1
        self.maximum_goal_error = self._maximum(self.maximum_goal_error, goal_error)
        self.maximum_horizontal_error = self._maximum(
            self.maximum_horizontal_error, horizontal_error)
        self.maximum_tracking_error = self._maximum(
            self.maximum_tracking_error, tracking_error)
        self.maximum_absolute_yaw_error = self._maximum(
            self.maximum_absolute_yaw_error,
            None if yaw_error is None else abs(yaw_error))
        self.maximum_speed = self._maximum(self.maximum_speed, speed)
        self.minimum_safety_clearance = self._minimum(
            self.minimum_safety_clearance, safety_clearance_m)
        self.maximum_force = self._maximum(self.maximum_force, force_magnitude)
        if saturation is not None:
            self.saturation_observed = [
                observed or bool(current)
                for observed, current in zip(self.saturation_observed, saturation)]

    def snapshot(self):
        return {
            "sample_count": self.sample_count,
            "maximum_goal_error": self.maximum_goal_error,
            "maximum_horizontal_error": self.maximum_horizontal_error,
            "maximum_tracking_error": self.maximum_tracking_error,
            "maximum_absolute_yaw_error": self.maximum_absolute_yaw_error,
            "maximum_speed": self.maximum_speed,
            "minimum_safety_clearance": self.minimum_safety_clearance,
            "maximum_force": self.maximum_force,
            "saturation_observed": tuple(self.saturation_observed),
        }
