#!/usr/bin/env python3
"""ROS-independent state, path, and metric helpers for assessment tooling."""

import math
from pathlib import Path


def prepare_output_directory(path, allow_overwrite=False, run_status="candidate"):
    """Create an output directory, refusing implicit or final-result overwrite."""
    output = Path(path)
    if output.exists() and not output.is_dir():
        raise FileExistsError(f"assessment output is not a directory: {output}")
    nonempty = output.exists() and any(output.iterdir())
    if nonempty and (run_status == "final" or not allow_overwrite):
        reason = "final assessment results cannot be overwritten" if run_status == "final" else \
            "assessment output already exists and is non-empty"
        raise FileExistsError(f"{reason}: {output}")
    output.mkdir(parents=True, exist_ok=True)
    return output


def normalized_target(position, yaw_rad=None):
    values = [float(value) for value in position]
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("target position must contain three finite values")
    yaw = None if yaw_rad is None else float(yaw_rad)
    if yaw is not None and not math.isfinite(yaw):
        raise ValueError("target yaw must be finite")
    return {"position": values, "yaw_rad": yaw}


def targets_match(observed, expected, position_tolerance=1e-6, yaw_tolerance=1e-6):
    if any(abs(a - b) > position_tolerance
           for a, b in zip(observed["position"], expected["position"])):
        return False
    if expected.get("yaw_rad") is None:
        return True
    if observed.get("yaw_rad") is None:
        return False
    yaw_error = math.remainder(observed["yaw_rad"] - expected["yaw_rad"], 2 * math.pi)
    return abs(yaw_error) <= yaw_tolerance


def target_sequences_match(observed, expected):
    return len(observed) == len(expected) and all(
        targets_match(actual, required) for actual, required in zip(observed, expected))


def rotate_body_velocity_to_map(quaternion, velocity):
    """Rotate a body-frame vector with a normalized body-to-map quaternion."""
    x, y, z, w = map(float, quaternion)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-12:
        return [math.nan, math.nan, math.nan]
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    vx, vy, vz = map(float, velocity)
    return [
        (1 - 2*(y*y + z*z))*vx + 2*(x*y - z*w)*vy + 2*(x*z + y*w)*vz,
        2*(x*y + z*w)*vx + (1 - 2*(x*x + z*z))*vy + 2*(y*z - x*w)*vz,
        2*(x*z - y*w)*vx + 2*(y*z + x*w)*vy + (1 - 2*(x*x + y*y))*vz,
    ]


def wrap_to_pi(angle):
    if angle is None or not math.isfinite(angle):
        return None
    return math.remainder(angle, 2 * math.pi)


def yaw_error_metrics(actual_yaws, reference_yaws):
    errors = [wrap_to_pi(reference - actual)
              for actual, reference in zip(actual_yaws, reference_yaws)
              if math.isfinite(actual) and math.isfinite(reference)]
    if not errors:
        return {"status": "unavailable", "sample_count": 0,
                "final_error_rad": None, "rms_error_rad": None,
                "maximum_absolute_error_rad": None}
    return {"status": "available", "sample_count": len(errors),
            "final_error_rad": errors[-1],
            "rms_error_rad": math.sqrt(sum(value * value for value in errors) / len(errors)),
            "maximum_absolute_error_rad": max(abs(value) for value in errors)}


def mission_relative_time(event_time, mission_start):
    if event_time is None or mission_start is None:
        return None
    difference = event_time - mission_start
    if difference < -1e-9:
        return None
    return 0.0 if abs(difference) <= 1e-9 else difference


def require_nonnegative_mission_times(values):
    if any(value is not None and math.isfinite(value) and value < -1e-9 for value in values):
        raise ValueError("negative mission_time_s detected")


def path_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:])) if len(points) > 1 else 0.0


def point_box_distance(point, box):
    lower, upper = box
    return math.sqrt(sum(max(lower[i] - point[i], 0.0, point[i] - upper[i]) ** 2
                         for i in range(3)))


def projection_overshoot(samples, hover=False):
    if hover:
        return max(0.0, max(p[2] - g[2] for p, g in samples)), None
    start, goal = samples[0][0], samples[-1][1]
    distance = math.dist(start, goal)
    if distance < 1e-12:
        return 0.0, None
    direction = [(goal[i] - start[i]) / distance for i in range(3)]
    value = max(0.0, max(sum((p[i] - goal[i]) * direction[i] for i in range(3))
                         for p, _ in samples))
    return value, 100.0 * value / distance


def longest_true_duration(times, flags):
    start = None
    longest = 0.0
    for time_s, flag in zip(times, flags):
        if flag:
            start = time_s if start is None else start
            longest = max(longest, time_s - start)
        else:
            start = None
    return longest


def held_condition_start(times, flags, hold_time):
    """Return the first condition start that remains true for the hold duration."""
    start = None
    for time_s, flag in zip(times, flags):
        if flag:
            start = time_s if start is None else start
            if time_s - start >= hold_time:
                return start
        else:
            start = None
    return None


def navigation_phase_start(paths, activation_events=()):
    """Return mission time/source of the first valid formal navigation artifact."""
    for name in ("reference", "planned", "simplified"):
        valid = [item for item in paths.get(name + "_segments", [])
                 if item.get("mission_time_s") is not None and
                 item.get("goal_index") is not None and item.get("goal_index") >= 0 and
                 item.get("points")]
        if valid:
            first = min(valid, key=lambda item: item["mission_time_s"])
            return float(first["mission_time_s"]), name + "_segment"
    valid_events = [event for event in activation_events
                    if event.get("mission_time_s") is not None and
                    event.get("source") == "navigation_goal_index"]
    if valid_events:
        first = min(valid_events, key=lambda event: event["mission_time_s"])
        return float(first["mission_time_s"]), "navigation_goal_index"
    return None, None


def phased_tracking_metrics(samples, phase_start):
    """Split finite (mission_time, tracking_error) pairs at navigation phase start."""
    full = [(time_s, error) for time_s, error in samples
            if time_s is not None and error is not None and
            math.isfinite(time_s) and math.isfinite(error)]
    takeoff = [] if phase_start is None else [item for item in full if item[0] < phase_start]
    navigation = [] if phase_start is None else [item for item in full if item[0] >= phase_start]

    def stats(items, include_final=False):
        errors = [item[1] for item in items]
        result = {
            "sample_count": len(errors) if errors else None,
            "max_error_m": max(errors) if errors else None,
            "rms_error_m": (math.sqrt(sum(value * value for value in errors) / len(errors))
                            if errors else None),
        }
        if include_final:
            result["final_error_m"] = errors[-1] if errors else None
        return result
    return stats(full), stats(takeoff), stats(navigation, include_final=True)


def goal_timing(goal_count, activation_events, complete_time):
    """Build activation, arrival, and duration arrays without inventing timestamps."""
    activations = [None] * goal_count
    for event in activation_events:
        index = event.get("goal_index")
        time_s = event.get("mission_time_s")
        if (isinstance(index, int) and 0 <= index < goal_count and time_s is not None and
                math.isfinite(time_s) and time_s >= 0 and activations[index] is None):
            activations[index] = float(time_s)
    arrivals = [None] * goal_count
    for index in range(goal_count - 1):
        arrivals[index] = activations[index + 1]
    if goal_count:
        arrivals[-1] = (float(complete_time) if complete_time is not None and
                        math.isfinite(complete_time) and complete_time >= 0 else None)
    durations = [None if start is None or end is None else end - start
                 for start, end in zip(activations, arrivals)]
    return activations, arrivals, durations


def directional_disturbance_metrics(samples, force_threshold=1e-6):
    """Return mean force direction, peak displacement, and post-release reverse overshoot.

    Samples are dictionaries containing actual/goal/force xy and force_active.
    """
    active = [sample for sample in samples if sample["force_active"]]
    if not active:
        return None, None, None, []
    mean_force = [sum(sample["force"][axis] for sample in active) / len(active)
                  for axis in (0, 1)]
    norm = math.hypot(*mean_force)
    if norm < force_threshold:
        return None, None, None, []
    unit = [value / norm for value in mean_force]
    signed = [sum((sample["actual"][axis] - sample["goal"][axis]) * unit[axis]
                  for axis in (0, 1)) for sample in samples]
    peak = max(signed)
    active_indices = [index for index, sample in enumerate(samples) if sample["force_active"]]
    after = signed[max(active_indices) + 1:]
    reverse = max(0.0, -min(after)) if after else None
    return mean_force, peak, reverse, signed


class PathHistory:
    """Retain unique planning segments and the latest full actual trajectory."""

    def __init__(self):
        self.actual = []
        self.segments = {name: [] for name in ("planned", "simplified", "reference")}
        self.clear_events = []

    def add(self, name, points, recording_time_s, mission_time_s=None, goal_index=None):
        normalized = [[float(v) for v in point] for point in points]
        if name == "actual":
            if len(normalized) >= len(self.actual):
                self.actual = normalized
            return False
        if not normalized:
            self.clear_events.append({"path": name, "recording_time_s": recording_time_s,
                                      "mission_time_s": mission_time_s,
                                      "goal_index": goal_index})
            return False
        existing = self.segments[name]
        if any(item["points"] == normalized and item["goal_index"] == goal_index
               for item in existing):
            return False
        existing.append({"sequence": len(existing), "goal_index": goal_index,
                         "recording_time_s": recording_time_s,
                         "mission_time_s": mission_time_s, "points": normalized})
        return True

    def as_dict(self):
        return {"schema_version": 3, "actual": self.actual,
                "planned_segments": self.segments["planned"],
                "simplified_segments": self.segments["simplified"],
                "reference_segments": self.segments["reference"],
                "clear_events": self.clear_events}


class ExperimentStopController:
    """Pure experiment-specific stop state machine."""

    def __init__(self, experiment, steady_window=3.0, arrival_position=0.10,
                 arrival_speed=0.08, arrival_hold=1.0, recovery_position=0.10,
                 recovery_speed=0.08, recovery_hold=1.0,
                 failure_observation_window=2.0):
        self.experiment = experiment
        self.steady_window = steady_window
        self.arrival_position = arrival_position
        self.arrival_speed = arrival_speed
        self.arrival_hold = arrival_hold
        self.recovery_position = recovery_position
        self.recovery_speed = recovery_speed
        self.recovery_hold = recovery_hold
        self.failure_observation_window = failure_observation_window
        self.state = "WAITING_FOR_MISSION"
        self.candidate_time = None
        self.confirmed_time = None
        self.finish_time = None
        self.stop_reason = None
        self.force_start_time = None
        self.force_release_time = None
        self.failure_reason = None

    def update(self, now, mission_started=False, goal_error=None, horizontal_error=None,
               speed=None, mission_complete=False, navigation_complete=False,
               navigation_success=None, interactive_active=None, force_active=None,
               failure_reason=None):
        events = []
        if self.experiment in ("hover", "single_goal"):
            if mission_started and self.state == "WAITING_FOR_MISSION":
                self.state = "WAITING_FOR_ARRIVAL"
            eligible = (goal_error is not None and speed is not None and
                        goal_error < self.arrival_position and speed < self.arrival_speed)
            if self.state == "WAITING_FOR_ARRIVAL":
                if eligible:
                    self.candidate_time = now if self.candidate_time is None else self.candidate_time
                    if now - self.candidate_time >= self.arrival_hold:
                        self.confirmed_time = self.candidate_time
                        self.finish_time = now + self.steady_window
                        self.state = "STEADY_WINDOW"
                        events.append("arrival_confirmed")
                else:
                    self.candidate_time = None
            if self.state == "STEADY_WINDOW" and now >= self.finish_time:
                self.stop_reason = "arrival_and_steady_window_complete"
        elif self.experiment == "multi_goal":
            if mission_started and self.state == "WAITING_FOR_MISSION":
                self.state = "WAITING_FOR_MISSION_COMPLETE"
            if mission_complete and self.state == "WAITING_FOR_MISSION_COMPLETE":
                self.confirmed_time = now; self.finish_time = now + self.steady_window
                self.state = "STEADY_WINDOW"; events.append("mission_complete_confirmed")
            if self.state == "STEADY_WINDOW" and now >= self.finish_time:
                self.stop_reason = "mission_complete_and_steady_window_complete"
        elif self.experiment == "navigation":
            if interactive_active is True and self.state == "WAITING_FOR_MISSION":
                self.state = "WAITING_FOR_NAVIGATION_COMPLETE"
            if navigation_complete and self.state == "WAITING_FOR_NAVIGATION_COMPLETE":
                self.confirmed_time = now; self.finish_time = now + self.steady_window
                self.state = "STEADY_WINDOW"; events.append("navigation_complete_confirmed")
                self.stop_reason = None
                self._navigation_success = navigation_success
            if self.state == "STEADY_WINDOW" and now >= self.finish_time:
                self.stop_reason = ("navigation_success_and_steady_window_complete" if
                                    self._navigation_success else
                                    "navigation_failure_and_observation_complete")
        elif self.experiment == "disturbance":
            if self.state == "WAITING_FOR_MISSION" and mission_started:
                self.state = "WAITING_FOR_FORCE"
            if self.state == "WAITING_FOR_FORCE" and force_active is True:
                self.force_start_time = now; self.state = "FORCE_ACTIVE"
                events.append("external_force_started")
            if self.state == "FORCE_ACTIVE" and force_active is False:
                self.force_release_time = now; self.state = "RECOVERING"
                self.candidate_time = None; events.append("external_force_released")
            eligible = (horizontal_error is not None and speed is not None and
                        horizontal_error < self.recovery_position and speed < self.recovery_speed)
            if self.state == "RECOVERING":
                if eligible:
                    self.candidate_time = now if self.candidate_time is None else self.candidate_time
                    if now - self.candidate_time >= self.recovery_hold:
                        self.confirmed_time = self.candidate_time
                        self.finish_time = now + self.steady_window
                        self.state = "STEADY_WINDOW"; events.append("recovery_confirmed")
                else:
                    self.candidate_time = None
            if self.state == "STEADY_WINDOW" and now >= self.finish_time:
                self.stop_reason = "disturbance_recovery_and_steady_window_complete"
        elif self.experiment == "failure_case":
            if mission_started and self.state == "WAITING_FOR_MISSION":
                self.state = "WAITING_FOR_FAILURE"
            if failure_reason and interactive_active is not True and self.state in (
                    "WAITING_FOR_MISSION", "WAITING_FOR_FAILURE"):
                self.failure_reason = failure_reason; self.confirmed_time = now
                self.finish_time = now + self.failure_observation_window
                self.state = "FAILURE_OBSERVATION"; events.append("failure_detected")
            if self.state == "FAILURE_OBSERVATION" and now >= self.finish_time:
                self.stop_reason = "failure_detected_and_observation_complete"
        return events

    def timeout(self):
        self.stop_reason = "timeout_in_state_" + self.state.lower()

    @property
    def stopped(self):
        return self.stop_reason is not None
