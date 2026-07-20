#!/usr/bin/env python3
"""ROS-independent state, path, and metric helpers for assessment tooling."""

import math


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


class PathHistory:
    """Retain unique planning segments and the latest full actual trajectory."""

    def __init__(self):
        self.actual = []
        self.segments = {name: [] for name in ("planned", "simplified", "reference")}
        self.clear_events = []

    def add(self, name, points, received_time_s, goal_index=None):
        normalized = [[float(v) for v in point] for point in points]
        if name == "actual":
            if len(normalized) >= len(self.actual):
                self.actual = normalized
            return False
        if not normalized:
            self.clear_events.append({"path": name, "received_time_s": received_time_s,
                                      "goal_index": goal_index})
            return False
        existing = self.segments[name]
        if any(item["points"] == normalized for item in existing):
            return False
        existing.append({"sequence": len(existing), "goal_index": goal_index,
                         "received_time_s": received_time_s, "points": normalized})
        return True

    def as_dict(self):
        return {"schema_version": 2, "actual": self.actual,
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
