#!/usr/bin/env python3
"""Offline layered navigation geometry, clearance, corner, and tracking diagnosis."""

import argparse
import csv
import json
import math
import re
import shutil
from pathlib import Path

import yaml


LAYERS = ("planned", "simplified", "reference", "actual")
TRAJECTORY_RE = re.compile(
    r"ordered goal (?P<goal>\d+) trajectory ready:.*?"
    r"duration=(?P<duration>[0-9.]+) s velocity_scale=(?P<velocity>[0-9.]+) "
    r"duration_scale=(?P<scale>[0-9.]+)")


def finite_number(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def finite_points(points):
    return [[float(value) for value in point[:3]] for point in points
            if len(point) >= 3 and all(finite_number(value) for value in point[:3])]


def finite_indexed_points(points):
    """Return (source index, xyz) pairs without losing row alignment."""
    return [(index, [float(value) for value in point[:3]])
            for index, point in enumerate(points)
            if len(point) >= 3 and all(finite_number(value) for value in point[:3])]


def cumulative_lengths(points):
    result = [0.0]
    for first, second in zip(points, points[1:]):
        result.append(result[-1] + math.dist(first, second))
    return result


def resample_polyline(points, step=0.02):
    """Resample a polyline at fixed arc length, retaining both endpoints."""
    if not finite_number(step) or not 0.01 <= float(step) <= 0.05:
        raise ValueError("spatial sample step must be within [0.01, 0.05] m")
    points = finite_points(points)
    if not points:
        return []
    compact = [points[0]]
    for point in points[1:]:
        if math.dist(point, compact[-1]) > 1e-12:
            compact.append(point)
    if len(compact) == 1:
        return compact
    arcs = cumulative_lengths(compact)
    total = arcs[-1]
    targets = [index * float(step) for index in range(int(total / float(step)) + 1)]
    if not targets or total - targets[-1] > 1e-12:
        targets.append(total)
    else:
        targets[-1] = total
    output, segment = [], 0
    for target in targets:
        while segment + 1 < len(arcs) - 1 and arcs[segment + 1] < target:
            segment += 1
        length = arcs[segment + 1] - arcs[segment]
        ratio = 0.0 if length <= 1e-12 else (target - arcs[segment]) / length
        output.append([compact[segment][axis] + ratio *
                       (compact[segment + 1][axis] - compact[segment][axis])
                       for axis in range(3)])
    return output


def turning_angle_degrees(first, middle, last):
    incoming = [middle[index] - first[index] for index in range(3)]
    outgoing = [last[index] - middle[index] for index in range(3)]
    left = math.sqrt(sum(value * value for value in incoming))
    right = math.sqrt(sum(value * value for value in outgoing))
    if left <= 1e-12 or right <= 1e-12:
        return 0.0
    cosine = sum(a * b for a, b in zip(incoming, outgoing)) / (left * right)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def identify_corners(points, angle_threshold=25.0, merge_distance=0.30):
    """Return merged corner regions identified from simplified-path vertices."""
    points = finite_points(points)
    if len(points) < 3:
        return []
    arcs = cumulative_lengths(points)
    candidates = []
    for index in range(1, len(points) - 1):
        angle = turning_angle_degrees(points[index - 1], points[index], points[index + 1])
        if angle > angle_threshold:
            candidates.append({"vertex_index": index, "position": points[index],
                               "angle_deg": angle, "arc_length_m": arcs[index],
                               "previous_segment_length_m": math.dist(points[index - 1], points[index]),
                               "next_segment_length_m": math.dist(points[index], points[index + 1])})
    groups = []
    for candidate in candidates:
        if groups and candidate["arc_length_m"] - groups[-1][-1]["arc_length_m"] < merge_distance:
            groups[-1].append(candidate)
        else:
            groups.append([candidate])
    merged = []
    for number, group in enumerate(groups, 1):
        primary = max(group, key=lambda item: item["angle_deg"])
        merged.append({**primary, "corner_index": number,
                       "merged_candidate_count": len(group),
                       "region_start_arc_m": group[0]["arc_length_m"],
                       "region_end_arc_m": group[-1]["arc_length_m"]})
    return merged


def identify_segment_corners(paths, layer="simplified", angle_threshold=25.0,
                             merge_distance=0.30):
    """Identify corners inside plans, excluding stop-and-replan joins."""
    corners = []
    global_offset = 0
    previous_endpoint = None
    for segment in sorted(paths.get(layer + "_segments", []),
                          key=lambda item: item["sequence"]):
        points = finite_points(segment.get("points", []))
        drop_first = bool(previous_endpoint is not None and points and
                          math.dist(previous_endpoint, points[0]) <= 1e-9)
        for corner in identify_corners(points, angle_threshold, merge_distance):
            corner["vertex_index"] += global_offset - int(drop_first)
            corner["goal_index"] = segment.get("goal_index")
            corner["source_path_sequence"] = segment.get("sequence")
            corner["corner_index"] = len(corners) + 1
            corners.append(corner)
        if points:
            global_offset += len(points) - int(drop_first)
            previous_endpoint = points[-1]
    return corners


def point_aabb_distance(point, box):
    lower, upper = box
    return math.sqrt(sum(max(lower[index] - point[index], 0.0,
                             point[index] - upper[index]) ** 2 for index in range(3)))


def nearest_point_on_polyline(point, polyline):
    """Return distance, nearest point, arc length, and segment index."""
    points = finite_points(polyline)
    if not points:
        return math.nan, None, math.nan, None
    if len(points) == 1:
        return math.dist(point, points[0]), points[0], 0.0, 0
    best = (math.inf, None, 0.0, 0)
    arc = 0.0
    for index, (start, end) in enumerate(zip(points, points[1:])):
        delta = [end[axis] - start[axis] for axis in range(3)]
        length_squared = sum(value * value for value in delta)
        ratio = 0.0 if length_squared <= 1e-24 else max(0.0, min(1.0,
            sum((point[axis] - start[axis]) * delta[axis] for axis in range(3)) /
            length_squared))
        nearest = [start[axis] + ratio * delta[axis] for axis in range(3)]
        distance = math.dist(point, nearest)
        length = math.sqrt(length_squared)
        if distance < best[0]:
            best = (distance, nearest, arc + ratio * length, index)
        arc += length
    return best


def cross_track_errors(actual_points, reference_points):
    return [item[0] for item in project_points_on_polyline(
        finite_points(actual_points), reference_points)]


def project_points_on_polyline(points, polyline, chunk_size=256):
    """Project many points exactly, using bounded NumPy chunks when available."""
    points = finite_points(points)
    polyline = finite_points(polyline)
    if not points:
        return []
    if len(polyline) < 2:
        return [nearest_point_on_polyline(point, polyline) for point in points]
    try:
        import numpy as np
    except ImportError:
        return [nearest_point_on_polyline(point, polyline) for point in points]
    reference = np.asarray(polyline, dtype=float)
    starts = reference[:-1]
    deltas = reference[1:] - starts
    length_squared = np.einsum("ij,ij->i", deltas, deltas)
    lengths = np.sqrt(length_squared)
    segment_arcs = np.concatenate(([0.0], np.cumsum(lengths[:-1])))
    safe_length_squared = np.where(length_squared > 1e-24, length_squared, 1.0)
    output = []
    for offset in range(0, len(points), chunk_size):
        batch = np.asarray(points[offset:offset + chunk_size], dtype=float)
        differences = batch[:, None, :] - starts[None, :, :]
        ratios = np.clip(np.einsum("bsi,si->bs", differences, deltas) /
                         safe_length_squared[None, :], 0.0, 1.0)
        ratios[:, length_squared <= 1e-24] = 0.0
        nearest = starts[None, :, :] + ratios[:, :, None] * deltas[None, :, :]
        residual = batch[:, None, :] - nearest
        distance_squared = np.einsum("bsi,bsi->bs", residual, residual)
        best_segments = np.argmin(distance_squared, axis=1)
        for row, segment in enumerate(best_segments):
            segment = int(segment)
            ratio = float(ratios[row, segment])
            nearest_point = nearest[row, segment].tolist()
            output.append((math.sqrt(float(distance_squared[row, segment])),
                           nearest_point,
                           float(segment_arcs[segment] + ratio * lengths[segment]),
                           segment))
    return output


def clearance_profile(points, boxes, safety_radius, step=0.02):
    sampled = resample_polyline(points, step) if points else []
    arcs = cumulative_lengths(sampled) if sampled else []
    rows = []
    for index, point in enumerate(sampled):
        distances = [point_aabb_distance(point, box) for box in boxes]
        raw = min(distances, default=math.inf)
        rows.append({"sample_index": index, "arc_length_m": arcs[index],
                     "x": point[0], "y": point[1], "z": point[2],
                     "raw_obstacle_distance_m": raw,
                     "safety_clearance_m": raw - safety_radius,
                     "obstacle_index": distances.index(raw) if distances else None})
    return rows


def percentile(values, percentage):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentage / 100.0
    lower = int(math.floor(position)); upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def profile_summary(name, original, profile, threshold):
    values = [row["safety_clearance_m"] for row in profile]
    if not values:
        return {"layer": name, "original_point_count": len(original),
                "resampled_point_count": 0, "path_length_m": 0.0,
                "minimum_safety_clearance_m": None,
                "minimum_clearance_position": None}
    minimum = min(profile, key=lambda row: row["safety_clearance_m"])
    below_length = sum(profile[index + 1]["arc_length_m"] - profile[index]["arc_length_m"]
                       for index in range(len(profile) - 1)
                       if (profile[index]["safety_clearance_m"] < threshold or
                           profile[index + 1]["safety_clearance_m"] < threshold))
    length = profile[-1]["arc_length_m"]
    return {"layer": name, "original_point_count": len(original),
            "resampled_point_count": len(profile), "path_length_m": length,
            "minimum_raw_obstacle_distance_m": minimum["raw_obstacle_distance_m"],
            "minimum_safety_clearance_m": minimum["safety_clearance_m"],
            "clearance_p05_m": percentile(values, 5.0),
            "mean_safety_clearance_m": sum(values) / len(values),
            "minimum_clearance_position": [minimum[key] for key in ("x", "y", "z")],
            "minimum_clearance_arc_length_m": minimum["arc_length_m"],
            "minimum_clearance_obstacle_index": minimum["obstacle_index"],
            "below_threshold_m": threshold,
            "below_threshold_length_ratio": below_length / length if length > 0 else 0.0}


def finite_csv_value(row, name):
    value = row.get(name)
    return float(value) if finite_number(value) else None


def read_samples(path):
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def derivative(times, vectors, min_dt=1e-4, max_dt=0.10):
    """First differences, filtering non-finite and implausible timestamps."""
    output = [None] * len(times)
    previous = None
    for index, (time_s, vector) in enumerate(zip(times, vectors)):
        if time_s is None or vector is None or not finite_number(time_s) or not all(
                finite_number(value) for value in vector):
            continue
        if previous is not None:
            old_index, old_time, old_vector = previous
            dt = float(time_s) - old_time
            if min_dt <= dt <= max_dt:
                output[index] = [(float(vector[axis]) - old_vector[axis]) / dt
                                 for axis in range(len(vector))]
        previous = (index, float(time_s), [float(value) for value in vector])
    return output


def held_signal_derivative(times, vectors, min_dt=1e-4, max_dt=0.10,
                           change_tolerance=1e-12):
    """Differentiate a lower-rate signal repeated in higher-rate sample rows."""
    output = [None] * len(times)
    previous = None
    for index, (time_s, vector) in enumerate(zip(times, vectors)):
        if time_s is None or vector is None or not finite_number(time_s) or not all(
                finite_number(value) for value in vector):
            continue
        current = [float(value) for value in vector]
        if previous is not None:
            _, old_time, old_vector = previous
            if max(abs(current[axis] - old_vector[axis])
                   for axis in range(len(current))) <= change_tolerance:
                continue
            dt = float(time_s) - old_time
            if min_dt <= dt <= max_dt:
                output[index] = [(current[axis] - old_vector[axis]) / dt
                                 for axis in range(len(current))]
        previous = (index, float(time_s), current)
    return output


def unwrap_angles(values):
    """Unwrap finite scalar angles while preserving missing-sample alignment."""
    output = [None] * len(values)
    previous_raw = previous_unwrapped = None
    for index, value in enumerate(values):
        if not finite_number(value):
            continue
        raw = float(value)
        if previous_raw is None:
            unwrapped = raw
        else:
            delta = math.remainder(raw - previous_raw, 2.0 * math.pi)
            unwrapped = previous_unwrapped + delta
        output[index] = unwrapped
        previous_raw, previous_unwrapped = raw, unwrapped
    return output


def vector_norm(vector, horizontal=False):
    if vector is None:
        return None
    values = vector[:2] if horizontal else vector
    return math.sqrt(sum(value * value for value in values))


def stats(values):
    values = [value for value in values if value is not None and finite_number(value)]
    return {"max": max(values) if values else None,
            "rms": math.sqrt(sum(value * value for value in values) / len(values)) if values else None}


def concatenate_segments(paths, layer):
    points, locations = [], []
    for segment in sorted(paths.get(layer + "_segments", []), key=lambda item: item["sequence"]):
        segment_points = finite_points(segment.get("points", []))
        if points and segment_points and math.dist(points[-1], segment_points[0]) <= 1e-9:
            segment_points = segment_points[1:]
        points.extend(segment_points)
        locations.extend({"goal_index": segment.get("goal_index"),
                          "path_segment_index": point_index,
                          "source_path_sequence": segment.get("sequence")}
                         for point_index in range(len(segment_points)))
    return points, locations


def nearest_path_location(point, polyline, locations):
    _, _, _, segment_index = nearest_point_on_polyline(point, polyline)
    if segment_index is None or not locations:
        return {"goal_index": None, "path_segment_index": None}
    return locations[min(segment_index, len(locations) - 1)]


def clearance_losses(summaries):
    def loss(first, second):
        left = summaries[first].get("minimum_safety_clearance_m")
        right = summaries[second].get("minimum_safety_clearance_m")
        return left - right if left is not None and right is not None else None
    return {
        "simplified_clearance_loss_m": loss("planned", "simplified"),
        "reference_clearance_loss_m": loss("simplified", "reference"),
        "actual_clearance_loss_m": loss("reference", "actual"),
    }


def load_environment(path):
    data = yaml.safe_load(path.read_text())
    parameters = next(iter(data.values()))["ros__parameters"]
    values = parameters.get("obstacles", [])
    boxes = []
    for index in range(0, len(values), 6):
        x, y, z, sx, sy, sz = map(float, values[index:index + 6])
        boxes.append(((x - sx / 2, y - sy / 2, z - sz / 2),
                      (x + sx / 2, y + sy / 2, z + sz / 2)))
    return boxes, float(parameters["safety_radius"])


def navigation_rows(samples, paths):
    starts = [item.get("mission_time_s") for layer in ("planned", "simplified", "reference")
              for item in paths.get(layer + "_segments", [])
              if finite_number(item.get("mission_time_s"))]
    start = min(map(float, starts)) if starts else 0.0
    selected = []
    for row in samples:
        time_s = finite_csv_value(row, "mission_time_s")
        if time_s is not None and time_s + 1e-9 >= start:
            selected.append(row)
    return selected, start


def local_profile_min(profile, center_arc, window):
    selected = [row for row in profile if abs(row["arc_length_m"] - center_arc) <= window]
    if not selected:
        return None
    row = min(selected, key=lambda item: item["safety_clearance_m"])
    return row["safety_clearance_m"]


def max_or_none(values):
    values = [value for value in values if value is not None and finite_number(value)]
    return max(values) if values else None


def min_or_none(values):
    values = [value for value in values if value is not None and finite_number(value)]
    return min(values) if values else None


def parse_trajectory_diagnostics(path):
    if not path.exists():
        return {}
    result = {}
    for match in TRAJECTORY_RE.finditer(path.read_text(errors="replace")):
        result[int(match.group("goal"))] = {
            "selected_duration_scale": float(match.group("scale")),
            "selected_velocity_scale": float(match.group("velocity")),
            "reference_total_duration_s": float(match.group("duration"))}
    return result


def write_csv(path, rows, fieldnames=None):
    if fieldnames is None:
        fieldnames = list(rows[0]) if rows else []
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader(); writer.writerows(rows)


def plot_results(output, layer_points, profiles, boxes, safety_radius, corners,
                 samples, dynamics, summaries, window):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        return []
    output.mkdir(parents=True, exist_ok=True)
    for name in ("layered_paths_xy.png", "layered_paths_3d.png",
                 "clearance_by_arc_length.png", "corner_dynamics.png"):
        (output / name).unlink(missing_ok=True)
    for path in output.glob("corner_*_zoom.png"):
        path.unlink()
    colors = dict(planned="tab:blue", simplified="tab:orange",
                  reference="tab:green", actual="tab:red")
    fig, axis = plt.subplots(figsize=(13, 7))
    for lower, upper in boxes:
        axis.add_patch(Rectangle((lower[0], lower[1]), upper[0] - lower[0],
                                 upper[1] - lower[1], color="0.5", alpha=.35))
        axis.add_patch(Rectangle((lower[0] - safety_radius, lower[1] - safety_radius),
                                 upper[0] - lower[0] + 2 * safety_radius,
                                 upper[1] - lower[1] + 2 * safety_radius,
                                 fill=False, edgecolor="0.25", linestyle="--"))
    for layer in LAYERS:
        points = layer_points[layer]
        if points:
            axis.plot([p[0] for p in points], [p[1] for p in points],
                      color=colors[layer], label=layer, linewidth=1.5)
        position = summaries[layer].get("minimum_clearance_position")
        if position:
            axis.scatter(position[0], position[1], color=colors[layer], marker="x", s=60)
    for corner in corners:
        axis.annotate(str(corner["corner_index"]), corner["position"][:2])
    available = [points for points in layer_points.values() if points]
    if available:
        axis.scatter(available[0][0][0], available[0][0][1], marker="o", color="black")
        axis.scatter(available[0][-1][0], available[0][-1][1], marker="*", color="black")
    axis.set_aspect("equal", adjustable="box"); axis.set_xlabel("x (m)"); axis.set_ylabel("y (m)")
    axis.legend(); fig.tight_layout(); fig.savefig(output / "layered_paths_xy.png", dpi=160); plt.close(fig)

    fig = plt.figure(figsize=(12, 7)); axis = fig.add_subplot(111, projection="3d")
    for layer in LAYERS:
        points = layer_points[layer]
        if points:
            axis.plot([p[0] for p in points], [p[1] for p in points], [p[2] for p in points],
                      color=colors[layer], label=layer)
    axis.set_xlabel("x (m)"); axis.set_ylabel("y (m)"); axis.set_zlabel("z (m)")
    axis.legend(); fig.tight_layout(); fig.savefig(output / "layered_paths_3d.png", dpi=160); plt.close(fig)

    fig, axis = plt.subplots(figsize=(12, 6))
    for layer in LAYERS:
        axis.plot([row["arc_length_m"] for row in profiles[layer]],
                  [row["safety_clearance_m"] for row in profiles[layer]],
                  color=colors[layer], label=layer)
    axis.axhline(0.0, color="black", linewidth=.8); axis.set_xlabel("arc length (m)")
    axis.set_ylabel("safety clearance (m)"); axis.legend(); fig.tight_layout()
    fig.savefig(output / "clearance_by_arc_length.png", dpi=160); plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(13, 12), sharex=True)
    times = dynamics["times"]
    axes[0].plot(times, dynamics["reference_speed"], label="reference speed")
    axes[0].plot(times, dynamics["actual_speed"], label="actual speed")
    axes[1].plot(times, dynamics["reference_horizontal_acceleration"], label="reference horizontal accel")
    axes[1].plot(times, dynamics["actual_horizontal_acceleration"], label="actual horizontal accel")
    axes[2].plot(times, dynamics["reference_jerk"], label="reference jerk")
    axes[2].plot(times, dynamics["actual_jerk"], label="actual jerk")
    axes[3].plot(times, dynamics["temporal_tracking"], label="temporal tracking")
    axes[3].plot(times, dynamics["spatial_cross_track"], label="spatial cross-track")
    for axis in axes:
        axis.legend(loc="upper right"); axis.grid(alpha=.2)
    axes[-1].set_xlabel("mission time (s)"); fig.tight_layout()
    fig.savefig(output / "corner_dynamics.png", dpi=160); plt.close(fig)

    ranked = sorted(corners, key=lambda item: (
        (item.get("actual_clearance_m") if item.get("actual_clearance_m") is not None
         else math.inf),
        -(item.get("spatial_cross_track_max_m") or 0.0)))[:3]
    for corner in ranked:
        fig, axis = plt.subplots(figsize=(7, 7))
        center = corner["position"]
        for layer in LAYERS:
            points = [point for point in layer_points[layer]
                      if math.dist(point, center) <= window * 2.0]
            if points:
                axis.plot([p[0] for p in points], [p[1] for p in points],
                          color=colors[layer], label=layer)
        axis.scatter(center[0], center[1], color="black", marker="x")
        axis.set_xlim(center[0] - window, center[0] + window)
        axis.set_ylim(center[1] - window, center[1] + window)
        axis.set_aspect("equal", adjustable="box"); axis.legend(); fig.tight_layout()
        fig.savefig(output / f"corner_{corner['corner_index']:02d}_zoom.png", dpi=160)
        plt.close(fig)
    return [str(path) for path in sorted(output.glob("*.png"))]


def analyze(run, environment, astar, trajectory, output=None, step=0.02,
            corner_angle=25.0, merge_distance=0.30, corner_window=0.60,
            clearance_threshold=0.10):
    run = Path(run); output = Path(output or run); output.mkdir(parents=True, exist_ok=True)
    metadata = json.loads((run / "metadata.json").read_text())
    paths = json.loads((run / "paths.json").read_text())
    samples_all = read_samples(run / "samples.csv")
    samples, navigation_start = navigation_rows(samples_all, paths)
    boxes, safety_radius = load_environment(Path(environment))
    layer_points, layer_locations = {}, {}
    for layer in LAYERS[:-1]:
        layer_points[layer], layer_locations[layer] = concatenate_segments(paths, layer)
    actual_rows = [[finite_csv_value(row, name) for name in
                    ("actual_x", "actual_y", "actual_z")] for row in samples]
    valid_actual = finite_indexed_points(actual_rows)
    valid_sample_indices = [item[0] for item in valid_actual]
    layer_points["actual"] = [item[1] for item in valid_actual]
    layer_locations["actual"] = [
        {"goal_index": (int(value) if value is not None else None),
         "path_segment_index": (int(segment) if segment is not None else None)}
        for row_index in valid_sample_indices
        for value, segment in [(finite_csv_value(samples[row_index], "navigation_goal_index"),
                                finite_csv_value(samples[row_index], "navigation_segment_index"))]]
    profiles = {layer: clearance_profile(layer_points[layer], boxes, safety_radius, step)
                for layer in LAYERS}
    summaries = {layer: profile_summary(layer, layer_points[layer], profiles[layer], clearance_threshold)
                 for layer in LAYERS}
    for layer in LAYERS:
        position = summaries[layer].get("minimum_clearance_position")
        location = (nearest_path_location(position, layer_points[layer], layer_locations[layer])
                    if position is not None else
                    {"goal_index": None, "path_segment_index": None})
        summaries[layer]["minimum_clearance_goal_index"] = location["goal_index"]
        summaries[layer]["minimum_clearance_segment_index"] = location["path_segment_index"]
    losses = clearance_losses(summaries)

    times = [finite_csv_value(row, "mission_time_s") for row in samples]
    actual_velocity = [[finite_csv_value(row, name) for name in ("velocity_x", "velocity_y", "velocity_z")]
                       for row in samples]
    reference_velocity = [[finite_csv_value(row, name) for name in
                           ("reference_velocity_x", "reference_velocity_y", "reference_velocity_z")]
                          for row in samples]
    reference_acceleration = [[finite_csv_value(row, name) for name in
                               ("reference_acceleration_x", "reference_acceleration_y", "reference_acceleration_z")]
                              for row in samples]
    actual_acceleration = derivative(times, actual_velocity)
    actual_jerk = derivative(times, actual_acceleration)
    reference_jerk = held_signal_derivative(times, reference_acceleration)
    reference_points_at_time = [[finite_csv_value(row, name) for name in
                                 ("reference_x", "reference_y", "reference_z")]
                                for row in samples]
    cross_track = [None] * len(samples); actual_arcs = [None] * len(samples)
    projections = project_points_on_polyline(
        layer_points["actual"], layer_points["reference"])
    for sample_index, projection in zip(valid_sample_indices, projections):
        cross_track[sample_index] = projection[0]
        actual_arcs[sample_index] = projection[2]
    temporal = [finite_csv_value(row, "tracking_error") for row in samples]
    actual_yaw_rate = [abs(finite_csv_value(row, "angular_speed_z"))
                       if finite_csv_value(row, "angular_speed_z") is not None else None for row in samples]
    reference_yaw = [[value] if value is not None else None for value in
                     unwrap_angles([finite_csv_value(row, "reference_yaw") for row in samples])]
    reference_yaw_rate_vectors = held_signal_derivative(times, reference_yaw)
    reference_yaw_rate = [abs(value[0]) if value else None for value in reference_yaw_rate_vectors]
    yaw_accel = derivative(times, reference_yaw_rate_vectors)
    actual_yaw_accel = derivative(times, [[finite_csv_value(row, "angular_speed_z")]
                                          for row in samples])
    diagnostics = parse_trajectory_diagnostics(run / "launch.log")
    corners = identify_segment_corners(
        paths, "simplified", corner_angle, merge_distance)
    simplified_arcs = cumulative_lengths(layer_points["simplified"])
    for corner in corners:
        vertex = corner["vertex_index"]
        goal = corner.get("goal_index")
        corner["goal_index"] = goal
        corner["segment_index"] = max(0, vertex - 1)
        layer_clearances = []
        for layer in LAYERS:
            _, _, center_arc, _ = nearest_point_on_polyline(corner["position"], layer_points[layer])
            value = local_profile_min(profiles[layer], center_arc, corner_window)
            corner[layer + "_clearance_m"] = value; layer_clearances.append(value)
        reference_corner_arc = nearest_point_on_polyline(
            corner["position"], layer_points["reference"])[2]
        indices = [index for index, arc in enumerate(actual_arcs)
                   if finite_number(arc) and
                   abs(arc - reference_corner_arc) <= corner_window]
        def selected(values): return [values[index] for index in indices if index < len(values)]
        corner.update({
            "reference_max_speed_m_s": max_or_none(selected([vector_norm(value) for value in reference_velocity])),
            "reference_min_speed_m_s": min_or_none(selected([vector_norm(value) for value in reference_velocity])),
            "actual_max_speed_m_s": max_or_none(selected([vector_norm(value) for value in actual_velocity])),
            "actual_min_speed_m_s": min_or_none(selected([vector_norm(value) for value in actual_velocity])),
            "reference_max_acceleration_m_s2": max_or_none(selected([vector_norm(value) for value in reference_acceleration])),
            "actual_max_acceleration_m_s2": max_or_none(selected([vector_norm(value) for value in actual_acceleration])),
            "reference_max_jerk_m_s3": max_or_none(selected([vector_norm(value) for value in reference_jerk])),
            "actual_max_jerk_m_s3": max_or_none(selected([vector_norm(value) for value in actual_jerk])),
            "temporal_tracking_max_m": stats(selected(temporal))["max"],
            "temporal_tracking_rms_m": stats(selected(temporal))["rms"],
            "spatial_cross_track_max_m": stats(selected(cross_track))["max"],
            "spatial_cross_track_rms_m": stats(selected(cross_track))["rms"],
            "maximum_absolute_roll_rad": max_or_none(selected([abs(finite_csv_value(row, "roll")) if finite_csv_value(row, "roll") is not None else None for row in samples])),
            "maximum_absolute_pitch_rad": max_or_none(selected([abs(finite_csv_value(row, "pitch")) if finite_csv_value(row, "pitch") is not None else None for row in samples])),
            "actual_maximum_yaw_rate_rad_s": max_or_none(selected(actual_yaw_rate)),
            "reference_maximum_yaw_rate_rad_s": max_or_none(selected(reference_yaw_rate)),
            "reference_maximum_yaw_acceleration_rad_s2": max_or_none(selected([abs(value[0]) if value else None for value in yaw_accel])),
            "actual_maximum_yaw_acceleration_rad_s2": max_or_none(selected([abs(value[0]) if value else None for value in actual_yaw_accel])),
            "saturation_sample_count": sum(1 for index in indices if any(
                finite_csv_value(samples[index], field) == 1.0 for field in
                ("horizontal_saturated", "altitude_saturated", "attitude_saturated", "mixer_saturated"))),
            "minimum_four_layer_clearance_m": min(value for value in layer_clearances if value is not None),
            **diagnostics.get(goal, {})})

    clearance_rows = []
    for layer in LAYERS:
        clearance_rows.extend({"layer": layer, **row} for row in profiles[layer])
    write_csv(output / "path_clearance.csv", clearance_rows)
    write_csv(output / "corner_diagnostics.csv", corners)
    comparison_rows = [
        {"transition": "planned_to_simplified", "clearance_loss_m": losses["simplified_clearance_loss_m"]},
        {"transition": "simplified_to_reference", "clearance_loss_m": losses["reference_clearance_loss_m"]},
        {"transition": "reference_to_actual", "clearance_loss_m": losses["actual_clearance_loss_m"]}]
    for row in comparison_rows:
        loss = row["clearance_loss_m"]
        row["diagnostic_severity"] = ("unavailable" if loss is None else
                                      "none" if loss < .01 else
                                      "attention" if loss <= .03 else "significant")
    write_csv(output / "layer_comparison.csv", comparison_rows)
    dynamics = {
        "times": times,
        "reference_speed": [vector_norm(value) for value in reference_velocity],
        "actual_speed": [vector_norm(value) for value in actual_velocity],
        "reference_horizontal_acceleration": [vector_norm(value, True) for value in reference_acceleration],
        "actual_horizontal_acceleration": [vector_norm(value, True) for value in actual_acceleration],
        "reference_jerk": [vector_norm(value) for value in reference_jerk],
        "actual_jerk": [vector_norm(value) for value in actual_jerk],
        "temporal_tracking": temporal, "spatial_cross_track": cross_track}
    plots = plot_results(output / "plots", layer_points, profiles, boxes, safety_radius,
                         corners, samples, dynamics, summaries, corner_window)
    for config in (Path(environment), Path(astar), Path(trajectory)):
        shutil.copy2(config, output / config.name)
    summary = {"schema_version": 1, "run_directory": str(run.resolve()),
               "run_status": metadata.get("status"),
               "repository_commit": metadata.get("repository_commit"),
               "git_dirty": metadata.get("git_dirty"),
               "navigation_start_time_s": navigation_start,
               "spatial_sample_step_m": step, "corner_window_m": corner_window,
               "safety_radius_m": safety_radius, "layers": summaries,
               "clearance_losses": losses, "corner_count": len(corners),
               "corners": corners, "plots": plots,
               "global_dynamics": {
                   "temporal_tracking": stats(temporal),
                   "spatial_cross_track": stats(cross_track),
                   "maximum_absolute_roll_rad": max_or_none([abs(finite_csv_value(row, "roll")) if finite_csv_value(row, "roll") is not None else None for row in samples]),
                   "maximum_absolute_pitch_rad": max_or_none([abs(finite_csv_value(row, "pitch")) if finite_csv_value(row, "pitch") is not None else None for row in samples]),
                   "maximum_actual_yaw_rate_rad_s": max_or_none(actual_yaw_rate),
                   "saturation_sample_count": sum(1 for row in samples if any(
                       finite_csv_value(row, field) == 1.0 for field in
                       ("horizontal_saturated", "altitude_saturated", "attitude_saturated", "mixer_saturated")))}}
    (output / "geometry_summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    return summary


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--environment", type=Path, required=True)
    parser.add_argument("--astar", type=Path, required=True)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--spatial-sample-step", type=float, default=.02)
    parser.add_argument("--corner-angle", type=float, default=25.)
    parser.add_argument("--corner-merge-distance", type=float, default=.30)
    parser.add_argument("--corner-window", type=float, default=.60)
    parser.add_argument("--clearance-threshold", type=float, default=.10)
    return parser.parse_args(argv)


def main(argv=None):
    args = arguments(argv)
    analyze(args.run, args.environment, args.astar, args.trajectory, args.output,
            args.spatial_sample_step, args.corner_angle,
            args.corner_merge_distance, args.corner_window,
            args.clearance_threshold)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
