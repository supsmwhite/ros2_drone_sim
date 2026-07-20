# Final assessment results

This directory contains only approved report experiments for the current three public
entrypoints. Parameter sweeps, tuning runs, scratch plots, and development regressions must
not be committed here. Each experiment class keeps one final approved run; a run marked
`smoke` proves the workflow only and is not report evidence.

## Layout and run lifecycle

The numbered experiment classes are `01_hover`, `02_single_goal`, `03_multi_goal`,
`04_navigation`, `05_disturbance`, and `06_failure_case`; `regression` is reserved for final
workflow regression evidence. Only `parameters/` and an actually executed hover smoke run
are populated during the initial rebuild. Empty or fabricated metrics are never added.

Each run is self-contained:

1. `assessment_recorder.py` writes immutable raw `samples.csv`, `events.csv`, `paths.json`,
   `metadata.json`, and `recorder.log`.
2. `analyze_assessment_run.py` reads those files plus the parameter snapshots and writes
   `summary.json` and PNG figures. It never needs a running ROS graph.
3. `manifest.json` lists a run only after it exists.

Regenerate plots without rerunning simulation:

```bash
python3 tools/analyze_assessment_run.py results/01_hover/smoke \
  --parameters results/parameters
```

Do not choose final interactive-navigation goals in tooling. The project owner must confirm
the goals, route, thresholds, and report figures before that experiment is recorded.

## Fixed metric definitions

- `final_position_error_m`: 3-D Euclidean distance from the last position sample to the final
  target. `final_window_mean_error_m` is the mean over the final 1 s.
- `arrival_time_s`: time from target publication/task acceptance to the first interval for
  which position error is below the configured threshold and speed is below its threshold
  continuously for the configured hold time. Defaults are 0.10 m, 0.08 m/s, and 1.0 s.
- `maximum_overshoot_m`: for hover, `max(z-target_z)` clipped at zero. For a translation
  segment, the maximum positive distance beyond the target after projection on the unit
  start-to-target direction. `maximum_overshoot_percent` divides this by segment length;
  it is null for a zero-length hover segment.
- `steady_state_mean_error_m`, `steady_state_rms_error_m`, and
  `steady_state_max_error_m`: position-error statistics over the final 3 s (or the explicitly
  reported shorter available window).
- `minimum_raw_obstacle_distance_m`: minimum Euclidean point-to-AABB distance using the raw
  boxes in `environment.yaml`. `minimum_safety_clearance_m` equals that raw distance minus
  `safety_radius_m`. Avoidance passes only when raw distance is greater than the safety radius,
  equivalently safety clearance is positive.
- Path lengths are sums of adjacent 3-D point distances. `path_efficiency` is actual path
  length divided by reference path length. Missing planned/reference paths are JSON `null`,
  never zero.
- Attitude metrics use absolute roll/pitch and angular-speed magnitude. RPM metrics report
  extrema, non-finite counts, saturation sample count, longest continuous saturation, and
  whether saturation remains at the end. A single clipped sample does not itself fail a run.
- Disturbance runs additionally report peak horizontal deviation, disturbance steady-state
  error, recovery time, and reverse overshoot. Failure runs retain the rejection/safety event
  timeline.

Every summary records both assignment thresholds (hover error below 0.30 m) and stricter
project thresholds (final error 0.10 m, positive safety clearance, no non-finite values, no
sustained attitude divergence, and no RPM saturation at task end).

## History

The former development results remain recoverable from `main`, historical commits, and tag
`assessment-feature-complete-v1`. Do not rewrite those commits or tags; use `git show` or a
separate worktree when historical evidence is needed.
