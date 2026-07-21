# Formal assessment results

`results/` stores reproducible assessment evidence. Development plots, tuning sweeps, and
unreviewed ad-hoc runs do not belong here. New runs are created by
`scripts/run_final_assessment.sh`; the script refuses every existing run directory and never
promotes an old smoke or trial to final.

## Run classes

- `smoke`: short workflow validation. It may use shortened recorder observation time and is
  never report evidence.
- `trial`: full-protocol rehearsal used to inspect stability, presentation, and manual
  screenshots. It is never report evidence.
- `final`: immutable report candidate using the fixed targets below. It requires a clean
  tracked source/parameter state before launch. The analyzer passing is necessary but not
  sufficient for report eligibility.

The five fixed base/navigation scenarios are:

| Directory / `scenario_id` | Recorder type | Fixed protocol |
|---|---|---|
| `01_hover` / `hover` | `hover` | `(0,0,1.5)`, yaw `0 rad` |
| `02_single_goal` / `single_goal` | `single_goal` | pre-hover `(0,0,1.5,0)`, then formal `(2,1,1.5,0)` |
| `03_multi_goal` / `multi_goal` | `multi_goal` | pre-hover outside recording, then `(3,0,1.5,0°)` → `(3,3,1.5,90°)` → `(0,3,1.5,180°)` → `(0,0,1.5,-90°)` |
| `04_static_avoidance` / `static_avoidance` | `navigation` | `(13.2,5.5,1.5)`, `path_tangent` |
| `05_narrow_corridor` / `narrow_corridor` | `navigation` | `(12.1,1.1,1.5)`, `path_tangent` |

Static avoidance and narrow corridor intentionally share the ROS recorder type
`navigation`. Their distinct `scenario_id`, numbered directory, target snapshot, and manifest
entry must never be collapsed.

Two independent bonus disturbance scenarios are recorded separately:

| Directory / `scenario_id` | Recorder type | Fixed protocol |
|---|---|---|
| `06_disturbance/short_gust` / `disturbance_short_gust` | `disturbance` | `(0,0,1.5,0)`, `+X 0.30 N` for `2 s`, then recovery |
| `06_disturbance/persistent_release` / `disturbance_persistent_release` | `disturbance` | `(0,0,1.5,0)`, `+X 0.30 N` for `10 s`, then force-release recovery |

Short gust evaluates transient suppression and recovery. Persistent release evaluates
steady-state compensation and recovery after force removal. Neither uses a mission submission
Service; both reuse the disturbance launch and the unified `disturbance` Recorder.

## Directory and invocation

```text
results/<numbered_scenario>/<smoke|trial|final>/<run_id>/
```

Example dry run:

```bash
scripts/run_final_assessment.sh \
  --experiment static_avoidance --status trial --run-id run_01 \
  --use-rviz true --output-root results --timeout 180 --dry-run
```

Remove `--dry-run` only after checking the printed launch, Service, target, output path, Git
state, and ROS domain. `run_id` is never reused. Failed and partial directories are retained for
diagnosis or moved outside `results/`; they are not overwritten.

Each completed run contains:

- recorder evidence: `metadata.json`, `samples.csv`, `events.csv`, `paths.json`,
  `diagnostics.csv`, and `recorder.log`;
- `summary.json`, protocol checks, failure reasons, `overall_pass`, and PNG figures;
- `launch.log`, `submission.log`, `recorder_stdout.log`, and `analyzer.log`;
- copied YAML under `parameters/` plus `parameter_sha256.txt`;
- `git_state.json`, `evidence_sha256.txt`, `manifest_entry.json`, and `manifest.log`;
- `manual_acceptance.md`, initially marked `Status: incomplete`.

RViz screenshots are added without replacing recorder or analyzer artifacts. They are optional
for hover, single-goal, multi-goal, and both disturbance runs, but required for both navigation
scenarios.

## Manifest schema 4

`manifest.json` has a `runs` array. Every new entry records:

- identity: `scenario_id`, `recorder_experiment`, `status`, `run_id`, `path`, and
  `protocol_version`;
- evidence links: metadata, summary, per-run manifest, parameter directory, and checksums;
- Git provenance: commit before/after and source-clean state before/after;
- outcome: stop reason, analyzer `overall_pass`, and failure reasons;
- manual acceptance status;
- every eligibility condition, `report_eligible`, and unmet conditions.

The historical hover smoke predates this workflow and remains explicitly marked
`legacy_layout`; it is not final evidence.

## Final evidence eligibility

`report_eligible=true` is allowed only when all conditions are true:

1. status is `final`;
2. recorder stop reason is not a timeout;
3. analyzer reports `overall_pass=true`;
4. HEAD is unchanged across the run;
5. source worktree is clean before and after, excluding only artifacts generated for that run
   and its manifest update;
6. all required parameter snapshots and checksums are complete;
7. protected raw evidence still matches its recorded checksums;
8. manual acceptance is complete;
9. for `static_avoidance` and `narrow_corridor`, at least one valid RViz screenshot is present
   and referenced by manual acceptance.

The orchestration script always creates the manual template as incomplete, so a new run starts
with `report_eligible=false`. A reviewer must inspect the available RViz evidence, curves, and
logs before finalization, mark every checklist item complete, and fill in reviewer/date. Hover,
single-goal, and multi-goal runs may then be finalized without `--screenshot`:

```bash
python3 tools/final_assessment_manifest.py finalize \
  --manifest results/manifest.json \
  --run-dir results/01_hover/final/run_01
```

For `static_avoidance` and `narrow_corridor`, save and reference at least one screenshot inside
the run directory, then pass it during finalization:

```bash
python3 tools/final_assessment_manifest.py finalize \
  --manifest results/manifest.json \
  --run-dir results/04_static_avoidance/final/run_01 \
  --screenshot screenshots/rviz_overview.png
```

Navigation screenshots may be added after the formal data run. Until they are supplied,
finalization is rejected without altering the recorded metrics. Finalization automatically uses
`scenario_id` to apply the screenshot policy and always verifies protected raw evidence,
`overall_pass`, parameter checksums, Git conditions, and manual fields; it then recalculates
evidence checksums and updates both manifests. Repeating finalization is rejected explicitly.
Smoke and trial runs can never become report eligible.

## Signal and metric semantics

- `/drone/motor_rpm_cmd` is commanded motor RPM, not measured motor speed. New CSV columns
  are `commanded_motor_rpm_m1` through `commanded_motor_rpm_m4`.
- Odom linear twist is expressed in `base_link`. `body_velocity_*` preserves that value;
  `map_velocity_*` is the same vector rotated by the Odom pose quaternion. `speed` is the
  frame-invariant vector magnitude.
- Saturation counts, duration, and end state for new runs come from individual
  `/drone/controller/diagnostics` callbacks in `diagnostics.csv`, not repeated Odom rows.
- `minimum_raw_obstacle_distance_m` is Euclidean point-to-raw-AABB distance.
  `minimum_safety_clearance_m = raw distance - safety_radius`; zero is the inflated-obstacle
  boundary. It is not distance to an already inflated box.
- `goal_position_error` is actual position to the active task goal. `tracking_error` is actual
  position to `/drone/trajectory_setpoint`; navigation acceptance uses the navigation-phase
  tracking metric.
- Yaw error is `wrap_to_pi(reference_yaw - actual_yaw)`. Basic runs use pose-goal yaw;
  navigation uses trajectory-setpoint yaw. Yaw is reported without inventing a separate
  assessment threshold; ROS completion-gate tolerances are identified only as configuration
  sources.

Legacy schema-3 recordings remain readable. Missing reference yaw is reported as
`unavailable`, and legacy saturation counts retain their original Odom-sample semantics.

For disturbance runs, `recovery_threshold_entry_time_s` is the first post-release entry into
the configured position and speed thresholds. `recovery_confirmed_time_s` is the later Recorder
confirmation after the hold condition remains satisfied, and is the value preferred in report
text. `recovery_confirmation_hold_time_s` is their difference. The legacy `recovery_time_s`
remains an alias of threshold entry for comparability.

The committed historical disturbance finals are not re-analyzed. Their report-only recovery
semantics are generated from checksum-verified immutable `summary.json` and `events.csv` files:

```bash
python3 tools/build_disturbance_report_metrics.py \
  --results-root results \
  --output results/06_disturbance/report_metrics.json
```
