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

The five fixed scenarios are:

| Directory / `scenario_id` | Recorder type | Fixed protocol |
|---|---|---|
| `01_hover` / `hover` | `hover` | `(0,0,1.5)`, yaw `0 rad` |
| `02_single_goal` / `single_goal` | `single_goal` | pre-hover `(0,0,1.5,0)`, then formal `(2,1,1.5,0)` |
| `03_multi_goal` / `multi_goal` | `multi_goal` | square goals with yaw `0°, 90°, 180°, -90°` |
| `04_static_avoidance` / `static_avoidance` | `navigation` | `(13.2,5.5,1.5)`, `path_tangent` |
| `05_narrow_corridor` / `narrow_corridor` | `navigation` | `(12.1,1.1,1.5)`, `path_tangent` |

Static avoidance and narrow corridor intentionally share the ROS recorder type
`navigation`. Their distinct `scenario_id`, numbered directory, target snapshot, and manifest
entry must never be collapsed.

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

RViz screenshots referenced by manual acceptance are added without replacing recorder or
analyzer artifacts.

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
7. manual acceptance is complete.

The orchestration script always creates the manual template as incomplete, so a new run starts
with `report_eligible=false`. A reviewer must inspect RViz evidence, screenshots, curves, and
logs before finalization. Mark every checklist item complete, set `Status: complete`, fill in
reviewer/date, and reference each required screenshot inside the run directory. Then run:

```bash
python3 tools/final_assessment_manifest.py finalize \
  --manifest results/manifest.json \
  --run-dir results/04_static_avoidance/final/run_01 \
  --screenshot screenshots/rviz_overview.png
```

Finalization verifies protected raw evidence, `overall_pass`, parameter checksums, Git
conditions, manual fields, and screenshots; it then recalculates evidence checksums and updates
both manifests. Repeating finalization is rejected explicitly. Smoke and trial runs can never
become report eligible.

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
