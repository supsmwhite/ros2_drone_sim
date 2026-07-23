# Navigation speed envelope validation

## Scope and evidence policy

This optimization searches for a useful navigation envelope, not a fixed `2.0 m/s`
target or a claimed physical maximum. Existing finalized runs, their parameter
snapshots, `results/manifest.json`, and Reviewer data are immutable. Every run described
here is temporary under `/tmp/ros2_drone_assessment_smoke/navigation_speed/` and is not
report-eligible evidence.

No candidate changes mass, inertia, thrust/drag coefficients, motor time constant,
`max_rpm`, the workspace, obstacles, A* inflation, collision validation, goal order, or
controller gains. The horizontal controller remains
`P + D + limited I + desired-acceleration feedforward`; altitude and attitude remain PD.

## Frozen acceptance policy

The policy was frozen before candidate testing and was not relaxed:

| Item | Frozen rule | Classification |
|---|---|---|
| Mission, order, state machine | complete every ordered goal through the public interface | correctness hard constraint |
| Collision / finite values | zero collision and zero NaN/Inf; planned, reference, and actual segments stay outside inflated obstacles | safety hard constraint |
| Motor / stability | RPM ≤ `20000`; recommended use ≤ `85%`; no divergence | physical hard constraint / margin |
| Saturation | no sustained saturation, not saturated at end; navigation candidates require zero samples | stability hard constraint / internal quality |
| Navigation tracking | maximum `< 0.05 m`; RMS is a ranking metric | frozen internal quality |
| Clearance | `>= 0.085 m` and no unexplained cliff from baseline | frozen safety quality |
| Terminal state | position `< 0.05 m`, speed `< 0.03 m/s`; existing per-goal yaw/angular-speed gates unchanged | frozen quality / protocol |
| Time | must improve; no fixed target | performance ranking |

## Reproduction

The smoke runner uses the production nodes and control chain, excludes takeoff from
navigation tracking, records paths/telemetry/diagnostics, and writes JSON plus a Markdown
comparison:

```bash
bash scripts/test_navigation_speed_smoke.sh open --candidate baseline
bash scripts/test_navigation_speed_smoke.sh obstacle --candidate baseline
bash scripts/test_navigation_speed_smoke.sh turning --candidate baseline
bash scripts/test_navigation_speed_smoke.sh all --candidate local_check
```

The three scenarios are an 8 m obstacle-free horizontal flight, the formal static-
avoidance goal `(13.2,5.5,1.5)`, and a shorter three-goal path with horizontal turns,
height changes, braking/re-acceleration, and `path_tangent` yaw.

## Candidates

All candidates retain `min_segment_duration=2.0 s` and `max_tilt_angle=0.15 rad`.
Columns are `nominal_speed`, `max_reference_speed`, `max_reference_acceleration`, and
`max_horizontal_acceleration`, in SI units.

| Candidate | Parameters | open s | obstacle s | turning s | Decision |
|---|---|---:|---:|---:|---|
| baseline | `0.50 / 0.90 / 0.60 / 0.80` | 17.84 | 54.41 | 41.10 | reference |
| A | `0.60 / 1.08 / 0.69 / 0.90` | 15.02 | 55.54 | 35.92 | reject: obstacle slower; turning tracking `0.05456 m` |
| B | `0.70 / 1.26 / 0.78 / 1.02` | 13.05 | 61.46 | — | reject: obstacle `duration_scale=1.50`, `13.0%` slower |
| C | `0.77 / 1.386 / 0.86 / 1.12` | 11.94 | 59.31 | — | reject: obstacle `duration_scale=1.50`, `9.0%` slower |
| D | `0.77 / 1.386 / 0.92 / 1.15` | 11.95 | 49.67 | 31.85 | reject: turning tracking `0.07332 m` |
| E | `0.65 / 1.17 / 0.82 / 1.05` | 13.94 | 53.32 | 33.59 | reject: turning tracking `0.06705 m` |
| F | `0.55 / 0.99 / 0.65 / 0.84` | 16.30 | 53.68 | 39.02 | boundary: passes, but only `0.00038 m` below tracking limit |
| G | `0.55 / 0.95 / 0.65 / 0.84` | 17.04 | 53.64 | 39.12 | recommended |

The principal limiter is not thrust or RPM. On obstacle-rich simplified paths, quintic
trajectory speed/acceleration validation selects discrete duration scales. A/B/C raise
nominal speed but trigger `1.25` or `1.50` scaling, which erases the nominal gain. Higher
acceleration-matched D restores obstacle speed but exceeds the frozen tracking limit in
rapid turns. G caps short-segment reference speed enough to retain tracking margin.

## Recommended smoke margins

| Scenario | Time change | Max actual speed | Tracking max / RMS | Clearance | RPM ratio | Tilt ratio | Saturation |
|---|---:|---:|---:|---:|---:|---:|---:|
| open | `17.84 → 17.04 s` (`-4.45%`) | `0.937 m/s` | `0.03088 / 0.00708 m` | n/a | `54.16%` | `12.65%` | 0 |
| obstacle | `54.41 → 53.64 s` (`-1.41%`) | `0.770 m/s` | `0.02807 / 0.01214 m` | `0.15010 m` | `55.82%` | `46.23%` | 0 |
| turning | `41.10 → 39.12 s` (`-4.81%`) | `0.960 m/s` | `0.04293 / 0.01343 m` | `0.23248 m` | `57.77%` | `45.52%` | 0 |

Every recommended smoke has zero collision, non-finite values, and controller/mixer
saturation. Selected duration scales are `1.10` (open), `1.15` (obstacle), and
`[1.50,1.10,1.00]` (turning); all velocity scales remain `1.0`.

## Four-goal protocol trial and RViz review

The fixed P1→P2→P3→P4 trial at commit `bc12e7c` completed all four plans in order:

| Metric | Existing finalized baseline | Candidate G temporary Trial |
|---|---:|---:|
| Total mission time | `133.868 s` | `122.057 s` (`-8.82%`) |
| Actual path length | `51.125 m` | `51.180 m` |
| Navigation tracking max / RMS | `0.03263 / 0.01039 m` | `0.03202 / 0.01111 m` |
| Minimum safety clearance | `0.18152 m` | `0.18022 m` |
| Final position / speed | `0.00666 m / 0.00019 m/s` | `0.00748 m / 0.00024 m/s` |
| Maximum RPM | `13067.5` (`65.34%`) | `13067.5` (`65.34%`) |
| Maximum roll / pitch | `0.04977 / 0.04227 rad` | `0.06032 / 0.05003 rad` |
| Saturation / collision / non-finite | `0 / 0 / 0` | `0 / 0 / 0` |

A second Trial with RViz enabled completed in `122.078 s`; tracking max remained
`0.04172 m`, clearance `0.18026 m`, with zero collision and saturation. Manual visual
review covered obstacles/inflation, planned/simplified/reference/actual paths, all four
replans, altitude changes, path-tangent yaw, turns, terminal behavior, and attitude. The
temporary screenshot and checklist remain with that Trial, not in `results/`.

## Remaining limits and rollback

Complex corners may still force path refinement and duration scaling; short segments,
height changes, and terminal yaw prevent the vehicle from sustaining the configured
speed cap. The project still has no dynamic-obstacle avoidance, local replanning, MPC,
or complete attitude planning.

Rollback is one commit:

```bash
git revert bc12e7c
```

This restores the previous defaults without touching the smoke infrastructure or any
finalized evidence.
