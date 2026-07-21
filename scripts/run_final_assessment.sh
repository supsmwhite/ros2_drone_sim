#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"
source "${script_dir}/final_assessment_lib.sh"

experiment=""
status=""
run_id=""
use_rviz="true"
output_root="${repo_root}/results"
assessment_timeout="180"
dry_run=false

usage() {
  echo "Usage: $0 --experiment NAME --status smoke|trial|final --run-id ID [--use-rviz true|false] [--output-root PATH] [--timeout SECONDS] [--dry-run]"
}

die() {
  echo "Error: $*" >&2
  exit 2
}

while (($#)); do
  case "$1" in
    --experiment|--status|--run-id|--use-rviz|--output-root|--timeout)
      (($# >= 2)) || die "missing value for $1"
      name="${1#--}"; name="${name//-/_}"; printf -v "$name" '%s' "$2"; shift 2 ;;
    --dry-run) dry_run=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

[[ -n "$experiment" ]] || die "--experiment is required"
[[ -n "$status" ]] || die "--status is required"
[[ -n "$run_id" ]] || die "--run-id is required"
[[ "$status" =~ ^(smoke|trial|final)$ ]] || die "invalid --status: $status"
[[ "$use_rviz" =~ ^(true|false)$ ]] || die "--use-rviz must be true or false"
[[ "$run_id" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || die "unsafe --run-id: $run_id"
[[ "$assessment_timeout" =~ ^[1-9][0-9]*$ ]] || die "--timeout must be a positive integer"

scenario_dir=""
recorder_experiment=""
service_name=""
launch_command=()
submission_description=""
expected_goals=()
case "$experiment" in
  hover)
    scenario_dir="01_hover"; recorder_experiment="hover"; service_name="/drone/mission/execute"
    launch_command=(ros2 launch drone_bringup assessment_basic_sim.launch.py "use_rviz:=${use_rviz}")
    expected_goals=(--expected-goal 0 0 1.5 0)
    submission_description="goal_cli single 0 0 1.5 yaw=0" ;;
  single_goal)
    scenario_dir="02_single_goal"; recorder_experiment="single_goal"; service_name="/drone/mission/execute"
    launch_command=(ros2 launch drone_bringup assessment_basic_sim.launch.py "use_rviz:=${use_rviz}")
    expected_goals=(--expected-goal 2 1 1.5 0)
    submission_description="pre-hover (0,0,1.5,0), then goal_cli single 2 1 1.5 yaw=0" ;;
  multi_goal)
    scenario_dir="03_multi_goal"; recorder_experiment="multi_goal"; service_name="/drone/mission/execute"
    launch_command=(ros2 launch drone_bringup assessment_basic_sim.launch.py "use_rviz:=${use_rviz}")
    expected_goals=(
      --expected-goal 3 0 1.5 0
      --expected-goal 3 3 1.5 1.5707963267948966
      --expected-goal 0 3 1.5 3.141592653589793
      --expected-goal 0 0 1.5 -1.5707963267948966)
    submission_description="pre-hover (0,0,1.5,0), then formal 3 m closed square (yaw 0,90,180,-90 deg)" ;;
  static_avoidance)
    scenario_dir="04_static_avoidance"; recorder_experiment="navigation"; service_name="/drone/interactive_goals/execute"
    launch_command=(ros2 launch drone_bringup assessment_navigation_sim.launch.py "use_rviz:=${use_rviz}" yaw_mode:=path_tangent)
    expected_goals=(--expected-goal 13.2 5.5 1.5 0)
    submission_description="navigation service goal (13.2,5.5,1.5), yaw_mode=path_tangent" ;;
  narrow_corridor)
    scenario_dir="05_narrow_corridor"; recorder_experiment="navigation"; service_name="/drone/interactive_goals/execute"
    launch_command=(ros2 launch drone_bringup assessment_navigation_sim.launch.py "use_rviz:=${use_rviz}" yaw_mode:=path_tangent)
    expected_goals=(--expected-goal 12.1 1.1 1.5 0)
    submission_description="navigation service goal (12.1,1.1,1.5), yaw_mode=path_tangent" ;;
  *) die "invalid --experiment: $experiment" ;;
esac

if [[ "$experiment" =~ ^(static_avoidance|narrow_corridor)$ ]]; then
  screenshot_checklist="- [ ] At least one RViz screenshot saved and referenced"
else
  screenshot_checklist="- [ ] Optional screenshots reviewed if supplied"
fi

output_root="$(realpath -m -- "$output_root")"
relative_path="${scenario_dir}/${status}/${run_id}"
run_dir="${output_root}/${relative_path}"
manifest_path="${output_root}/manifest.json"
[[ ! -e "$run_dir" ]] || die "refusing to overwrite existing run directory: $run_dir"

parameter_names=(dynamics.yaml controller.yaml environment.yaml astar.yaml planned_trajectory.yaml interactive_goal_editor.yaml interactive_goal_executor.yaml mission.yaml)
for parameter in "${parameter_names[@]}"; do
  [[ -f "${repo_root}/src/drone_bringup/config/${parameter}" ]] || die "missing parameter file: $parameter"
done
[[ -f /opt/ros/humble/setup.bash ]] || die "ROS 2 Humble setup not found"
[[ -f "${repo_root}/install/setup.bash" ]] || die "workspace is not built: install/setup.bash is missing"

commit_before="$(git -C "$repo_root" rev-parse HEAD)"
clean_before=true
[[ -z "$(git -C "$repo_root" status --porcelain)" ]] || clean_before=false
parameter_sources_clean=true
[[ -z "$(git -C "$repo_root" status --porcelain -- "src/drone_bringup/config")" ]] || parameter_sources_clean=false
if [[ "$status" == final ]]; then
  [[ "$clean_before" == true ]] || die "final run requires a clean worktree"
  [[ "$parameter_sources_clean" == true ]] || die "final run requires clean parameter sources"
  for parameter in "${parameter_names[@]}"; do
    git -C "$repo_root" ls-files --error-unmatch "src/drone_bringup/config/${parameter}" >/dev/null || die "final parameter is not tracked: $parameter"
  done
fi

domain_id=$((100 + $$ % 100))
printf -v launch_display '%q ' "${launch_command[@]}"
printf -v expected_display '%q ' "${expected_goals[@]}"
if [[ "$dry_run" == true ]]; then
  echo "scenario_id=$experiment"
  echo "scenario_dir=$scenario_dir"
  echo "recorder_experiment=$recorder_experiment"
  echo "status=$status"
  echo "run_id=$run_id"
  echo "output=$run_dir"
  echo "ros_domain_id=$domain_id"
  echo "launch=${launch_display% }"
  echo "service=$service_name"
  echo "expected_goals=${expected_display% }"
  echo "submission=$submission_description"
  if [[ "$experiment" =~ ^(single_goal|multi_goal)$ ]]; then
    echo "pre_hover=single 0 0 1.5 yaw=0 before recorder"
  fi
  echo "analyzer_parameters=${run_dir}/parameters"
  exit 0
fi

mkdir -p -- "$run_dir"
temporary_logs="$(mktemp -d /tmp/final_assessment_logs.XXXXXX)"
launch_pid=""
recorder_pid=""

stop_process_group() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT -- "-$pid" 2>/dev/null || true
    for _ in {1..30}; do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
    kill -TERM -- "-$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  stop_process_group "$recorder_pid"
  stop_process_group "$launch_pid"
  stop_ros_domain_daemon "$domain_id"
  preserve_assessment_logs "$temporary_logs" "$run_dir"
  rm -rf -- "$temporary_logs"
}
trap cleanup EXIT INT TERM

set +u
source /opt/ros/humble/setup.bash
source "${repo_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="$domain_id"
cd "$repo_root"

setsid "${launch_command[@]}" >"${temporary_logs}/launch.log" 2>&1 &
launch_pid=$!

wait_for_topic() {
  local topic="$1" limit="$2" deadline
  deadline=$((SECONDS + limit))
  while ((SECONDS < deadline)); do
    kill -0 "$launch_pid" 2>/dev/null || die "launch exited while waiting for $topic"
    if timeout 2 ros2 topic echo --once "$topic" >/dev/null 2>&1; then return 0; fi
  done
  die "timed out waiting for topic: $topic"
}

wait_for_service() {
  local service="$1" limit="$2" deadline
  deadline=$((SECONDS + limit))
  while ((SECONDS < deadline)); do
    kill -0 "$launch_pid" 2>/dev/null || die "launch exited while waiting for $service"
    [[ -n "$(ros2 service type "$service" 2>/dev/null)" ]] && return 0
    sleep 0.2
  done
  die "timed out waiting for service: $service"
}

wait_for_true() {
  local topic="$1" limit="$2" deadline output
  deadline=$((SECONDS + limit))
  while ((SECONDS < deadline)); do
    output="$(timeout 2 ros2 topic echo --once --qos-durability transient_local "$topic" std_msgs/msg/Bool 2>/dev/null || true)"
    grep -q "data: true" <<<"$output" && return 0
  done
  die "timed out waiting for true on: $topic"
}

submit_basic() {
  echo "+ ros2 run drone_mission goal_cli $*" >>"${temporary_logs}/submission.log"
  ros2 run drone_mission goal_cli "$@" >>"${temporary_logs}/submission.log" 2>&1
}

submit_navigation() {
  local x="$1" y="$2" z="$3"
  local request="{goals: {header: {frame_id: map}, poses: [{position: {x: ${x}, y: ${y}, z: ${z}}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}]}, draft_revision: 1}"
  echo "+ ros2 service call ${service_name} drone_msgs/srv/ExecuteGoalSequence ${request}" >>"${temporary_logs}/submission.log"
  ros2 service call "$service_name" drone_msgs/srv/ExecuteGoalSequence "$request" >>"${temporary_logs}/submission.log" 2>&1
  navigation_response_was_accepted "${temporary_logs}/submission.log" || die "navigation mission was not accepted"
}

wait_for_topic /drone/odom 20
wait_for_service "$service_name" 20

if [[ "$experiment" =~ ^(single_goal|multi_goal)$ ]]; then
  submit_basic single 0 0 1.5 yaw=0
  wait_for_true /drone/mission/complete "$assessment_timeout"
fi

recorder_command=(python3 tools/assessment_recorder.py
  --experiment "$recorder_experiment" --scenario-id "$experiment"
  --run-status "$status" --output "$run_dir" --service-name "$service_name"
  --timeout "$assessment_timeout" "${expected_goals[@]}")
setsid "${recorder_command[@]}" >"${temporary_logs}/recorder_stdout.log" 2>&1 &
recorder_pid=$!

wait_for_recorder() {
  local deadline=$((SECONDS + 15)) nodes info
  while ((SECONDS < deadline)); do
    kill -0 "$recorder_pid" 2>/dev/null || die "recorder exited before becoming ready"
    nodes="$(ros2 node list --no-daemon --spin-time 1.0 2>/dev/null || true)"
    info="$(ros2 node info /assessment_recorder --no-daemon --spin-time 1.0 2>/dev/null || true)"
    if grep -qx '/assessment_recorder' <<<"$nodes" && grep -q '/drone/odom' <<<"$info"
    then
      return 0
    fi
    sleep 0.1
  done
  die "timed out waiting for /assessment_recorder"
}
wait_for_recorder

case "$experiment" in
  hover) submit_basic single 0 0 1.5 yaw=0 ;;
  single_goal) submit_basic single 2 1 1.5 yaw=0 ;;
  multi_goal) submit_basic multi 3 0 1.5 yaw=0 3 3 1.5 yaw=90 0 3 1.5 yaw=180 0 0 1.5 yaw=-90 ;;
  static_avoidance) submit_navigation 13.2 5.5 1.5 ;;
  narrow_corridor) submit_navigation 12.1 1.1 1.5 ;;
esac

recorder_deadline=$((SECONDS + assessment_timeout + 20))
while kill -0 "$recorder_pid" 2>/dev/null; do
  ((SECONDS < recorder_deadline)) || die "recorder exceeded timeout guard"
  sleep 0.2
done
set +e
wait "$recorder_pid"
recorder_status=$?
set -e
recorder_pid=""
[[ "$recorder_status" -eq 0 ]] || die "recorder failed with status $recorder_status"

stop_process_group "$launch_pid"
launch_pid=""
preserve_assessment_logs "$temporary_logs" "$run_dir"

mkdir -- "${run_dir}/parameters"
for parameter in "${parameter_names[@]}"; do
  cp -- "${repo_root}/src/drone_bringup/config/${parameter}" "${run_dir}/parameters/${parameter}"
done
python3 tools/analyze_assessment_run.py "$run_dir" --parameters "${run_dir}/parameters" >"${temporary_logs}/analyzer.log" 2>&1
preserve_assessment_logs "$temporary_logs" "$run_dir"

(
  cd "${run_dir}/parameters"
  sha256sum -- "${parameter_names[@]}"
) >"${run_dir}/parameter_sha256.txt"
parameters_complete=true
for parameter in "${parameter_names[@]}"; do
  cmp -s "${repo_root}/src/drone_bringup/config/${parameter}" "${run_dir}/parameters/${parameter}" || parameters_complete=false
done

commit_after="$(git -C "$repo_root" rev-parse HEAD)"
clean_after=true
if [[ "$run_dir" == "$repo_root"/* ]]; then
  run_repo_relative="${run_dir#${repo_root}/}"
  manifest_repo_relative="${manifest_path#${repo_root}/}"
  post_status="$(git -C "$repo_root" status --porcelain -- . ":(exclude)${run_repo_relative}" ":(exclude)${manifest_repo_relative}")"
else
  post_status="$(git -C "$repo_root" status --porcelain)"
fi
[[ -z "$post_status" ]] || clean_after=false

cat >"${run_dir}/git_state.json" <<EOF
{
  "commit_before": "${commit_before}",
  "commit_after": "${commit_after}",
  "source_clean_before": ${clean_before},
  "source_clean_after": ${clean_after},
  "parameter_sources_clean_before": ${parameter_sources_clean}
}
EOF

cat >"${run_dir}/manual_acceptance.md" <<EOF
# Manual acceptance: ${experiment} / ${run_id}

Status: incomplete

- [ ] RViz trajectory and target markers checked
- [ ] No visually observed collision or attitude divergence
${screenshot_checklist}
- [ ] Curves and summary reviewed against the protocol
- [ ] Reviewer name and date recorded below

Reviewer:
Date:
Notes:
EOF

python3 tools/final_assessment_manifest.py create \
  --manifest "$manifest_path" --run-dir "$run_dir" --relative-path "$relative_path" \
  --scenario-id "$experiment" --recorder-experiment "$recorder_experiment" \
  --status "$status" --run-id "$run_id" \
  --commit-before "$commit_before" --commit-after "$commit_after" \
  --clean-before "$clean_before" --clean-after "$clean_after" \
  --parameters-complete "$parameters_complete" \
  --manual-acceptance-complete false >"${run_dir}/manifest.log"

(
  cd "$run_dir"
  find . -type f ! -name evidence_sha256.txt -print0 | sort -z | xargs -0 sha256sum
) >"${run_dir}/evidence_sha256.txt"
echo "Assessment completed: $run_dir"
