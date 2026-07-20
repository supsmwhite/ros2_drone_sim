#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"
cd "${repo_root}"

python3 -m pytest -q tools/tests

set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install --packages-up-to drone_bringup
set +u
source install/setup.bash
set -u
colcon test-result --delete-yes >/dev/null 2>&1 || true

colcon test \
  --packages-select drone_dynamics drone_controller drone_mission drone_planning \
  --event-handlers console_direct+

colcon test \
  --packages-select drone_bringup \
  --ctest-args \
    -R "physical_parameter_consistency|assessment_launch_structure|interactive_mission_service|interactive_preflight_failure|external_wrench|horizontal_integral_node|disturbance_demo_node" \
    --output-on-failure \
  --event-handlers console_direct+

colcon test-result --verbose
