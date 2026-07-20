#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"
cd "${repo_root}"

set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install --packages-up-to drone_bringup
set +u
source install/setup.bash
set -u
colcon test-result --delete-yes >/dev/null 2>&1 || true

colcon test \
  --packages-select drone_bringup \
  --ctest-args \
    -R "assessment_basic_single|assessment_basic_multi|interactive_goal_navigation|assessment_disturbance|interactive_preflight_failure" \
    --output-on-failure \
  --event-handlers console_direct+

colcon test-result --verbose
