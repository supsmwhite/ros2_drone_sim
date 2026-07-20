#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"
cd "${repo_root}"

set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install
set +u
source install/setup.bash
set -u
colcon test-result --delete-yes >/dev/null 2>&1 || true
colcon test --event-handlers console_direct+
colcon test-result --verbose
