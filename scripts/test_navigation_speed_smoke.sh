#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"
cd "${repo_root}"

if (($# == 0)); then
  echo "Usage: $0 open|obstacle|turning|all [candidate and parameter options]" >&2
  exit 2
fi

exec python3 tools/navigation_speed_smoke.py "$@"
