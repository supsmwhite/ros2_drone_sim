#!/usr/bin/env bash

preserve_assessment_logs() {
  local temporary_logs="$1" run_dir="$2" name
  [[ -d "$temporary_logs" && -d "$run_dir" ]] || return 0
  for name in launch submission recorder_stdout analyzer; do
    if [[ -f "${temporary_logs}/${name}.log" ]]; then
      cp -- "${temporary_logs}/${name}.log" "${run_dir}/${name}.log"
    fi
  done
}
