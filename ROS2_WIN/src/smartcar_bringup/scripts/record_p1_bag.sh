#!/usr/bin/env bash
set -euo pipefail

use_sim_time=false
if [[ "${1:-}" == "--use-sim-time" ]]; then use_sim_time=true; shift; fi
if [[ $# -ne 1 ]]; then
  echo "usage: record_p1_bag.sh [--use-sim-time] /ws/bags/run_name" >&2
  exit 2
fi
output="$1"
[[ -n "$output" && "$output" != *$'\n'* ]] || {
  echo "output must be non-empty" >&2
  exit 2
}
mkdir -p "$(dirname "$output")"
args=(bag record -o "$output" /scan /odom /tf /tf_static /diagnostics /map)
if [[ "$use_sim_time" == true ]]; then args+=(--use-sim-time); fi
exec ros2 "${args[@]}"
