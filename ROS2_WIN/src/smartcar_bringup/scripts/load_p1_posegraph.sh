#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 5 ]]; then
  echo "usage: load_p1_posegraph.sh PREFIX [continue|localize] [x,y,theta] [start_at_dock] [TIMEOUT_SEC]" >&2
  exit 2
fi
prefix="$1"
mode="${2:-continue}"
pose="${3:-0.0,0.0,0.0}"
start_at_dock="${4:-false}"
timeout_sec="${5:-20}"
if [[ "$prefix" == *.posegraph ]]; then prefix="${prefix%.posegraph}"; fi
if [[ "$prefix" == *.data ]]; then prefix="${prefix%.data}"; fi
if [[ -z "$prefix" || "$prefix" == *$'\n'* || "$prefix" == *"'"* ]]; then
  echo "PREFIX must be non-empty and contain no quote/newline" >&2
  exit 2
fi
[[ -s "${prefix}.posegraph" && -s "${prefix}.data" ]] || {
  echo "Both ${prefix}.posegraph and ${prefix}.data are required" >&2
  exit 2
}
case "$mode" in
  continue|localize) ;;
  *) echo "mode must be continue or localize" >&2; exit 2;;
esac
if [[ "$mode" == localize && "$start_at_dock" == true ]]; then
  echo "start_at_dock is valid only for continue mode" >&2
  exit 2
fi
IFS=',' read -r x y theta rest <<< "$pose"
[[ -n "${x:-}" && -n "${y:-}" && -n "${theta:-}" && -z "${rest:-}" ]] || {
  echo "POSE must be x,y,theta" >&2
  exit 2
}
number_re='[-+]?[0-9]+([.][0-9]*)?([eE][-+]?[0-9]+)?'
[[ "$x" =~ ^$number_re$ && "$y" =~ ^$number_re$ && "$theta" =~ ^$number_re$ ]] || {
  echo "POSE contains a non-numeric value" >&2
  exit 2
}
if ! [[ "$timeout_sec" =~ ^[1-9][0-9]*$ ]]; then
  echo "TIMEOUT_SEC must be positive" >&2
  exit 2
fi
if [[ "$mode" == localize ]]; then
  match_type=3
elif [[ "$start_at_dock" == true ]]; then
  match_type=1
else
  match_type=2
fi
available=false
for ((i=0; i<timeout_sec; i++)); do if ros2 service type /slam_toolbox/deserialize_map >/dev/null 2>&1; then available=true; break; fi; sleep 1; done
if [[ "$available" != true ]]; then
  echo "slam_toolbox/deserialize_map was not available within ${timeout_sec}s" >&2
  exit 1
fi
request="{filename: '$prefix', match_type: $match_type, initial_pose: {x: $x, y: $y, theta: $theta}}"
set +e
response="$(ros2 service call /slam_toolbox/deserialize_map \
  slam_toolbox/srv/DeserializePoseGraph "$request" 2>&1)"
status=$?
set -e
# Humble's DeserializePoseGraph.srv has an empty response section.  The CLI
# therefore reports success only through its exit code; the artifact checks
# above and the service type check are the remaining fail-closed gates.
if [[ $status -ne 0 ]]; then
  echo "deserialize_map failed:" >&2
  printf '%s\n' "$response" >&2
  exit 1
fi
echo "Loaded posegraph prefix $prefix in $mode mode"
