#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: save_p1_posegraph.sh PREFIX [TIMEOUT_SEC]" >&2
  exit 2
fi
prefix="$1"
timeout_sec="${2:-20}"
if [[ -z "$prefix" || "$prefix" == *$'\n'* || "$prefix" == *"'"* ]]; then
  echo "PREFIX must be non-empty and contain no quote/newline" >&2
  exit 2
fi
if [[ "$prefix" == *.posegraph ]]; then prefix="${prefix%.posegraph}"; fi
if [[ "$prefix" == *.data ]]; then prefix="${prefix%.data}"; fi
if ! [[ "$timeout_sec" =~ ^[1-9][0-9]*$ ]]; then
  echo "TIMEOUT_SEC must be a positive integer" >&2
  exit 2
fi
mkdir -p "$(dirname "$prefix")"
available=false
for ((i=0; i<timeout_sec; i++)); do
  if ros2 service type /slam_toolbox/serialize_map >/dev/null 2>&1; then
    available=true
    break
  fi
  sleep 1
done
if [[ "$available" != true ]]; then
  echo "slam_toolbox/serialize_map was not available within ${timeout_sec}s" >&2
  exit 1
fi
set +e
response="$(ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: '$prefix'}" 2>&1)"
status=$?
set -e
if [[ $status -ne 0 || ! "$response" =~ result(=|:)[[:space:]]*0([^0-9]|$) ]]; then
  echo "serialize_map failed:" >&2
  printf '%s\n' "$response" >&2
  exit 1
fi
[[ -s "${prefix}.posegraph" && -s "${prefix}.data" ]] || {
  echo "serialize_map returned success but files are missing" >&2
  exit 1
}
echo "Saved posegraph: ${prefix}.posegraph and ${prefix}.data"
