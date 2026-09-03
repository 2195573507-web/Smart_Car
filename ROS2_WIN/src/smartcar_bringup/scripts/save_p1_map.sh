#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: save_p1_map.sh /ws/maps/site_name" >&2
  exit 2
fi

output_prefix="$1"
if [[ -z "$output_prefix" ]]; then
  echo "output prefix must not be empty" >&2
  exit 2
fi
if [[ "$output_prefix" == *$'\n'* || "$output_prefix" == *"'"* ]]; then
  echo "output prefix must not contain a quote or newline" >&2
  exit 2
fi
if [[ "$output_prefix" == *.yaml ]]; then
  output_prefix="${output_prefix%.yaml}"
fi
mkdir -p "$(dirname "$output_prefix")"
ros2 run nav2_map_server map_saver_cli -t /map -f "$output_prefix" \
  --fmt pgm --mode trinary
[[ -s "${output_prefix}.yaml" && -s "${output_prefix}.pgm" ]] || {
  echo "map_saver_cli returned success but did not create .yaml and .pgm" >&2
  exit 1
}
echo "Saved map: ${output_prefix}.yaml and ${output_prefix}.pgm"
