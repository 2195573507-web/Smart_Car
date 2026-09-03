#!/usr/bin/env bash
set -euo pipefail

without_clock=false
rate=1
loop=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --without-clock) without_clock=true; shift;;
    --rate)
      [[ $# -ge 2 ]] || { echo '--rate requires a value' >&2; exit 2; }
      rate="$2"; shift 2;;
    --loop) loop=true; shift;;
    -*) echo "unknown option: $1" >&2; exit 2;;
    *) break;;
  esac
done
if [[ $# -ne 1 ]]; then
  echo "usage: play_p1_bag.sh [--without-clock] [--rate N] [--loop] BAG_PATH" >&2
  exit 2
fi
bag_path="$1"
[[ -d "$bag_path" ]] || { echo "bag path is not a directory: $bag_path" >&2; exit 2; }
args=(bag play "$bag_path" --rate "$rate" --topics /scan /odom /tf /tf_static /diagnostics /map)
if [[ "$without_clock" == false ]]; then args+=(--clock); fi
if [[ "$loop" == true ]]; then args+=(--loop); fi
exec ros2 "${args[@]}"
