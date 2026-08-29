#!/usr/bin/env bash
set -eo pipefail

# ROS 2 Humble's setup scripts reference optional variables while they are
# being sourced; enable nounset only after the environment has been loaded.
source /opt/ros/humble/setup.bash
if [[ -f /ws/install/setup.bash &&
      -f /ws/install/s3_ydlidar_bridge/share/s3_ydlidar_bridge/local_setup.bash ]]; then
  source /ws/install/setup.bash
fi
set -u
exec "$@"
