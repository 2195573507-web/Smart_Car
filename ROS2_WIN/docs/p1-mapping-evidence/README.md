# P1 Mapping Evidence

This directory is the evidence boundary for the host-side P1 mapping work.
It records reproducible checks and their limits; an offline or loopback result
must never be reported as real S3, vehicle, or mapping acceptance.

## Current matrix

| Gate | Input and check | Current status | Evidence limit |
| --- | --- | --- | --- |
| H0 | C++ unit tests, parser/framing checks, bounded queues, wheel math and freshness gates | PASS | Proves host code only |
| H1 | Offline replay and synthetic wheel fixtures; container ROS startup | PARTIAL | No integrated `/scan` + `/odom` rosbag has been accepted |
| H2 | Real S3 radar over the LAN, one gateway owner, live `/scan` and diagnostics | BLOCKED | No frozen S3 uplink contract or real capture exists |
| H3 | Real STM32 wheel telemetry with source freshness and calibrated geometry | BLOCKED | No approved SCBP parser or live telemetry path is present |
| H4 | Dynamic `odom -> base_link`, static sensor TF, and clock/TF diagnostics | PARTIAL | Static TF is verified; live odom and time alignment are not |
| H5 | `/scan` + `/odom` + TF rosbag through `slam_toolbox`, save/load map and posegraph | BLOCKED | No accepted real sensor bag or map artifact exists |
| H6 | Low-speed manual driving with physical emergency-stop supervision | BLOCKED | P1 does not control the vehicle and has no `/cmd_vel` path |

## Reproduce host evidence

Run these commands from `ROS2_WIN/docker` in PowerShell:

```powershell
docker compose build
docker compose run --rm ros2-dev bash -lc "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-force-configure --executor sequential"
docker compose run --rm ros2-dev bash -lc "source /opt/ros/humble/setup.bash && colcon test --event-handlers console_direct+ --executor sequential && colcon test-result --verbose"
docker compose run --rm ros2-dev ros2 launch smartcar_bringup p1_mapping.launch.py --show-args
docker compose run --rm ros2-dev ros2 launch smartcar_bringup description.launch.py --show-args
docker compose run --rm ros2-dev xacro /ws/src/smartcar_description/urdf/smart_car.urdf.xacro
```

The expected current test result is `101 tests, 0 errors, 0 failures, 0
skipped`. The default launch configuration must report:

```text
transport=unconfigured
allow_live_telemetry=false
enable_live_odom=false
publish_odom=false
publish_tf=false
```

The bridge YAML now recognizes the reviewed SRP chassis discriminator with
`s3_opaque_message_types: [2]`. It remains an opaque outer payload and is
strictly isolated from YDLIDAR decoding. The four live/odom/TF parameters above
still prevent default consumption or publication.

For a runtime smoke test, use a private `ROS_DOMAIN_ID` and capture the node
list, `/diagnostics`, `/tf_static`, `/scan`, `/odom`, `/map`, and
`/cmd_vel` topic info. The expected default result is a single
`robot_state_publisher`, a single `s3_ydlidar_bridge`, `slam_toolbox`, static
`base_link -> laser_frame` and `base_link -> imu_link`, no `/odom` publisher,
and no `/cmd_vel` publisher. Stop the launch before deleting its container.

## Required artifacts for hardware gates

For H2-H6, keep the original input and the exact command beside the result:

```text
evidence/<gate>-<date>/
  git-commit.txt
  docker-image.txt
  apt-versions.txt
  command.txt
  diagnostics.yaml
  tf-tree.yaml
  topics.txt
  input-capture-or-bag.sha256
  conclusion.txt
```

H2 requires a reviewed S3RD/SCBP contract, a raw capture, gateway sequence and
age counters, and a real `/scan`. H3 additionally requires source
`sample_tick`, `sample_seq`, `valid`, source timestamp policy, measured track
width/wheel diameter/signs, and a disconnect test showing that odometry stops
and remains invalid until `beginSession()`. H4-H5 require the corresponding
three-clock/TF evidence and a saved YAML/PGM plus `slam_toolbox` posegraph.
H6 requires a separate vehicle-control review; this workspace must remain
read-only with respect to motor commands.

For an accepted map, `p1_localization.launch.py` takes a non-empty
`posegraph:=/ws/maps/...` and/or `map_yaml:=/ws/maps/...yaml`. The helper
scripts `save_p1_posegraph`, `load_p1_posegraph`, `record_p1_bag`, and
`play_p1_bag` call the official ROS services/tools and fail closed when the
service or input is missing.

## Explicit non-claims

`smartcar_state_bridge` accepts only structured results from an approved
decoder or an explicitly labelled offline fixture. `s3_ydlidar_bridge` keeps
opaque telemetry bytes observable but does not infer a wheel schema from raw
payloads. The repository has no `Common/SCBP_CAN` source and therefore does
not add a duplicate parser or claim live `/odom`/SLAM readiness.
