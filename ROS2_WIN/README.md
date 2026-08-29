# Smart_Car Windows ROS 2 bridge

This workspace is the host-side half of the intended chain:

```text
YDLIDAR -> ESP32-S3 UART1/GPIO44 -> S3 Wi-Fi -> WSL2/Docker Ubuntu 22.04
        -> ROS 2 Humble -> s3_ydlidar_bridge -> /scan
```

The ESP32-S3 remains the sole owner of the radar UART. The ROS 2 node never
opens a serial device, sends radar start/stop/query/DTR commands, or publishes
vehicle control. The official YDLIDAR SDK is used for packet checksum,
distance, angle, and node conversion. The ROS 2 driver archive is retained as
source reference only because its runtime opens a serial port.

## Current runtime modes

`transport: unconfigured` is the safe default. It reports that the S3 gateway
wire contract is not frozen and publishes no data. `transport: replay` reads a
file containing one or more complete raw YDLIDAR packets and publishes one
`sensor_msgs/msg/LaserScan`. The replay file is intentionally a raw payload;
it is not an invented S3 network format.

`transport: tcp` opts into the experimental live path. It listens as a TCP
server, accepts one S3 client at a time, survives disconnect/reconnect, and
validates the candidate `S3RD` envelope before handing the complete YDLIDAR
payload to the official parser. TCP reads are arbitrary chunks; the bounded
assembler handles split, sticky, and multi-frame reads in FIFO order, without
dropping valid frames as "latest". The default remains `transport: unconfigured`.

## Docker and colcon

From `ROS2_WIN/docker` in a WSL2 Linux-container Docker context:

```bash
docker compose build
docker compose run --rm ros2-dev colcon build --symlink-install \
  --packages-select s3_ydlidar_bridge
docker compose run --rm ros2-dev colcon test --packages-select s3_ydlidar_bridge
docker compose run --rm ros2-dev colcon test-result --verbose
```

If Docker reports `HCS_E_HYPERV_NOT_INSTALLED`, open an **Administrator**
PowerShell and run `.\enable-wsl2.ps1` from this directory, then reboot Windows
and restart Docker Desktop. The script enables the two required Windows
optional features, sets WSL default version 2, and enables hypervisor launch.

The editable source is mounted from `ROS2_WIN/src`; build, install, and log
trees live in named Linux volumes and are not written to Git.

To replay an approved raw-payload file mounted under `/ws/test`, override the
safe default transport explicitly:

```bash
docker compose run --rm ros2-dev ros2 run s3_ydlidar_bridge s3_ydlidar_bridge_node \
  --ros-args -p transport:=replay -p replay_file:=/ws/test/capture.bin
```

`capture.bin` must contain complete original YDLIDAR packet bytes. No S3
envelope is inferred by replay mode.

## ROS parameters

See `config/bridge.yaml`. Important fields are `frame_id`, `angle_min/max`
(radians), `range_min/max` (metres), `samples`, `scan_frequency_hz`, and
`stale_after_ms`. TCP fields include `tcp_listen_address`, `tcp_listen_port`,
bounded buffer/frame limits, and expected S3 version/type, allowed flags mask,
and device/stream IDs. The inner payload minimum is 10 bytes, the YDLIDAR
packet header length. The experimental flags mask defaults to `0x0001`:
flags `0x0000` and `0x0001` are accepted only when bit 0 matches the YDLIDAR
payload CT bit. A zero-position packet is a revolution boundary: the first
starts accumulation, and the next publishes the preceding complete revolution
before starting the next one. A missing boundary for `zero_packet_timeout_ms`
(1000 ms by default) drops the incomplete revolution rather than publishing a
partial scan. The publisher uses `rclcpp::SensorDataQoS()` and diagnostics are
published on `/diagnostics`. Invalid rays are `+inf` by default (configurable
to `NaN`).
When both `ydlidar_intensities` and `publish_intensities` are enabled, the
published intensity value is the raw YDLIDAR `sync_quality` byte (0..255), not
a calibrated physical intensity.

Run the live PoC receiver from `ROS2_WIN/docker`:

```bash
docker compose run --rm --service-ports ros2-dev ros2 run s3_ydlidar_bridge s3_ydlidar_bridge_node \
  --ros-args -p transport:=tcp -p tcp_listen_port:=8765
```

Then run a deterministic sender in another container shell:

```bash
python3 /ws/testdata/s3_replay_sender.py --scenario sticky
```

For visualization, use the installed RViz config and rqt diagnostics:

```bash
docker compose run --rm --service-ports ros2-dev rviz2 -d /ws/install/s3_ydlidar_bridge/share/s3_ydlidar_bridge/rviz/s3_ydlidar_bridge.rviz
docker compose run --rm --service-ports ros2-dev rqt
```

## Evidence boundary

`S3RD` is an experimental candidate based on the task-provided fields because
`ESPS3/main/radar/radar_uplink_protocol.c` is absent from this repository. This
workspace provides an opt-in LAN TCP PoC; it does not claim a real S3 Wi-Fi
connection, real-device `/scan`, timing accuracy, SLAM, or vehicle operation.
