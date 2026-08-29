# ROS2_WIN delivery report

This report covers the experimental S3 -> Windows ROS2 LAN/TCP PoC. It does
not modify STM32H757, ESPS3, IOS-APP, GPIO4 PWM, UART2, SCBP-CAN, BLE, vehicle
control, Nav2, or `/cmd_vel` paths.

## 1. Added and modified files

| File or directory | Reason |
| --- | --- |
| `.gitignore` | Keeps generated colcon/Docker artifacts out of source control. |
| `README.md` | Documents scope, modes, Docker commands, receiver/RViz/rqt start commands, and PoC boundary. |
| `config/bridge.yaml` | Provides the safe `unconfigured` default and all TCP, protocol, scan, intensity, and staleness parameters. |
| `docker/Dockerfile` | Builds the ROS 2 Humble runtime with colcon, tests, RViz2, rqt, and rosbag tools. |
| `docker/compose.yaml` | Defines the actual `ros2-dev` service, source mounts, named build volumes, and host TCP `8765:8765` publication. |
| `docker/entrypoint.sh` | Sources ROS 2 and an existing overlay before invoking the requested command. |
| `docker/enable-wsl2.ps1` | Documents the administrator-only Windows remediation required when Docker Desktop's Linux engine is unavailable. |
| `docker/open_rviz2.ps1` | Provides a Windows-side helper for launching RViz2 through the configured Compose service. |
| `src/s3_ydlidar_bridge/CMakeLists.txt`, `package.xml` | Declare the ament package, C++17 build, ROS dependencies, vendored SDK, executable, and six gtest targets. |
| `src/s3_ydlidar_bridge/config/bridge.yaml` | Installs the package-local form of the runtime configuration. |
| `include/s3_ydlidar_bridge/framing.hpp`, `src/framing.cpp` | Define and validate the experimental envelope, bounded TCP framing, and sequence classification. |
| `include/s3_ydlidar_bridge/transport.hpp`, `src/transport.cpp` | Implement unconfigured/replay transports and the single-client reconnecting TCP server. |
| `include/s3_ydlidar_bridge/official_decoder.hpp`, `src/official_decoder.cpp` | Adapt a caller-owned read-only payload buffer to the official SDK parser, without a serial port or control writes. |
| `include/s3_ydlidar_bridge/scan_mapper.hpp`, `src/scan_mapper.cpp` | Convert official parser nodes into parameterized metre/radian `LaserScan` data. |
| `include/s3_ydlidar_bridge/bridge_node.hpp`, `src/bridge_node.cpp`, `src/main.cpp` | Provide ROS parameters, `/scan`, `/diagnostics`, staleness, and sequence handling. |
| `launch/bridge.launch.py`, `rviz/s3_ydlidar_bridge.rviz`, `rqt/README.md` | Supply minimal launch, visualization, and diagnostics viewing configuration. |
| `test/test_framing.cpp`, `test/test_s3_protocol.cpp`, `test/test_tcp_server.cpp` | Test bounded half/sticky framing, envelope rejection, and loopback reconnect. |
| `test/test_official_decoder.cpp`, `test/test_scan_mapper.cpp`, `test/test_transport.cpp` | Test the official decoder, scan contract, replay, and unconfigured state. |
| `third_party/ydlidar_sdk/**`, `third_party/README.md` | Vendored, license-preserved minimal source subset of the official SDK and inventory of its provenance. |
| `test/README.md`, `testdata/README.md`, `testdata/s3_replay_sender.py` | Provide fixture guidance and an offline deterministic TCP sender for every required scenario. |
| `docs/design.md`, `docs/source-audit.md`, `docs/s3-gateway-protocol-TODO.md`, `docs/testing.md` | Record architecture, source audit, experimental protocol gap, and verification scope. |
| `findings.md`, `progress.md`, `task_plan.md` | Preserve the audit, execution history, and completion evidence. |

The pre-existing `S3_ROS2_STM_BUILD_AUDIT_PLAN.md` was not modified.

## 2. Official source reuse

The SDK extraction is from
`资料/EAI X3&X3 Pro激光雷达/ROS2教程/源码/YDLidar-SDK-master.tar.xz`,
version 1.0.6, SHA-256
`50F869C3EB3CFE39C6CAE1022EFFE79D4EED9C3E1112D96788C91C27A77F7F54`, under
the MIT license. Its `YDlidarDriver` parser and Linux channel backends are
compiled as `ydlidar_sdk`. The bridge calls the existing official
`waitPackage`, `parseResponseHeader`, `parseResponseScanData`, checksum, node
distance, and angle-correction code through the project-only
`parseMemoryChannel` entry point.
The concrete vendored locations are
`src/s3_ydlidar_bridge/third_party/ydlidar_sdk/src/ydlidar_driver.cpp/.h`,
`core/common`, the required `core/base` helpers, `core/network/ActiveSocket.*`
and `SimpleSocket.*`, and the Unix channel sources under
`core/serial/impl/unix`; unreferenced math/passive-socket sources are omitted.
The original MIT license is at `third_party/ydlidar_sdk/LICENSE.txt`.

The ROS2 driver reference is
`资料/EAI X3&X3 Pro激光雷达/ROS2教程/源码/ydlidar_ros2_driver-master.tar.xz`,
version 1.0.1, SHA-256
`EEFE97D97397DA7048FBE18202AB6D193DF1C4D4395455693B7795547A777DB1`, MIT.
It is audit-only, not the runtime entry point, because its node opens a serial
port and exposes radar controls.

## 3. Experimental protocol

`ESPS3/main/radar/radar_uplink_protocol.c` was requested but is absent from
this repository. The receiver therefore follows the supplied candidate only:
`S3RD` magic, version 1, type 1 (`RAW_YDLIDAR_FRAME`), little-endian flags,
device ID, stream ID, sequence, timestamp_ms, payload length, complete AA 55
YDLIDAR frame, and little-endian CRC16-Modbus. CRC covers version through
payload, excluding magic and final CRC. The fixed header is 26 bytes. Defaults
require version/type/device/stream = `1/1/1/1`, allow flags mask `0x0001`, and
require flags bit 0 to match payload CT bit 0. Zero-position packets delimit
complete scans: the first starts accumulation and the following one publishes
the preceding complete revolution. A missing boundary drops the partial
revolution; no synthetic partial scan is emitted. This is an experimental
compatibility assumption, not protocol freeze.

## 4. Custom adapter boundary

Custom code is limited to a caller-owned read-only memory channel, replay file
transport, S3RD extraction, bounded TCP chunk reassembly, single-client TCP
server/reconnect handling, diagnostics, sequence/staleness checks, and ROS
`LaserScan` publication. No COM/TTY open, radar command, motor/DTR action, or
STM32 protocol output exists in the bridge node.
The default scan contract is `frame_id=laser_frame`, angles in radians,
distances/ranges in metres, `range_min=0.10`, `range_max=8.0`, and
`rclcpp::SensorDataQoS()`; all are configurable in `config/bridge.yaml`.
Transport tests cover replay success/error paths and the explicit refusal of
unconfigured live mode.

## 5. Environment

Dockerfile base is `ros:humble-ros-base-jammy` (Ubuntu 22.04), with colcon,
ament CMake/gtest, pytest, and rosbag2 packages. Compose mounts editable source
and keeps `build`, `install`, and `log` in named Linux volumes. Run commands
from `ROS2_WIN/docker` in Docker Desktop WSL2 Linux-container mode. The package
enforces C++17 for its `std::optional` transport API.

## 6. Build and test evidence

`git diff --check` passed, project-authored files have no trailing whitespace,
the Compose file renders with `docker compose config`, package XML parses, the
launch file has the expected ROS 2 entry point, all CMake-listed vendor sources
exist, and no `build/`, `install/`, or `log/` paths are tracked. The required
commands were attempted:

```text
docker compose build
docker compose run --rm ros2-dev colcon build --symlink-install --packages-select s3_ydlidar_bridge
docker compose run --rm ros2-dev colcon test --packages-select s3_ydlidar_bridge
docker compose run --rm ros2-dev colcon test-result --verbose
```

After the WSL2/hypervisor remediation and reboot, Docker Desktop's Linux
engine recovered. `docker version` reports Client/Server 29.7.2 with context
`desktop-linux`; `docker info` reports `ostype=linux`.

The required commands completed successfully:

```text
docker compose build                                  PASS
docker compose run --rm ros2-dev colcon build ...     PASS
docker compose run --rm ros2-dev colcon test ...      PASS
docker compose run --rm ros2-dev colcon test-result... PASS
Summary: 25 tests, 0 errors, 0 failures, 0 skipped
```

An additional temporary container build/test with AddressSanitizer and
UndefinedBehaviorSanitizer flags also completed successfully; its build and
test trees were under `/tmp` and were discarded with the container.

The build emits warnings from the unchanged portions of the vendored official
SDK; they do not fail the build. A container-side Python compile check of the
launch file and offline sender also passed without writing bytecode into the
read-only fixture mount. No real-S3 runtime or hardware capture was attempted.

## 7. Offline replay result

`testdata/s3_replay_sender.py` exercises legal no-intensity and intensity
frames, half/sticky/multi-frame reads, bad CRC/version/length, duplicate/jump
sequences, and TCP disconnect/reconnect. It is deterministic offline input;
no physical S3 was involved.

The sender was run against a live bridge process inside the `ros2-dev`
container (with the Compose host-port mapping enabled). A no-intensity frame
published a `LaserScan` with `frame_id=laser_frame`; a separate intensity-mode
run with `ydlidar_intensities:=true` and `publish_intensities:=true` published
the intensity array, including the fixture quality values `24` and `36`. The
error run reported exactly one each of
`crc_errors`, `version_errors`, and `length_errors`, and no published scans.
The duplicate/jump run reported one duplicate, one jump, and two scans. The
reconnect run reported two accepted connections, two disconnects, and two
published scans. Unit tests cover the half/sticky framing paths as well.

## 8. S3 gateway protocol status

No frozen S3 Wi-Fi envelope or capture was found. The S3 radar test document
only covers UART1/GPIO44 raw capture, and the architecture/data-flow documents
mark the Wi-Fi radar bridge as future work. Magic/version/type, length
encoding, device identity, sequence semantics, outer CRC/authentication,
timestamp policy, and packet batching remain unknown. The existing STM-S3
`AA 55` control/status envelope is a separate protocol and is not reused.
Live mode consequently uses `UnconfiguredTransport` and publishes nothing.

## 9. How to start

Start the receiver with `transport:=tcp` and the Compose `8765:8765` mapping;
start RViz2 with the installed config and rqt with Diagnostics as documented in
`README.md`. Configure the S3 TCP client to the **Windows host's LAN IPv4
address** (not a Docker-internal address) and the configured receiver port
(default `8765`). The S3 and Windows host must be on the same routable Wi-Fi or
Ethernet LAN, with Wi-Fi client isolation disabled and Windows Firewall allowing
inbound TCP on that port. This path is S3-to-container TCP only; it does not
depend on DDS discovery across hosts.

## 10. What cannot be claimed yet

This delivery does not claim a live S3 Wi-Fi connection, real-device `/scan`,
end-to-end timing, SLAM/TF correctness, or vehicle operation. The inspected
SDK/ROS2 driver archives and radar-material directory contain no binary
YDLIDAR sample frame, so the offline tests use a synthetic test envelope and a
deterministic SDK-format packet, not a real S3 capture or an official binary
sample.

## 11. Minimum conditions for live S3 data

Still unverified: real S3 Wi-Fi, real capture, same-network hardware link,
cross-network/NAT, and all vehicle-control behavior. This task explicitly does
not modify STM32, GPIO4 PWM, UART2/SCBP-CAN, BLE, Nav2, or `/cmd_vel`, and does
not represent a formal protocol freeze.
