# ROS2_WIN delivery report

This report covers the experimental S3 -> Windows ROS2 LAN/TCP PoC. It does
not modify STM32H757, ESPS3, IOS-APP, GPIO4 PWM, UART2, SCBP-CAN, BLE, vehicle
control, Nav2, or `/cmd_vel` paths.

The P1 mapping host implementation is included below. It is complete for the
reviewed/offline and container boundaries, but it is not a live-hardware
acceptance report: the S3 uplink contract, SCBP parser, wheel freshness source,
and measured sensor calibration are still external prerequisites.

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
| `src/smartcar_state_bridge/**` | Read-only structured telemetry gate, wheel kinematics/FIFO, exact upstream odometry subset, standalone diagnostics node, and focused tests. |
| `src/smartcar_description/**` | URDF/xacro, provisional sensor extrinsics, and static frame definitions. |
| `src/smartcar_bringup/**` | P1 mapping launch, slam_toolbox configuration, RViz, map/rosbag workflow helpers, and evidence scripts. |
| `bags/.gitkeep`, `maps/.gitkeep`, `evidence/.gitkeep`, `docs/p1-mapping-evidence/README.md` | Durable host mounts and the H0-H6 evidence matrix/template. |
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

The required commands completed successfully. The final verification used the
Compose-built image `docker-ros2-dev:latest` with digest
`sha256:aa3d6c3f4708f37d7fe79e25a888d74e723f91dcc57ddb197033a7a941b2f5`:

```text
docker compose build                                  PASS
docker compose run --rm ros2-dev colcon build ...     PASS
docker compose run --rm ros2-dev colcon test ...      PASS
docker compose run --rm ros2-dev colcon test-result... PASS
Summary: 75 tests, 0 errors, 0 failures, 0 skipped
```

The count was independently rerun with a fresh Compose project and fresh
named volumes (`ros2_final_clean2`); its image digest was
`sha256:b06e22755c504f76adaa58c1fb8d0aa5d2e6db028959c9ea36d667104c440cac`.
This removes stale XML from the test-count evidence. The default mapping smoke
started `robot_state_publisher`, `s3_ydlidar_bridge`, and `slam_toolbox`, with
`/scan`, `/map`, and `/tf_static` endpoints present and no `/odom` or
`/cmd_vel` publishers.

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

## 12. P1 state and odometry boundary

`smartcar_state_bridge` is a transport-independent library used by the single
gateway process. It accepts a `TelemetryEnvelope` only when an approved parser
has already populated the structured wheel sample. It never interprets raw
bytes, opens a socket, or duplicates `Common/SCBP_CAN` (which is absent from
this repository). Live samples require configured and present source and
destination identities, the reviewed wheel type `0x0210`, source freshness
fields, a matching connection epoch, and a contiguous outer sequence. Decoder,
sequence, source-time, stale-watchdog, and FIFO faults latch odometry invalid
until an explicit `beginSession()`.

The Humble image currently contains `diff_drive_controller` 2.53.x. Its full
plugin has an unresolved `librsl.so` dependency in this lean image, so the
workspace compiles the exact upstream `Odometry` implementation from commit
`eb4ca17d610eb4315f7241c0134de1bdfc5748ea` against the installed public header.
This is a source-subset workaround and an ABI probe, not a claim that the
complete controller or `controller_manager` is linked or started.

The safe defaults remain `transport=unconfigured`,
`allow_live_telemetry=false`, `enable_live_odom=false`, `publish_odom=false`,
and `publish_tf=false`. The P1 launch starts only the gateway,
`robot_state_publisher`, `slam_toolbox`, and optional RViz2; it has no
`ros2_control`, controller, or `/cmd_vel` path.

## 13. H0-H6 acceptance status

The detailed matrix and artifact naming rules are in
`docs/p1-mapping-evidence/README.md`.

| Gate | Status | Current evidence |
| --- | --- | --- |
| H0 host/offline | PASS | 101 tests, four-package build, bounded parser/odom checks |
| H1 offline ROS workflow | PARTIAL | Replay/synthetic components and container startup pass; no accepted integrated rosbag |
| H2 real radar | BLOCKED | No frozen S3 contract or real capture |
| H3 real wheel telemetry | BLOCKED | No approved SCBP parser/freshness capture or calibrated geometry |
| H4 TF/clock | PARTIAL | Static sensor TF and launch pass; dynamic odom/time alignment unverified |
| H5 SLAM/map | BLOCKED | No accepted sensor bag or saved map/posegraph |
| H6 manual driving | BLOCKED | Outside P1; vehicle control remains deliberately absent |

Accordingly, this report claims host code/build/test completion only. It does
not claim real `/scan`, `/odom`, SLAM map quality, calibrated TF, or vehicle
operation.

## 14. SRP v4 chassis-state odometry delivery (2026-08-31)

### 1. Modification locations

- `src/smartcar_state_bridge`: independent SRP decoder, authoritative-pose
  tracker, shared odom/TF message builder, CMake/package wiring, and focused
  unit fixtures/tests.
- `src/s3_ydlidar_bridge`: type-2 opaque dispatch, connection lifecycle reset,
  odom/TF publication integration, diagnostics, configuration, routing test,
  and default graph launch test.
- `src/smartcar_bringup`: retained the existing single-gateway topology and
  exposed all four live/publication gates with false defaults in mapping,
  localization, and continue-mapping entry points.
- `docs/srp-v4-chassis-state-wire-contract.md`, workspace/package READMEs, and
  task evidence files: frozen host contract, golden vector, safety boundary,
  and verification record. `smartcar_description` and its provisional
  `base_link -> laser_frame` transform were reused unchanged.

### 2. Modification reasons

S3RD `message_type=2` now has a reviewed local contract: its payload is one
complete SRP v4 chassis-state frame. This permits Windows ROS 2 to consume the
STM authoritative pose without depending on firmware-only `Common/SRP`,
changing the S3RD outer decoder, or involving the YDLIDAR payload decoder.

### 3. Modification content

The decoder requires exactly 36 bytes, validates all framing/header/payload
fields and CRC16-CCITT-FALSE, rejects non-finite floats and clear
`ODOMETRY_VALID`, and converts millimetres/degrees to metres/radians. The
tracker publishes no first-frame update; later valid poses produce body-frame
twist from consecutive authoritative poses using the previous yaw and shortest
yaw difference. Duplicate/rollback timestamps, unreasonable dt, stale data,
bad frames, sequence faults, disconnects, and epoch changes clear the baseline.

Accepted updates build `/odom` in `odom` with child `base_link`. Optional
dynamic TF is copied from that already-stamped odometry message, so pose and
ROS receive-time stamp are identical. Existing conservative non-zero
covariance values are reused. S3RD type 2 returns before both the legacy wheel
adapter and official YDLIDAR decoder; type 1 `/scan` behavior is unchanged.

### 4. Potential impact

With defaults unchanged, there is no new `/odom` or dynamic TF publisher and
no live telemetry consumption. When deliberately enabled after hardware
review, the first valid chassis frame after every start/recovery is baseline
only, and any rejected frame creates one additional baseline-only recovery
frame. Type 2 is now an explicitly recognized opaque S3RD discriminator and
appears in diagnostics. The gateway remains the sole live transport and
potential dynamic TF owner; a second odom/TF publisher must not be launched.

### 5. Tests and build results

From `ROS2_WIN/docker`, all required commands passed:

```text
docker compose build                                             PASS
docker compose run --rm ros2-dev colcon build --symlink-install  PASS
docker compose run --rm ros2-dev colcon test                     PASS
docker compose run --rm ros2-dev colcon test-result --verbose    PASS
Summary: 101 tests, 0 errors, 0 failures, 0 skipped
```

A second build used the new Compose project `srp_v4_clean` plus
`--cmake-force-configure`; its independent test result was also 98/0/0/0.
Coverage includes all requested SRP field failures, NaN/Inf, unit/quaternion
conversion, first-frame/body twist/yaw wrap, source timestamps, stale state,
epoch/disconnect reset, shared odom/TF stamp, safe-default ROS graph, and
type-2/YDLIDAR isolation. These are offline/component results, not evidence of
successful real-vehicle odometry, SLAM, or mapping.

### 6. Golden frame and CRC

```text
AA 55 18 00 00 2A 15 02 01 04 00 00 E8 03 00 00
00 00 7A 44 00 00 FA C3 00 00 33 43 00 00 48 41
7F C0 0D 0A
```

CRC input is the 30 bytes from `18 00` through `48 41`. The
CRC16-CCITT-FALSE result is `0xC07F`, stored little endian as `7F C0`. The
frame represents sequence `0x2A`, timestamp 1000 ms, x 1000 mm, y -500 mm,
yaw 179 degrees, and total distance 12.5 m.

### 7. Default-disabled parameters

The following remain false in code, YAML, and bringup launch defaults:

```text
allow_live_telemetry=false
enable_live_odom=false
publish_odom=false
publish_tf=false
```

No live odometry was started during this delivery.

### 8. Interfaces awaiting STM/S3 integration

- STM must emit the exact 24-byte chassis payload and SRP framing documented
  in `docs/srp-v4-chassis-state-wire-contract.md`, including monotonic
  `timestamp_ms`, finite SI-convertible values, and correct validity flags.
- S3 must place exactly one complete 36-byte SRP frame, unchanged, in each
  S3RD `message_type=2` payload and preserve contiguous outer sequencing over
  each TCP connection. Disconnect/reconnect must establish a new connection
  epoch at the Windows gateway.
- Joint acceptance still needs captured golden-vector parity, corrupt-frame
  rejection, disconnect/reconnect and stale recovery, ROS `/odom` plus TF
  observation with one owner, and an integrated real `/scan` + `/odom` bag.
  Only after those checks should the four false gates be deliberately enabled.

## 15. SRP v4 live integration continuation (2026-08-31)

### Offline compatibility: PASS

The original 36-byte flags-`0x04` golden frame still decodes with CRC
`0xC07F`. An otherwise identical flags-`0x0C` frame also decodes; its CRC is
`0xD844`, stored as `44 D8`. Focused tests now cover the complete low-nibble
mask, rejection of every high nibble, the independent ODOMETRY_VALID rule, and
non-fixed inner sequence/timestamp values. Four packages built and the full
result is `101 tests, 0 errors, 0 failures, 0 skipped`.

### Real-time chassis telemetry: BLOCKED

The physical S3 established TCP from `192.168.31.239` and real `/scan` stayed
healthy for 120.60 seconds: 567 messages, 4.704 Hz average, 0.326-second
maximum gap. Diagnostics increased by 13,596 raw frames and 564 published
scans, with no added outer CRC error, queue overflow, or sequence gap.
However, `opaque_frames`, `chassis_frames`, and `telemetry_accepted` all
remained zero. No S3RD type 2 payload reached the ROS chassis decoder.

### Odom and TF: BLOCKED

Before opt-in, `/odom` did not exist and `odom -> base_link` was absent. The
latest bridge was then run with command-line-only true values for all four
gates and remained the sole potential owner. During the acceptance window it
published zero odometry updates and zero matching TF because no chassis frame
arrived. Frame IDs, finite values, rates, timestamp monotonicity, and exact
odom/TF stamp matching therefore cannot yet be accepted.

### Stale and reconnect: BLOCKED

No chassis baseline or odom stream existed, so asking the user to power-cycle
S3 would not test stopped odom extrapolation or first/second-sample recovery.
That observation must be repeated only after type-2 counters and real odom are
already increasing.

### Bag and safe final state

`bags/srp_v4_live_20260831_1453` contains 288.25 seconds and 6.5 MiB: 1,357
`/scan` messages, 288 `/diagnostics`, zero `/odom`, and zero `/tf`.
`/tf_static` had no publisher/message and does not appear in metadata. The
temporary all-true bridge was stopped. Current container
`smartcar-scan-safe-0831` uses the latest binary with all four gates false,
has reconnected to S3, and publishes only `/scan` plus diagnostics. A
pre-existing slam_toolbox process was not started or used by this turn;
mapping remains ineligible because live odom is absent and the laser extrinsic
is still provisional.
