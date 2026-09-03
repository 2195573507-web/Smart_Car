# ROS2_WIN Audit Findings

- `ROS2_WIN/` initially contained only `.gitkeep` and the pre-existing
  `S3_ROS2_STM_BUILD_AUDIT_PLAN.md`.
- Git baseline is commit `6de387a`; the existing audit plan is untracked and
  must be preserved.
- Official ROS2 SDK archive:
  `资料/EAI X3&X3 Pro激光雷达/ROS2教程/源码/YDLidar-SDK-master.tar.xz`.
  It contains `LICENSE.txt`, `core/common/ydlidar_protocol.h`, common/core
  parser sources, and serial/network backends.
- Official ROS2 driver archive:
  `资料/EAI X3&X3 Pro激光雷达/ROS2教程/源码/ydlidar_ros2_driver-master.tar.xz`.
  `package.xml` reports version `1.0.1` and MIT; its node directly opens a
  serial port and exposes start/stop services, so it is source reference only
  for this task.
- SDK README states C/C++11 support and identifies the communication protocol
  documentation under `doc/YDLidar-SDK-Communication-Protocol.md`.
- S3 gateway framing is not present as a frozen project contract in the
  material inspected so far. Live transport therefore must remain explicit,
  injectable, and disabled without configured framing details.
- X3/X3PRO manuals identify 115200 8N1 UART and the raw scan fields `PH` on
  wire as `AA 55`, followed by `CT`, `LSN`, `FSA`, `LSA`, `CS`, and `Si`; the
  SDK's triangle path is the adopted source of the exact q2 distance scale
  and second-level angle correction.
- Existing project audit states radar input is ESP32-S3 UART1/GPIO44 and ROS2
  must not access the radar UART or alter STM32 UART2/SCBP-CAN/PWM/BLE paths.
- The inspected YDLIDAR SDK and ROS2 driver archives and the radar-material
  directory contain no binary/raw scan capture suitable for an official sample
  replay. Offline fixtures are therefore deterministic SDK-format test bytes,
  explicitly not a real-device capture.
- Cross-project protocol audit: `ESPS3/docs/S3_YDLIDAR_X3PRO_TEST.md` is limited
  to UART1/GPIO44 raw capture, while `DOCS/architecture/data_flow.md` and
  `DOCS/ESP32/S3_ARCHITECTURE.md` mark the Wi-Fi radar bridge contract as
  future work. The existing STM-S3 `AA 55` SCBP frame is a separate control/
  status envelope and is not reused for radar data.

## Windows bridge startup diagnostic (2026-08-29)

- The Compose service's default `bash` command is not a daemon; `up -d
  ros2-dev` consequently exits with status 0 and produces no service log.
- The documented live command is the actual runtime entry point:
  `ros2 run s3_ydlidar_bridge s3_ydlidar_bridge_node --ros-args -p
  transport:=tcp -p tcp_listen_port:=8765`.
- The bridge listens on `0.0.0.0:8765` inside the container. Docker publishes
  both IPv4 wildcard and IPv6 wildcard host bindings for `8765`.
- Windows has wildcard TCP listeners for port 8765 and a local WLAN-path
  `Test-NetConnection 192.168.31.101:8765` succeeded.
- The container image lacks `ss` and `netstat`; `/proc/net/tcp` was used for
  the equivalent in-container LISTEN evidence. No bridge source or S3RD
  protocol field was changed.

## P1 mapping audit (2026-08-29)

- `P1_MAPPING_PLAN.md` requires a single gateway to dispatch raw YDLIDAR and
  SCBP telemetry, a read-only wheel odometry path, measured TF, and a
  slam_toolbox/map workflow. The current workspace has only
  `s3_ydlidar_bridge`; the other packages and `/odom` are absent.
- The repository has no frozen S3 Wi-Fi telemetry envelope or real capture. A
  `Common/SCBP_CAN` directory is also absent; SCBP reference implementations
  are in firmware trees. Host-side code must therefore expose an explicit
  telemetry decoder boundary and keep live wheel odometry disabled.
- `TcpChunkAssembler` currently ignores its ready-queue limit. This is a host
  correctness gap to fix before adding dispatch: bounded buffers and drop
  counters are required by P1.
- STM32 wheel freshness (`sample_tick`/`sample_seq`/`valid`) and four-wheel
  encoder support are not present in the inspected source. Synthetic odometry
  tests can prove math and gating only; they cannot prove live `/odom`, TF, or
  SLAM.
- Safe scope selected for this turn: edits remain under `ROS2_WIN/`; no S3,
  STM32, SCBP, UART2, BLE, PWM, controller_manager, or `/cmd_vel` changes.

## P1 hardening and final host verification (2026-08-30)

- Live `TelemetryDecoder` input now requires both expected source and
  destination IDs to be configured and both IDs to be present in every
  envelope. Offline fixtures may continue to omit those IDs.
- The decoder accepts only the reviewed wheel schema identifier `0x0210` even
  if a compatibility parameter is supplied; arbitrary message types cannot
  authorize odometry. Rejected decodes clear the caller's output sample.
- `WheelOdom` rejects invalid wheel diameter configuration before integration;
  a connection epoch explicitly bound by `beginSession()` remains mandatory on
  all subsequent samples. Recovery from a live decoder, sequence, time, stale,
  or FIFO fault still requires `beginSession()`.
- The Humble runtime package is `diff_drive_controller` 2.53.x in the current
  image. Because its shared plugin has an unresolved `librsl.so` dependency,
  the project compiles the exact upstream `Odometry` source subset at commit
  `eb4ca17d610eb4315f7241c0134de1bdfc5748ea` against the public header. This is
  a deliberate workaround, not evidence of linking the complete controller.
- Current verification is four-package Docker build plus `101 tests, 0 errors,
  0 failures, 0 skipped`. Launch/xacro/static-TF checks pass with safe defaults;
  no `/odom`, `/cmd_vel`, controller manager, or ros2_control process is
  started by P1.
- H0 host/offline evidence is complete. H1 and H4 have component/static
  evidence only. H2/H3 require a frozen S3/SCBP contract and real captures;
  H5 requires an accepted sensor bag and saved map/posegraph; H6 is outside P1
  and requires a separate vehicle-control review.

## SRP v4 chassis-state audit (2026-08-31)

- The existing S3RD extractor already classifies explicitly allowed outer
  message types as opaque and returns before the official YDLIDAR decoder.
- `TcpServerTransport` already attaches a connection epoch and emits ordered
  open/close callbacks. These callbacks are the correct reset boundary for
  chassis pose history.
- The existing state path integrates a four-wheel velocity sample. SRP v4
  instead supplies authoritative x/y/yaw pose, so it needs a separate decoder
  and pose-delta tracker rather than adaptation into `WheelStatusSample`.
- `s3_ydlidar_bridge` is already the only bringup node that can own live TCP,
  `/odom`, and dynamic odom TF. The standalone state node is not launched and
  all publication gates currently default false.
- Existing odom publishers use conservative non-zero covariance values and
  copy one odometry header into TF. The SRP path will share one message builder
  so the ROS receive timestamp and covariance behavior cannot diverge.
- The implemented decoder validates exactly 36 bytes and independently checks
  SRP magic, 24-byte payload length, priority/type/header flags, CCITT-FALSE
  CRC, EOF, schema, payload flags, reserved bytes, ODOMETRY_VALID, and all four
  floating-point fields. It has no firmware or `Common/SRP` dependency.
- Invalid SRP data, non-contiguous outer sequence, duplicate/rollback source
  time, unreasonable dt, stale receive/source intervals, disconnect, and epoch
  changes all clear the pose baseline. The next valid frame anchors only; it
  cannot reuse a pre-fault velocity interval.
- The earlier clean-project result before the compatibility-only test extension
  was `98 tests, 0 errors, 0 failures, 0 skipped`. This remains host/offline
  evidence only and does not prove live S3/STM timing, odometry accuracy, TF
  calibration, SLAM, or real-vehicle mapping.

## SRP v4 live integration continuation (2026-08-31)

- Compatibility tests now prove the fixed `0x04` golden vector (`0xC07F`), an
  otherwise identical `0x0C` frame (`0xD844`), all low-nibble flag values with
  ODOMETRY_VALID enforced separately, every unsupported high nibble, and
  producer-owned inner sequence/timestamp values. Current result is 101/0/0/0.
- The physical S3 at `192.168.31.239` sustained raw radar data, but the gateway
  diagnostics observed only outer type 1: `raw_frames` increased while
  `opaque_frames=0` and `chassis_frames=0`. The ROS decoder was never invoked.
- Consequently, live chassis telemetry, odom/TF, first-frame baseline, and
  stale/reconnect behavior are not failures of decoded data; they are blocked
  before the ROS chassis decoder by the absence of S3RD type 2 input.
- A pre-existing slam_toolbox process was visible and publishes `/tf`
  endpoints, but no `odom -> base_link` transform existed before the test and
  the recorded bag contained zero TF messages. This turn did not start or use
  slam_toolbox and makes no mapping claim.

## 2026-08-31 interleaved SRP v4 telemetry audit

- The only direct `chassis_frames_` increment is in
  `s3_ydlidar_bridge/src/bridge_node.cpp` at the outer type-2 branch. It
  currently counts every type-2 payload before inspecting the inner SRP
  message ID.
- The only direct `chassis_anchored_` increment is the
  `ChassisSubmitStatus::kAnchored` branch in the same function.
- Diagnostic `chassis_decoder_rejected` is the state adapter's
  `decode_rejected` counter. Its only increment is the failed chassis decoder
  result in `smartcar_state_bridge/src/chassis_state.cpp`.
- Diagnostic `chassis_sequence_rejected` is the state adapter's
  `sequence_rejected` counter. Its only increment is `rejectSequence()` in the
  same file; the current caller treats outer jumps as chassis sequence faults.
- The active bridge tracked outer sequence by
  `(device_id, stream_id, message_type)`, producing two domains and false gaps
  when the sender interleaved type 1 and type 2.
- Runtime snapshot: one published 8765 bridge container,
  `smartcar-scan-safe-0831`; the S3 connection was established and
  `allow_live_telemetry`, `enable_live_odom`, `publish_odom`, and `publish_tf`
  were all false.
- A NET_RAW capture sharing that existing container's network namespace read
  9,400 TCP payload bytes in 15 segments and reconstructed 120 S3RD frames.
  There were zero TCP stream gaps/overlaps, zero outer CRC errors, and zero
  outer sequence discontinuities. Raw outer sequence was exactly
  `30934..31053`, including type 1 at sequence `30957` among type-2 frames.
- Type-2 counts in that capture were: inner `0x10` IMU 32, `0x14` wheel 31,
  `0x15` chassis 26, and other valid `0x11` telemetry 30. Every parsed inner
  SRP frame had a matching declared total length, valid CRC16-CCITT-FALSE, and
  valid EOF.
- Chassis inner sequence is the interleaved SRP global sequence. Examples in
  the contiguous capture are chassis `148 -> 159 -> 165 -> 171 -> 177 -> 182`.
  Requiring chassis-only `+1` would reject valid input; forward gaps must be
  admitted and bounded by source timestamp/dt instead.
- The sender did not skip outer sequence in this capture, so the requested
  hard-stop condition did not trigger and host correction may proceed.

## 2026-09-01 interleaved SRP v4 completion and live validation

- `ChassisOdomTracker::clearBaseline()` now clears the previous pose plus the
  last inner sequence, source timestamp, and host receive timestamp. A valid
  frame after any reject/stale event therefore re-anchors cleanly.
- Fresh forced-reconfigure Docker build completed all four packages. Full
  result after the non-chassis outer-sequence regression test: `110 tests, 0
  errors, 0 failures, 0 skipped`.
- With all four gates false for a real 30-second window, the connection stayed
  `connected`; `opaque_frames` and `srp_frames` increased by 2,302, strict
  chassis `0x15` frames increased by 484, IMU by 607, wheel by 616, and other
  telemetry by 595. Decoder, outer-sequence, outer-CRC, queue-overflow, and
  stale counters did not increase.
- After the precheck, a temporary all-true bridge observed the stationary real
  stream for 120 seconds. `chassis_frames` increased by 4,779 and all 4,779
  updates were accepted and published as `/odom`; decoder, inner-sequence,
  odom, outer-CRC, outer-sequence, and queue counters remained zero. `tf2_echo
  odom base_link` returned a continuous transform after initial discovery.
- The all-true bridge was stopped and a single final bridge was started with
  `allow_live_telemetry=false`, `enable_live_odom=false`, `publish_odom=false`,
  and `publish_tf=false`. S3 reconnected to that final safe instance. No
  vehicle, RViz, SLAM, replay, or synthetic data was used; no mapping claim is
  made.

## 2026-09-01 real SLAM integration

- Fresh safe preflight found one SRP v4 bridge on TCP 8765, real `/scan` near
  5 Hz, and no `/odom`, TF, or map before runtime gates were enabled.
- The default `docker-ros2-dev` all-true run was stale and discarded. The
  accepted runtime used `srp_interleave_0831-ros2-dev` plus its matching
  install volume.
- `robot_state_publisher` supplied `base_link -> laser_frame`; the bridge
  supplied `odom -> base_link`; and `slam_toolbox` supplied `map -> odom`.
  No temporary TF publisher was started. RViz used Fixed Frame `map` with
  `/scan`, `/map`, TF, and RobotModel enabled.
- The fixed laser extrinsic is from the existing provisional URDF value
  `(0.200, 0, 0.155) m`; `sensors.yaml` remains `measured: false`, so physical
  extrinsic calibration is not claimed even though the ROS chain resolved.
- Stationary acceptance retained real scans, odometry, TF, and OccupancyGrid.
  The manual-motion bag is 343.300 s with 25,013 messages and all requested
  topics. Saved map is `maps/slam_live_20260901.pgm/.yaml`.
- The final combined run had 49 ready-queue overflows, two outer-sequence
  rejects, 12 scan timeouts, and one reconnect. These are residual stability
  risks. The final safe bridge is the only 8765 owner and all four live gates
  are false.

## 2026-09-01 mapping console and map reset

- The desktop one-click entry must launch the mapping console, not the
  display-only `open_rviz2.ps1`; otherwise an operator can open RViz without
  bringing up real SLAM/odom/TF.
- The console uses only the previously validated Compose project and image.
  It identifies bridge candidates by the host TCP-8765 publication, starts a
  single named mapping container, and restores a single named safe bridge.
- A mapping start is accepted only after node presence, `connected` and
  non-stale diagnostics, a real `/scan` in `laser_frame`, and a real `/odom`
  with child `base_link`. It never substitutes temporary transforms or
  synthetic messages when the S3 stream is unavailable.
- Map cleanup is recoverable: `Microsoft.VisualBasic.FileIO.FileSystem`
  sends only map artifacts under `ROS2_WIN/maps/` to the Windows Recycle Bin.
  Existing rosbag data under `ROS2_WIN/bags/` is outside the cleanup scope.
- In the current reset test, the cleanup succeeded but the S3 bridge had no
  live connection/data. The fresh session was therefore not accepted as a
  real mapping run; Stop restored false gates and a scan-only safe graph.
