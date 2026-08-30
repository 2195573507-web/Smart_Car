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
  must not access the radar UART or alter STM32 UART2/SRPv4/PWM/BLE paths.
- The inspected YDLIDAR SDK and ROS2 driver archives and the radar-material
  directory contain no binary/raw scan capture suitable for an official sample
  replay. Offline fixtures are therefore deterministic SDK-format test bytes,
  explicitly not a real-device capture.
- Cross-project protocol audit: `ESPS3/docs/S3_YDLIDAR_X3PRO_TEST.md` is limited
  to UART1/GPIO44 raw capture, while `DOCS/architecture/data_flow.md` and
  `DOCS/ESP32/S3_ARCHITECTURE.md` mark the Wi-Fi radar bridge contract as
  future work. The existing STM-S3 `AA 55` SRPv4 frame is a separate control/
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
