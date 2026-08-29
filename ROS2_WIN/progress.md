# ROS2_WIN Progress Log

## 2026-08-28

- Read mandatory workflow, brainstorming, and planning instructions.
- Checked `git status`; only the pre-existing
  `ROS2_WIN/S3_ROS2_STM_BUILD_AUDIT_PLAN.md` is untracked.
- Audited ROS2 YDLIDAR SDK and ROS2 driver archive manifests and key metadata.
- Confirmed official driver version `1.0.1`, MIT licensing, and serial/control
  behavior that cannot be used as the runtime entry point.
- Asked for confirmation of the explicit no-frozen-protocol live transport
  boundary before implementation.
- Added the ament package, Docker/Compose files, replay and unconfigured
  transport interfaces, bounded TCP chunk assembler, sequence tracker, and
  ROS LaserScan mapper.
- Imported the official SDK parser core and Linux channel backends with its MIT
  license; removed high-level CYdLidar/ETLidar, filters, samples, and Windows
  backend from the vendor tree.
- Added the project-only `parseMemoryChannel` adapter to the official driver;
  it temporarily reads from a caller-owned channel and restores parser state.
- Added gtests for split/sticky chunks, illegal lengths, sequence jumps,
  official checksum acceptance/rejection, and LaserScan field mapping.
- `docker compose build` was attempted from `ROS2_WIN/docker`; Docker Desktop's
  Linux engine is not running (`dockerDesktopLinuxEngine` named pipe missing),
  so container build/test evidence is unavailable in this environment.
- Starting Docker Desktop did not recover the engine; the required build, test,
  and test-result compose commands each timed out at the Docker API boundary.
- Static checks passed: XML and launch Python syntax parse, authored files have
  no trailing whitespace, no forbidden serial/control calls occur in custom
  bridge code, and no build/install/log tree is tracked.
- Added the durable delivery report under `ROS2_WIN/docs/final-report.md`.
- A non-destructive `DockerCli -SwitchLinuxEngine` diagnostic also timed out;
  the engine remains unavailable and no further Docker state changes were made.
- Source-level hardening added: bounded framing now terminates safely when an
  extractor reports invalid input on an empty buffer, sequence wrap is treated
  as a jump until the S3 contract defines wrap semantics, and scan mapping uses
  strict-C++17 PI constants rather than a non-standard `M_PI` macro. Added
  framing tests for both edge cases and made `<utility>` explicit in the
  transport interface.
- Confirmed the inspected SDK/driver archives and radar-material directory do
  not contain a binary YDLIDAR sample frame. The test suite therefore keeps its
  deterministic SDK-format packet as a negative/field fixture and makes no
  official-sample or live-device claim. Scan mapping now rejects Q6 angles
  outside the official normalized `0..360` degree range and tests that case.
- Final static checks passed: Compose configuration rendering, package XML
  parsing, launch structure check, CMake vendor-source existence, no authored
  trailing whitespace, no tracked `build/`/`install/`/`log/`, and no custom
  source invocation of serial/control or live-socket APIs. Docker/colcon remain
  blocked solely by the unavailable Docker Desktop Linux engine.
- Decoder success now clears a reused error string, and the mapper rejects a
  non-finite/non-positive configured angle increment before indexing, avoiding
  undefined behavior for malformed parameters.
- Pruned three unreferenced vendor files (`core/math/angles.h` and
  `core/network/PassiveSocket.*`) and removed `PassiveSocket.cpp` from CMake;
  source-audit notes now identify the exact retained network/parser
  dependencies.
- CMake now explicitly enforces C++17 so the `std::optional` transport API does
  not depend on an external toolchain default.
- TCP assembly now trims oversized input before insertion, keeping peak buffer
  allocation bounded even when one received chunk exceeds the configured cap.
- Hardened ROS parameter declarations to use Humble-native `double`/`int64`
  types before converting to internal float/size types, and added explicit
  stale-time bounds.
- Added replay/unconfigured transport gtests and explicit Linux `libm` linkage
  for the official parser's trigonometric symbols.
- Re-ran the required Docker build after the hardening changes; Docker Desktop
  returned HTTP 500 on the `dockerDesktopLinuxEngine` ping. The Windows
  `com.docker.service` is stopped/manual and cannot be started without the
  required host permissions, so colcon remains externally blocked.
- Made `docs/final-report.md` self-contained with both official archive paths,
  versions, SHA-256 values, and license references.
- Removed the unnecessary read-only channel `setDTR` override so custom bridge
  code has no DTR method or control call at all; the SDK base default remains
  unused during memory parsing.
- Re-attempted the exact `colcon build`, `colcon test`, and
  `colcon test-result --verbose` compose commands; each timed out while Docker
  returned HTTP 500 from the Linux-engine `_ping`, before a container started.
- Docker backend logs pinpointed `HCS_E_HYPERV_NOT_INSTALLED` while importing
  `docker-desktop`; DISM confirmed Error 740 because this session is not
  elevated. Added `docker/enable-wsl2.ps1` as the repeatable administrator
  remediation without altering project or Docker data.
- User confirmed both optional features report `Enabled`; Docker still fails
  because the hypervisor boot setting is not confirmed. Extended the admin
  script with `bcdedit /set hypervisorlaunchtype auto`; this requires one more
  reboot after the script completes.
- User rebooted and reran the WSL2 remediation. `docker version` now reports
  both Client and Linux Server 29.7.2 under the `desktop-linux` context, and
  `docker info` reports `ostype=linux`.
- The first post-recovery build exposed a `set -u` conflict in the ROS 2 setup
  script; the entrypoint now enables nounset only after sourcing ROS 2.
- Fixed Humble CMake link-signature compatibility and SDK global `node_info` /
  `rclcpp::Time` API compatibility.
- Fixed the project-only memory-parser drain so all samples in a packet are
  handed through the official `waitPackage` path.
- Final verification passed in Docker: `docker compose build`, colcon build,
  colcon test, and `colcon test-result --verbose` all succeeded. The current
  result is 25 tests, 0 errors, 0 failures, 0 skipped. Remaining compiler
  output consists of warnings from the vendored official SDK.

## 2026-08-29

- Rebuilt the package after correcting the protocol identity-error fixture and
  re-ran colcon test/result: 25 tests, 0 errors, 0 failures, 0 skipped.
- Verified the launch file and deterministic TCP sender parse in the container
  without attempting to write to their intentionally read-only source mounts.
- Ran a temporary container build and test pass with AddressSanitizer and
  UndefinedBehaviorSanitizer flags; it completed successfully in `/tmp`.

## 2026-08-29 Windows bridge startup diagnostic

- `docker compose -f .\\docker\\compose.yaml config --services` returned
  `ros2-dev`.
- `docker compose -f .\\docker\\compose.yaml up -d ros2-dev` started the
  default `bash` command, which exited cleanly; ordinary `compose ps` was
  therefore empty and its log output was empty.
- The requested container socket probe reported that this image has neither
  `ss` nor `netstat`.
- The README/Compose live command was used unchanged to start the bridge as
  the container foreground process with TCP transport on port 8765.
- The running one-off container exposes `0.0.0.0:8765->8765/tcp`; its
  `/proc/net/tcp` contains `00000000:223D ... 0A`, i.e. `0.0.0.0:8765` in
  LISTEN state. The bridge PID is 22 and node parameters report `tcp` and
  `0.0.0.0:8765`.
- Windows `Get-NetTCPConnection -LocalPort 8765 -State Listen` reports
  wildcard listeners on `::1` and `::`; `Test-NetConnection` from WLAN
  address `192.168.31.101` to port 8765 succeeded.
- No source change or S3RD protocol-field change was needed.
