# ROS2_WIN Progress Log

## 2026-09-01 real SLAM integration

- Fresh preflight found one safe SRP v4 bridge with real `/scan` near 5 Hz;
  `/odom`, `/tf`, `/tf_static`, and `/map` were absent while all four live
  publication gates were false.
- A first all-true attempt used the stale default `docker-ros2-dev` image and
  was discarded; no evidence from it was used. The corrected run used the
  `srp_interleave_0831-ros2-dev` image and matching install volume.
- Started `robot_state_publisher`, `slam_toolbox`, and RViz in the existing
  bridge container. No temporary TF or `/cmd_vel` was used. Verified
  `base_link -> laser_frame`, `odom -> base_link`, and `map -> odom`; RViz
  config has Fixed Frame `map` with `/scan`, `/map`, TF, and RobotModel.
- Stationary 120-second gate passed with continuous scan/odom and map output;
  diagnostics showed no SRP decoder, outer CRC, or sequence faults in the
  accepted period. A real bag was recorded while the vehicle was then moved
  manually and slowly.
- Rosbag finalized at `bags/slam_live_20260901`: 343.300 s, 25,013 messages,
  `/scan` 1,215, `/odom` 3,107, `/tf` 20,278, `/tf_static` 1, `/map` 69,
  `/diagnostics` 343. Map saved to `maps/slam_live_20260901.pgm/.yaml`.
- Final safe bridge `srp_interleave_0831-safe-final3` is the only 8765 owner;
  `allow_live_telemetry`, `enable_live_odom`, `publish_odom`, and `publish_tf`
  are false. The temporary RViz container was removed.
- Runtime warnings retained as evidence: the combined run ended with 49 ready
  queue overflows, 2 SRP outer-sequence rejects, 12 scan timeouts, and one
  reconnect. These do not invalidate topic presence but prevent claiming a
  clean stability window.

## 2026-09-01 mapping console and map reset

- Added `docker/open_mapping_console.ps1` and `docker/mapping_session.ps1`.
  The console starts the real bridge, `robot_state_publisher`,
  `slam_toolbox`, and RViz; it offers Start Mapping, Save Map, Clear Saved
  Maps And Reset SLAM, Stop And Restore Safe Mode, and Open Session Log.
- Updated `C:\Users\至亲\Desktop\Smart Car RViz2.lnk` in place and added
  `C:\Users\至亲\Desktop\Smart Car Mapping.lnk`; both launch
  `open_mapping_console.ps1` with `ExecutionPolicy Bypass` and the
  description `Start Smart Car real SLAM mapping console`.
- The Clear action stops the session, restores a safe bridge before the
  recoverable cleanup, moves only `.pgm`, `.yaml`, `.posegraph`, and `.data`
  under `maps/` to the Windows Recycle Bin, then attempts a new live session.
  `bags/` is never traversed by that action.
- Actual Clear validation moved `slam_live_20260901.pgm` and
  `slam_live_20260901.yaml` to the Recycle Bin. Afterwards `maps/` contained
  only `.gitkeep`; `bags/` remained seven files totaling 29,066,820 bytes.
- The immediate fresh session launched all intended ROS components but current
  physical S3 data was absent. Its strict live scan/odom gate did not pass;
  the test was stopped and the script restored `smartcar-mapping-safe`.
  Final ROS graph exposes only `/scan` and `/diagnostics`, with all four gates
  false. No `/cmd_vel`, STM/S3, serial, or protocol action occurred.
- PowerShell parser validation passed for both console scripts. A GUI smoke
  test opened a responsive window titled `Smart Car Mapping` and closed only
  that test process; no mapping session was started by the UI test.

## 2026-08-29 P1 mapping implementation start

- Read `C:\Users\至亲\Downloads\P1_MAPPING_PLAN.md`, the existing bridge
  design/report, ROS2_WIN task files, and protocol/hardware references before
  editing.
- Confirmed Docker Desktop Linux is available (`29.7.2`, `linux/amd64`) but
  the current image does not yet contain the P1 SLAM/state dependencies.
- Confirmed the only safe host-side scope is a gated/offline implementation:
  the S3 telemetry contract and wheel freshness fields are not frozen, so no
  live `/odom` publisher will be enabled by default.
- Planned work: bounded single-owner gateway dispatch, telemetry decoder
  interface and diagnostics, synthetic wheel FIFO/kinematics tests, URDF/TF
  and mapping bringup, then container build and layered verification.

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

## 2026-08-30 P1 completion pass

- Reviewed the resumed P1 workspace against `P1_MAPPING_PLAN.md` and the
  independent source/build audits. Confirmed that `Common/SCBP_CAN` is absent;
  no duplicate SCBP parser or guessed wheel payload layout was added.
- Hardened `TelemetryDecoder`: live input now requires both configured and
  present source/destination identities, only reviewed wheel type `0x0210` is
  accepted, and rejected calls clear the output sample before returning.
- Hardened `WheelOdom` configuration validation to reject non-positive or
  non-finite wheel diameter. Added focused tests for identity, schema, output
  clearing, diameter, live decoder-fault latching, and explicit session epochs.
- A first test pass caught three stale test fixtures that omitted a bound
  connection epoch after `beginSession()`. The fixtures were corrected and the
  complete suite was rerun successfully.
- Initial P1 container pass: four packages built and `74 tests, 0 errors, 0
  failures, 0 skipped`. Compiler stderr was limited to warnings in the
  unchanged official YDLIDAR SDK; this was an intermediate stale-volume
  result and the final forced reconfigure pass is recorded below.
- Added `docs/p1-mapping-evidence/README.md` with the H0-H6 matrix, command
  recipes, artifact template, and explicit non-claims. H0 and host/container
  evidence are complete; H2, H3, H5, and H6 remain blocked by missing live
  protocol/capture/hardware evidence, while H1/H4 are partial.
- Verified the default bridge parameter file with `s3_opaque_message_types: ~`.
  A literal empty `[]` override is rejected by Humble's parameter parser before
  node construction, so it is documented as an external CLI/YAML limitation.

## 2026-08-30 final build audit

- Rebuilt the Docker image after adding the P1 runtime dependencies. The final
  image is `docker-ros2-dev:latest` with digest
  `sha256:aa3d6c3f4708f37d7fe79e25a888d74e723f91dcc57ddb197033a7a941b2f5`.
  The image contains `nav2_lifecycle_manager`, `nav2_map_server`,
  `slam_toolbox`, `robot_state_publisher`, `xacro`, and the other declared P1
  packages.
- In fresh named Compose volumes, the final command
  `colcon build --symlink-install --cmake-force-configure --executor sequential`
  built all four packages. The matching `colcon test` and
  `colcon test-result --verbose` completed with `75 tests, 0 errors, 0
  failures, 0 skipped`.
- `p1_mapping`, `p1_localization`, `description`, `localization`,
  `map_server`, and `continue_mapping` launch argument listings were checked;
  default mapping/localization/description smoke launches started their
  expected nodes. A parameter smoke confirmed launch-provided identity values
  remain integer ROS parameters and boolean gates remain booleans.
- The map-server non-default smoke started the official lifecycle manager;
  its deliberate missing-file error confirms the lifecycle path without
  fabricating a map artifact.
- Corrected the posegraph-load helpers for Humble: `DeserializePoseGraph.srv`
  has an empty response section, so the shell and PowerShell helpers now use
  the service-call exit code plus non-empty artifact checks instead of looking
  for a nonexistent `result: 0` response.
- The host result remains bounded to H0 and component/static H1/H4 evidence.
  No real S3 capture, approved SCBP parser, calibrated geometry, integrated
  sensor bag, saved map/posegraph, or vehicle-control evidence was available;
  H2/H3/H5/H6 remain open as documented.

## 2026-08-30 final verification rerun

- From `ROS2_WIN`, an initial compose invocation without `-f docker\\compose.yaml`
  failed before container startup; the same commands with the explicit compose
  file then completed successfully.
- Rebuilt the image as `docker-ros2-dev:latest` and reran the
  complete four-package `colcon build`, `colcon test`, and
  `colcon test-result --verbose`. The observed result is
  `75 tests, 0 errors, 0 failures, 0 skipped`.
- A host cleanup attempt for three verified task-generated temporary paths was
  rejected by the local command policy before execution; no repository files
  were affected and those paths remain outside `ROS2_WIN`.

## 2026-08-30 clean-project verification rerun

- Rebuilt and tested with a new Compose project name (`ros2_final_clean2`) and
  fresh named build/install/log volumes, so stale test XML could not affect the
  result. The image was `ros2_final_clean2-ros2-dev:latest` with digest
  `sha256:b06e22755c504f76adaa58c1fb8d0aa5d2e6db028959c9ea36d667104c440cac`.
- The four-package forced-reconfigure build passed. `colcon test` and
  `colcon test-result --verbose` reported exactly `75 tests, 0 errors, 0
  failures, 0 skipped`.
- All six bringup launch files passed `--show-args`; xacro expansion and shell/
  PowerShell syntax checks passed. A five-second default `p1_mapping` smoke
  showed `robot_state_publisher`, `s3_ydlidar_bridge`, and `slam_toolbox`, with
  `/scan`, `/map`, and `/tf_static` endpoints present and no `/odom` or
  `/cmd_vel` topic publishers. The bridge's expected protocol-not-frozen error
  was the only runtime warning.

## 2026-08-31 SRP v4 chassis-state odometry

- Read current Git status, all relevant workspace/module documentation,
  package manifests, launch/config files, state tests, and the gateway
  telemetry/connection lifecycle before editing.
- Confirmed extensive pre-existing dirty work, including the entire state,
  description, and bringup packages. These changes are being preserved and
  extended in place; no Git cleanup or commit operation is permitted.
- User approved an isolated SRP memory decoder plus authoritative-pose tracker.
  The old wheel fixture path remains intact, outer S3RD framing is unchanged,
  and type-2 telemetry will return before the YDLIDAR decoder.
- Body twist convention: rotate the odom-frame pose delta into the previous
  valid `base_link` orientation; use the shortest yaw delta. The first valid
  frame after startup, invalidity, stale state, disconnect, or epoch change is
  baseline-only.
- Added the pure-memory SRP decoder, chassis state adapter/tracker, and shared
  odom/TF message builder. The old wheel fixture/integrator remains intact.
- Integrated only S3RD type 2 into the new path. It returns from the opaque
  branch before legacy wheel handling and before the YDLIDAR decoder; raw type
  1 scan behavior was not edited.
- Added protocol, semantic, finite-value, pose/twist, yaw-wrap, freshness,
  epoch/disconnect, covariance/timestamp, type-routing, and default ROS graph
  tests. Container execution is still pending.
- Generated the fixed 36-byte vector with CRC `0xC07F`, stored little endian
  as `7F C0`, and recorded the same bytes in test code and the wire contract.
- Ran the four required commands from `ROS2_WIN/docker`: Compose image build,
  four-package symlink build, complete test run, and verbose test-result all
  passed with `98 tests, 0 errors, 0 failures, 0 skipped`.
- Repeated image build, forced-reconfigure colcon build, and tests under the
  new Compose project `srp_v4_clean`; its independent result is also
  `98 tests, 0 errors, 0 failures, 0 skipped`, excluding stale result XML.
- Confirmed the installed launch interfaces still expose
  `allow_live_telemetry=false`, `enable_live_odom=false`,
  `publish_odom=false`, and `publish_tf=false`. The default launch test sees
  no bridge-owned `/odom` or dynamic `/tf` publisher.
- Final source/config scans found no new `/cmd_vel`, controller manager,
  `ros2_control`, identity odom, or temporary `rviz_world` TF path. No live
  odometry process or real-hardware mapping test was started.

## 2026-08-31 SRP v4 live integration continuation

- Re-read the current plan, findings, progress, Git status, decoder, shared
  fixture, and test registration before changing tests.
- Confirmed the decoder already treats payload bits `0x00..0x0F` as the only
  supported mask while independently requiring `ODOMETRY_VALID=0x04`.
- Existing tests covered the fixed `0x04` golden frame but not `0x0C`, the
  complete low/high nibble boundary, or producer-owned inner sequence and
  timestamp values. Added only those focused cases to
  `test_srp_v4_chassis.cpp`; product code and protocol remain unchanged.
- The flags `0x0C` variant with otherwise identical fields has
  CRC16-CCITT-FALSE `0xD844`, stored little endian as `44 D8`.
- Complete Docker colcon build and test passed for all four packages. Current
  result: `101 tests, 0 errors, 0 failures, 0 skipped`.
- Current hardware gate passed before any runtime replacement: Windows showed
  an established `192.168.31.239 -> 192.168.31.101:8765` session, a bounded
  12-second subscriber measured real `/scan` near 4.74 Hz, and diagnostics
  reported connected, stale=false, coverage 360/360.
- The active bridge used the older image `4a44de94...` and its diagnostics did
  not contain the SRP chassis fields. An initial `/proc/1/exe` hash targeted
  the Python `ros2` launcher, so it was not used as bridge-binary evidence.
  `/odom` did not exist and `tf2_echo odom base_link` confirmed no such
  transform; existing `/tf` publishers belonged only to the already running
  slam_toolbox process.
- Replaced only the confirmed port-8765 bridge with the current image and
  C++ PID 22 hash `9c540798...`. Runtime-only overrides set all four live gates
  true; parameter dump confirmed them and `s3_opaque_message_types=[2]`.
- A 120.60-second real-device monitor received 567 LaserScan messages at
  4.704 Hz with a 0.326-second maximum gap. Diagnostics grew by 13,596 raw
  packets and 564 published scans; outer CRC errors, ready-queue overflow, and
  sequence gaps did not increase. YDLIDAR checksum errors increased by one.
- In the same window, `opaque_frames`, `chassis_frames`,
  `chassis_updates_accepted`, `telemetry_accepted`, and `odom_published` all
  changed by zero. No `/odom` message or `odom -> base_link` TF was received;
  chassis decode remained `not_attempted`.
- Saved the requested real-device bag at
  `bags/srp_v4_live_20260831_1453`: 288.25 seconds, 6.5 MiB, 1,357 `/scan`,
  288 `/diagnostics`, zero `/odom`, and zero `/tf`. `/tf_static` had no
  publisher/message and was therefore not added to bag metadata.
- Did not request an S3 power cycle because no chassis baseline or odom stream
  existed; it could not distinguish odom stop/re-anchor behavior. Stale and
  reconnect acceptance remain blocked until S3 actually emits type 2.
- Stopped the temporary all-true runtime and restored current-image container
  `smartcar-scan-safe-0831` with the four gates false. S3 reconnected,
  diagnostics returned connected/stale=false near 5 Hz, and node inspection
  showed only `/scan` and `/diagnostics` application publishers.

## 2026-08-31 interleaved SRP v4 telemetry correction

- Re-read the current plan, findings, progress, protocol contract, bridge
  dispatcher, state adapter, sequence tracker, tests, Git status, and live
  runtime before editing product code.
- Audited all requested chassis counter increment sites. Confirmed that
  `chassis_frames` currently means outer type-2 frames and that outer sequence
  is incorrectly split into per-message-type domains.
- Confirmed one active 8765 bridge, an established S3 TCP connection, and all
  four safety gates false. No ROS process was started or restarted.
- Windows `pktmon` was unavailable without elevation. A temporary NET_RAW
  helper container captured the existing namespace without replacing the
  bridge or injecting data.
- Reconstructed 120 consecutive real S3RD frames (`30934..31053`) with valid
  outer CRC and no TCP/outer sequence gaps. Counted type-2 inner IDs and
  confirmed non-contiguous forward chassis sequences caused by SRP
  interleaving.
- Recorded the approved public-decoder-first design and test/live-validation
  gates. Product implementation is the next phase.

## 2026-09-01 interleaved SRP v4 completion

- Corrected baseline invalidation so sequence/timestamp/host-time history is
  cleared together with the pose baseline; adjusted duplicate/rollback tests
  to prove rejection only while a valid baseline exists and re-anchoring after
  invalidation.
- Rebuilt with `--cmake-force-configure` and reran all package tests:
  `109 tests, 0 errors, 0 failures, 0 skipped`.
- Real false-gated 30-second precheck passed on the established S3 connection:
  `opaque_frames +2302`, `srp_frames +2302`, `chassis_frames +484`,
  `srp_decoder_rejected +0`, `srp_outer_sequence_rejected +0`,
  `outer_crc_error +0`, `ready_queue_overflow +0`, `stale=false`.
  Snapshots are in `evidence/srp_precheck_20260901/`.
- Real stationary all-true 120-second observation passed after the precheck:
  `chassis_frames +4779`, `chassis_updates_accepted +4779`,
  `odom_published +4779`, `chassis_decoder_rejected +0`,
  `chassis_sequence_rejected +0`, `chassis_odom_rejected +0`, and
  `outer_crc_error/sequence/queue +0`. The `/odom` timestamp capture and
  `tf2_echo odom base_link` output are in `evidence/srp_alltrue_20260901/`.
- Stopped the temporary all-true instance and restored the final bridge with
  all four safety gates false. Current S3 TCP state is established on port
  8765, and the final parameter dump confirms all four values are false.
- Added a regression guard so outer-sequence rejection of valid IMU/wheel SRP
  frames cannot clear chassis baseline or increment chassis reject counters;
  rebuilt and reran the suite with final result `110 tests, 0 errors, 0
  failures, 0 skipped`. Restarted the final safe bridge from this build;
  current container is `srp_interleave_0831-safe-final2`, with one established
  S3 connection and all four publication/telemetry gates false.

## 2026-09-02 mapping console missing-container recovery

- Reproduced the reported .NET popup: after a failed 60-second live mapping
  gate removes `smartcar-mapping-session`, the GUI's `Set-Controls` refresh
  called `docker inspect` while `$ErrorActionPreference` was `Stop`; Windows
  PowerShell promoted Docker's `no such object` stderr to a terminating error.
- Hardened `docker/open_mapping_console.ps1` so `Test-MappingRunning` scopes
  native-command errors, returns `false` for an absent/unavailable mapping
  container, and restores the caller's strict error preference.
- PowerShell parser checks passed for both console scripts. An extracted
  `Test-MappingRunning` probe returned `False` with the mapping container
  absent and left the outer preference at `Stop`.

## 2026-09-02 mapping console popup hardening

- The first probe fix was insufficient on the user's Windows host because
  PowerShell 5.1 could still surface the native `docker inspect` failure during
  the GUI event cycle.
- Replaced that probe with `docker ps --filter name=^/smartcar-mapping-session$`
  and `status=running`; an absent container now produces an empty successful
  result and never emits `no such object`.
- Added defensive handling around `Set-Controls`, the timer refresh, and form
  closing so a transient Docker/status-file error cannot escape the WinForms
  event loop as a .NET dialog.
- Parser checks, missing-container probe, Docker empty-result check, and GUI
  open/close smoke test all passed. No mapping session was started.

## 2026-09-02 stale GUI process and worker probe cleanup

- The reported popup was traced to an already-running `Smart Car Mapping`
  PowerShell process (PID `52580`, started before the second fix). Its in-memory
  event handlers still contained the old `docker inspect` implementation, so
  editing the script on disk could not change that window.
- Closed only that stale mapping-console process; the `smartcar-mapping-safe`
  container and its TCP-8765 listener were left running.
- Removed the last worker-side `docker inspect` call from
  `mapping_session.ps1`; all mapping-console container-state checks now use
  non-throwing `docker ps` filters.
- Started the current script in a fresh PowerShell process, confirmed a
  responsive `Smart Car Mapping` window for five seconds, and closed it cleanly
  with exit code 0. No .NET popup occurred.

## 2026-09-03 non-fatal mapping live gate

- Audited both desktop shortcuts: `Smart Car RViz2.lnk` and `Smart Car
  Mapping.lnk` target `docker/open_mapping_console.ps1` without a strict-gate
  argument. The launch starts RViz, SLAM, the TCP bridge, and robot description
  together inside `smartcar-mapping-session`.
- Separated a 60-second incomplete live-data gate from genuine startup failures.
  The default timeout now writes `实时里程计/TF未就绪，暂不能建图` and returns
  successfully without stopping TCP 8765, removing the mapping container, or
  fabricating any ROS data. The strict original behavior is opt-in through
  `-StrictLiveGate` on either console script.
- Extended the live acceptance check to require real diagnostics, `/scan`,
  `/odom`, and an `odom -> base_link` TF lookup. Save Map remains disabled
  until that full live-ready status is observed.
- Ran the default worker with incomplete live odometry/TF. It completed after
  the gate with exit code 0; `smartcar-mapping-session` remained the sole
  TCP-8765 owner. Container inspection found live `robot_state_publisher`,
  `s3_ydlidar_bridge`, `async_slam_toolbox_node`, and `rviz2` processes.
  Independent `/odom` and TF probes timed out, so no mapping-ready claim was
  made. An explicit `Stop` then removed that session and restored one
  false-gated `smartcar-mapping-safe` bridge.

## 2026-09-03 automatic navigation increment

- Added the scoped `smartcar_motion_gateway` package. It owns TCP 8766,
  encodes the local SCBP-shaped velocity frame, runs at 20 Hz, clamps to
  `0.10 m/s` and `0.30 rad/s`, and never sends unless both explicit motion
  gates plus scan/odom/TF/lease health are true.
- Added the local ROS motion-control compatibility record because the
  requested external `DOCS/protocol/ros-motion-control-v1.md` is absent. The
  gateway therefore keeps `protocol_ready=false` by default and does not
  claim a released S3 firmware contract.
- Added saved-map Nav2 launch with `map_server`, AMCL, planner, controller,
  and lifecycle managers. Nav2 controller output is scoped to `/nav2/cmd_vel`
  before the motion gateway; arbitrary `/cmd_vel` is not consumed by the
  gateway.
- RViz `SetGoal` now requests `ComputePathToPose` and publishes the returned
  `nav_msgs/Path` as `/smartcar/goal_preview`. Only the `~/start` Trigger calls
  `NavigateToPose`; `~/cancel`, health loss, action rejection, and failure
  clear state and publish a zero command.
- Added frame validation for `laser_frame`, `odom -> base_link`, and finite
  odometry samples. Added a one-shot zero frame when a previously active
  command loses health or freshness.
- Added package-level goal confirmation pytest coverage for pending/start/
  cancel/preview semantics and exported `ament_cmake_pytest` dependency.
- Fresh Compose project `smartcar_ros2_final2` passed image build, forced
  reconfigure build, test, and verbose result: `129 tests, 0 errors, 0
  failures, 0 skipped`. No real motion, S3 lease, measured extrinsic, or
  vehicle mapping/navigation acceptance was run.
- Updated the desktop mapping worker and console to carry explicit
  `-LaserExtrinsicsMeasured`, `-LaserXyz`, and `-LaserRpy` inputs. They default
  to the safe rejected state, so an operator cannot bypass the calibration
  gate through the legacy P1 entry point.

## 2026-09-03 continuation verification

- Re-ran PowerShell AST parsing for `docker/mapping_session.ps1` and
  `docker/open_mapping_console.ps1`; both reported zero parse errors after the
  laser-extrinsics argument quoting change.
- Re-ran the requested Docker checks from `ROS2_WIN/docker`: image build,
  symlinked `colcon build`, `colcon test`, and verbose test results all passed.
- Current result is `129 tests, 0 errors, 0 failures, 0 skipped` across five
  ROS packages. Docker reported only pre-existing orphan-container warnings;
  no cleanup was performed.
- No live S3 control lease, released 8766 wire contract, measured laser
  extrinsics, real map save/load, or vehicle navigation acceptance was run.

## 2026-09-03 mapping artifact completion

- Extended the desktop `p1_mapping.launch.py` path to record observation-only
  rosbag topics to a timestamped `/ws/bags/mapping_YYYYMMDD_HHMMSS` URI.
- The mapping console's save action now requires its existing live-ready gate,
  then saves both the Nav2 `.yaml`/`.pgm` map and slam_toolbox
  `.posegraph`/`.data` graph with one prefix. The active rosbag is finalized
  when the mapping container stops.
- Updated the operator label and design record to reflect all three artifact
  classes; no control topic is recorded or published by the mapping workflow.
- Added a default-unchecked GUI confirmation for measured laser extrinsics plus
  editable XYZ/RPY fields. The desktop Start action refuses to launch until
  the operator confirms measured values; launch-level rejection remains in
  place for direct invocations.
- Started the updated WinForms console in a visible PowerShell process,
  observed the `Smart Car Mapping` window, and closed only that test process
  cleanly. The new controls did not raise an event-loop or close-time error.
- Changed named-container cleanup to stop for up to 10 seconds before removal,
  allowing rosbag2 and slam_toolbox to flush metadata instead of using a
  force-remove SIGKILL on normal Stop/Clear transitions.
