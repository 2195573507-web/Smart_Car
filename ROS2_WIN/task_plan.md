# ROS2_WIN S3 YDLIDAR Bridge Task Plan

## Objective

Create a Windows-hosted ROS 2 Humble workspace under `ROS2_WIN/` that can
replay and validate official YDLIDAR frames, while leaving the live S3 Wi-Fi
transport behind an explicit interface until its protocol is frozen.

## Constraints

- Only files under `ROS2_WIN/` may be added or changed.
- The ESP32-S3 owns the radar UART; the ROS 2 node must never open a serial
  device or send radar control commands.
- YDLIDAR protocol parsing, checksum, distance/angle conversion, and scan
  mapping must come from the official SDK with minimal adaptation.
- No real S3 gateway wire format may be invented.

## Phases

| Phase | Status | Deliverable |
| --- | --- | --- |
| Baseline and source audit | complete | Source/version/license inventory and protocol gap record |
| Design approval | complete | Approved bridge architecture and test boundary |
| Workspace and vendor import | complete | Docker files, ament package, scoped official sources |
| Transport and replay adapters | complete | Bounded latest-only interface and offline fixture path |
| Bridge and tests | complete | `/scan` publisher plus framing/validation/field tests |
| Build and verification | complete | Docker build succeeded; four-package colcon build and all 101 current tests passed |
| Final report | complete | Files, reuse boundaries, blockers, and handoff conditions recorded |

## 2026-08-29 Diagnostic extension

| Phase | Status | Deliverable |
| --- | --- | --- |
| Windows bridge startup and listener check | complete | Foreground TCP bridge on `0.0.0.0:8765`, Docker/Windows listener evidence, no source changes |

## 2026-08-29 P1 mapping implementation

The host-side portion of `P1_MAPPING_PLAN.md` is being implemented inside
`ROS2_WIN/` only. Live wheel telemetry remains hard-gated until the S3
envelope, SCBP payload contract, source freshness fields, and real captures are
provided; no ROS receive timestamp is treated as a wheel sample timestamp.

| Phase | Status | Deliverable |
| --- | --- | --- |
| P1-0 evidence and boundary record | complete | H0-H6 matrix, calibration/protocol gates, and reproducible evidence layout |
| P1-2 gateway dispatch and bounded queues | complete | One TCP owner with raw-radar and explicitly gated opaque telemetry dispatch |
| P1-3 offline wheel kinematics/odom library | complete | Synthetic-only wheel FIFO, freshness/epoch gates, exact arc integration, and tests |
| P1-4 description and TF bringup | complete | URDF/xacro, provisional static sensor TF, mapping launch, and RViz config |
| P1-5 SLAM/map workflow | complete (host workflow) | slam_toolbox configuration and map-save helpers exist; real bag/map save-load remains unverified (H5 blocked) |
| P1-6 layered host verification | complete | Static, unit, replay, container build, and launch smoke evidence; H2-H6 hardware gates remain open |

## Errors Encountered

| Error | Attempt | Resolution |
| --- | --- | --- |
| Docker Desktop Linux engine unavailable | 1 | Recorded as an environment blocker; continue with static audit and report that container build/test could not run. |
| Docker CLI/API remained unavailable after Desktop start | 2 | `docker version/info` and all three required `compose run` commands timed out; no build/test result is claimed. |
| Linux-engine switch did not return before timeout | 3 | Recorded as the same external Docker availability blocker; no further engine manipulation performed. |
| Host Python runtime lacked encodings / PDF path encoding failed | 1 | Switched to the bundled workspace Python runtime and ASCII glob discovery; manual text extraction then succeeded. |
| Initial PDF skill path was stale | 1 | Located and read the active bundled `pdf` skill version before continuing. |
| Docker entrypoint nounset conflict | 1 | Source ROS 2 setup before enabling `set -u` in `docker/entrypoint.sh`. |
| Humble CMake link signature conflict | 1 | Used plain `target_link_libraries` for targets also passed to `ament_target_dependencies`. |
| SDK node type/time API mismatch | 1 | Used the SDK's global `::node_info` and Humble-compatible `rclcpp::Time` conversion. |
| Memory parser drained only first sample | 1 | Kept calling the official `waitPackage` while its packet-owned sample state remained. |
| Compose default service exited | 1 | Expected `bash` command exited cleanly; started the documented bridge command as a foreground one-off container. |
| `ss`/`netstat` unavailable in image | 1 | Used `/proc/net/tcp` to verify the equivalent LISTEN entry without changing the image. |
| Epoch-bound odometry fixtures omitted the bound epoch | 1 | Updated tests to carry the explicit session epoch; missing epochs remain rejected after `beginSession()`. |
| Humble rejects a literal empty `s3_opaque_message_types: []` override before node construction | 1 | Use YAML null (`~`) for the default empty allow-list and document the ROS 2 parser limitation. |
| Compose command run from workspace root without an explicit file | 1 | Re-ran with `-f docker\\compose.yaml`; no container was started by the failed invocation. |
| Host cleanup command rejected by local policy | 1 | Left the three verified task-generated temporary paths untouched and recorded the limitation; repository state was unaffected. |

## 2026-08-31 SRP v4 chassis-state odometry

This increment consumes a complete SRP v4 chassis-state frame carried only by
S3RD outer `message_type=2`. It remains read-only, preserves the existing
YDLIDAR `/scan` path, and keeps all live telemetry/odom/TF gates disabled by
default.

| Phase | Status | Deliverable |
| --- | --- | --- |
| SRP contract and golden vector | complete | Frozen local wire contract, independent memory decoder, and shared bytes/CRC |
| Authoritative pose and twist adapter | complete | STM pose conversion, baseline/freshness/epoch gates, body-frame twist |
| Single-gateway integration | complete | Type-2 opaque dispatch without YDLIDAR decode and one odom/TF owner |
| Offline tests | complete | Decoder, units, twist, reset, timestamp, publication, and isolation coverage |
| Docker/colcon verification | complete | Required commands passed; fresh `srp_v4_clean` project reports 98/0/0/0 |
| Final handoff report | complete | Changed scope, impact, defaults, golden vector, and STM/S3 interfaces |

### SRP increment errors

| Error | Attempt | Resolution |
| --- | --- | --- |
| Parallel context scan returned exit 1 when an optional `rg` search had no matches | 1 | Re-ran only the required reads and treated no matches as a normal search result |
| Initial PowerShell CRC helper overflowed during `UInt16` left shifts | 1 | Recomputed with a masked 32-bit intermediate; C++ golden-vector tests confirmed `0xC07F` |

## 2026-08-31 SRP v4 live integration continuation

This continuation first proves decoder compatibility offline, then checks the
current physical S3/TCP and `/scan` state. It must stop at the offline boundary
when no real device data is present; replay, synthetic odometry, temporary TF,
and SLAM are excluded.

| Phase | Status | Deliverable |
| --- | --- | --- |
| Flags/sequence/timestamp compatibility audit | complete | Added only missing `0x0C`, nibble-mask, variable sequence/time tests |
| Colcon build and offline tests | complete | Four packages; 101 tests, 0 errors/failures/skips |
| Current S3/TCP and `/scan` gate | complete | Peer `192.168.31.239` established; bounded real `/scan` about 4.74 Hz |
| Single-owner live odom start | complete | No competing `/odom` or `odom -> base_link`; command-line-only true gates verified |
| 120-second odom/TF/scan acceptance and rosbag | blocked | Real scan passed, but S3 sent zero type-2/chassis frames and therefore zero odom/TF |
| User-driven stale/reconnect observation | blocked | No chassis baseline or odom stream exists, so a power cycle cannot test odom recovery |

All four file defaults remain false. No Git commit or firmware/protocol change
is permitted.

### Live continuation errors

| Error | Attempt | Resolution |
| --- | --- | --- |
| `/odom` inspection returned unknown topic before opt-in | 1 | Treated absence as owner evidence and checked TF independently |
| `/proc/1/exe` hashed the Python `ros2` launcher | 1 | Located the C++ child PID 22 and matched it to build/install SHA-256 |
| Bounded `ros2 topic hz` exited through its timeout | 1 | Kept its emitted 4.74 Hz measurement and used a normal-exit Python monitor for the 120-second gate |

## 2026-08-31 interleaved SRP v4 telemetry correction

This increment is limited to `ROS2_WIN/`. It does not change S3 or STM
firmware, S3RD/SRP protocol bytes, vehicle control, or the four false safety
defaults. Real validation must not start RViz or SLAM.

| Phase | Status | Deliverable |
| --- | --- | --- |
| Counter and sequence audit | complete | All requested increment sites identified; per-message-type outer tracking identified as the false-gap cause |
| Real S3RD metadata capture | complete | 120 CRC-valid frames captured from the active TCP connection with no TCP or outer-sequence gap |
| Design and evidence record | complete | Public SRPv4 decode, typed dispatch, sequence, baseline, and test boundaries frozen |
| Implementation | complete | Common SRPv4 decoder, message-id routing, global outer tracker, and non-contiguous chassis sequence admission |
| Focused and regression tests | complete | Interleaving, strict chassis errors, sequence/time/stale/reconnect, and type-1/type-2 continuity |
| Docker build and full tests | complete | Fresh forced-reconfigure build; `110 tests, 0 errors, 0 failures, 0 skipped` |
| 30-second false-gated live precheck | complete | Connected real S3; `opaque_frames +2302`, `srp_frames +2302`, `chassis_frames +484`, no decoder/outer/CRC/queue faults |
| 120-second all-true odom/TF check | complete | Stationary real S3; `chassis_updates_accepted +4779`, `odom_published +4779`, TF query returned continuously |
| Final safety snapshot | complete | All-true instance stopped; one final 8765 owner, all four gates false, no vehicle/RViz/SLAM start |

### Interleaving audit errors

| Error | Attempt | Resolution |
| --- | --- | --- |
| Windows `pktmon` access denied without elevation | 1 | Used a temporary NET_RAW helper sharing the existing bridge network namespace; the bridge was not restarted or replaced |
| Initial documentation patch used stale `findings.md` tail context | 1 | The patch was rejected atomically; re-read exact tails and applied bounded append patches |

## 2026-09-01 real SLAM integration

| Phase | Status | Deliverable |
| --- | --- | --- |
| Current bridge/scan/odom/TF gate | complete | Fresh runtime snapshot and evidence directory |
| Real description, SLAM, and RViz startup | complete | No temporary TF; Fixed Frame `map`; `/scan`, `/map`, TF displays |
| 120-second stationary acceptance | complete | Real scan/odom/TF/map and diagnostics window |
| Human slow-motion mapping and artifacts | complete | Operator-driven motion, rosbag, saved map |
| Safe shutdown and report | complete | All-true stack stopped, false gates restored, final evidence |

## 2026-09-01 mapping console and map reset

The existing desktop RViz shortcut now opens the real-SLAM mapping console.
The implementation stays within `ROS2_WIN/`, keeps all normal bridge defaults
false, and never publishes `/cmd_vel` or changes STM/S3 firmware or protocol.

| Phase | Status | Deliverable |
| --- | --- | --- |
| Operator console and desktop entry | complete | `open_mapping_console.ps1`; existing desktop shortcut targets it; GUI smoke check passed |
| Real mapping start and RViz configuration | complete | Named mapping container starts description, bridge, slam_toolbox, and RViz with live topic gates |
| Save, clear, and reset control | complete | Timestamped map save; recoverable `.pgm/.yaml/.posegraph/.data` cleanup; fresh-session gate |
| Clear/reset live failure recovery | complete | Current no-S3 test cleared two actual map artifacts and restored one false-gated bridge after the fresh-session gate could not pass |
| Final safety snapshot | complete | One TCP-8765 owner; all four live gates false; no mapping container remains |

## 2026-09-02 mapping console missing-container recovery

The console's normal post-failure refresh must tolerate the mapping container
being absent after the worker restores the false-gated safe bridge. The change
is limited to `ROS2_WIN/docker/open_mapping_console.ps1`; mapping worker failure
logging and safe recovery remain unchanged.

| Phase | Status | Deliverable |
| --- | --- | --- |
| Root-cause reproduction | complete | Missing-container `docker inspect` reproduced as a terminating error under `ErrorActionPreference=Stop` |
| GUI probe hardening | complete | `Test-MappingRunning` treats absent/unavailable Docker state as `false` and restores the caller preference |
| Focused verification | complete | PowerShell parser and extracted probe check passed with the mapping container absent |

### Mapping console recovery errors

| Error | Attempt | Resolution |
| --- | --- | --- |
| GUI timer raised `no such object` after failed live gate | 1 | Scoped native-command error handling inside `Test-MappingRunning`; missing containers now report not running |
| Whole-file ASCII scan flagged `progress.md` | 1 | Existing historical non-ASCII bytes are outside this fix; touched PowerShell files and new text remain ASCII |
| First probe hardening still allowed a PowerShell native-command popup | 2 | Removed `docker inspect` from the GUI probe; use `docker ps` running-name filter plus timer/close-event guards |
| User still saw the same popup after the second fix | 3 | Found the desktop window was an old PID with pre-fix code already loaded; closed that stale console and removed the remaining worker-side `docker inspect` path |

## 2026-09-03 non-fatal mapping live gate

The default desktop mapping flow must retain the running visualization and ROS
topology when real odometry or TF is unavailable. It must never synthesize
sensor, odometry, TF, or map data. Strict fail-closed timeout cleanup remains
available only through an explicit `-StrictLiveGate` option.

| Phase | Status | Deliverable |
| --- | --- | --- |
| Entry-point and timeout-path audit | complete | Both desktop shortcuts target `open_mapping_console.ps1`; live timeout and cleanup path isolated in `mapping_session.ps1` |
| Default timeout behavior | complete | After 60 seconds, a warning leaves the mapping container, RViz, SLAM, bridge, and description running |
| Strict-mode retention | complete | `-StrictLiveGate` forwards from GUI to worker and retains original timeout cleanup |
| No-data runtime acceptance | complete | Default 60-second run kept the named container and all four expected ROS processes; `/odom` and TF remained absent, then explicit Stop restored safe mode |

### Non-fatal validation note

| Check | Result | Resolution |
| --- | --- | --- |
| Host `python -m py_compile` | Host Python cannot load `encodings` | Use the existing ROS container Python for read-only AST parsing; no source bytecode is written |
