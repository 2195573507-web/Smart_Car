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
| Build and verification | complete | Docker build succeeded; colcon build and all 25 tests passed |
| Final report | complete | Files, reuse boundaries, blockers, and handoff conditions recorded |

## 2026-08-29 Diagnostic extension

| Phase | Status | Deliverable |
| --- | --- | --- |
| Windows bridge startup and listener check | complete | Foreground TCP bridge on `0.0.0.0:8765`, Docker/Windows listener evidence, no source changes |

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
