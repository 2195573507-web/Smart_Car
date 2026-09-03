# SRP v4 Chassis Odometry Design

Status: approved for source implementation on 2026-08-31.

## Scope

This change restores the previously approved chassis-state behavior on the
current SRP v4 transport. CM7 owns odometry; ESP32-S3 validates and relays a
read-only state snapshot. Motion commands, safety admission, MotorBoard PID,
radar parsing, and S3RD raw-radar framing remain unchanged.

## Wire Contract

SRP priority is telemetry, message type is `0x15`, flags are stream-data, and
the payload is schema 1 with exactly 24 bytes:

| Offset | Field | Unit or rule |
| ---: | --- | --- |
| 0 | schema | `1` |
| 1 | flags | defined mask `0x0f` |
| 2 | reserved | little-endian zero |
| 4 | timestamp | CM7 monotonic ms |
| 8 | x | mm, finite float32 LE |
| 12 | y | mm, finite float32 LE |
| 16 | yaw | deg, finite float32 LE |
| 20 | total distance | m, finite nonnegative float32 LE |

`ODOMETRY_VALID` is set only when the MotorBoard feedback sample is fresh and
Primary DualAHRS yaw is fresh. Invalid snapshots still travel through S3 so a
host can stop publishing instead of retaining a stale valid state.

The fixed cross-end golden vector uses sequence `0x2a`, flags `0x04`,
timestamp `1000`, `(x,y,yaw,distance)=(1000,-500,179,12.5)`, and has SRP
CRC16-CCITT-FALSE `0xc07f` stored on wire as `7f c0`. This is the same vector
used by the Windows ROS2 consumer tests.

## CM7 Data Flow

The MotorBoard task remains the only MSPD owner. On each valid MSPD frame it
updates one critical-section-protected snapshot containing four corrected
speeds, CM7 arrival timestamp, source sequence, and validity.

A new low-priority chassis-state task starts after the attitude coordinator
has successfully started MotorBoard. Every 50 ms it reads the MotorBoard
snapshot and Primary DualAHRS heading. A pure odometry module averages the
four actual speeds, integrates `ds` using source timestamp delta, projects
with Primary yaw, and accumulates absolute distance. The first sample and the
first sample after an invalid gap only anchor time/yaw and add no distance.

MSPD older than 200 ms, repeated/regressed timestamps, non-finite data, or
invalid attitude clear `ODOMETRY_VALID` and reset only integration history;
the last pose is retained for diagnostics. Recovery cannot integrate the
missing interval.

The dormant `chassis_task` is not started by this work because doing so would
activate a separate motion-control path.

## ESP32-S3 Data Flow

The service relay adds `CHASSIS_STATE` to its existing telemetry sink. The
S3 queue re-decodes the complete SRP frame and validates type, priority,
stream flags, length, schema, flag mask, reserved bytes, finite values, and
nonnegative total distance.

Chassis state uses one latest-only queue slot and participates in the existing
bounded observation round-robin. It never enters the YDLIDAR decoder. The
existing generic S3RD telemetry envelope remains byte-for-byte unchanged.

## Resource and Concurrency Limits

- No allocation occurs on CM7 update or publish paths.
- The new CM7 task is low priority and uses bounded stack/local buffers.
- MotorBoard snapshot critical sections copy only fixed scalar data.
- S3 uses one static `radar_telemetry_entry_t` chassis slot.
- The service telemetry sink keeps its zero-wait mutex rule.

## Verification

Host tests cover odometry anchoring, timestamp wrap/gaps, stale invalidation,
yaw projection, reverse distance, payload layout, a fixed SRP golden frame,
S3 schema/flag/finite validation, latest-only overwrite, and observation
fairness. Target builds use the canonical CM7 `build/Debug` tree and ESP-IDF
5.5.4. Builds do not prove UART, TCP, ROS odometry, or vehicle behavior.
