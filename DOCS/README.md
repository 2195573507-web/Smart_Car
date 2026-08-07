# Smart_Car Canonical Documentation

Use the root [README.md](../README.md) and [.codex/BOOT.md](../.codex/BOOT.md)
for startup. This directory is the canonical engineering documentation root;
on this macOS workspace it is also exposed as lowercase `docs/`.

## Sections

| Section | Contents |
| --- | --- |
| `architecture/` | System ownership, communication, and data flow |
| `hardware/` | Source/IOC-visible hardware facts and evidence limits |
| `stm32/` | STM32 boot, RTOS, and controller boundary |
| `esp32s3/` | S3 gateway and BLE boundary |
| `app/` | macOS control app |
| `protocol/` | Frame and UART contracts |
| `imu/` | IMU, calibration, filter, attitude modules |
| `radar/` | Radar receive/parser/PWM |
| `motor/` | Motor and encoder ownership |
| `debug/` | Logger and diagnostics |
| `ros2/` | Planned ROS2 boundary |
| `history/` | Superseded root-owned central records |

See [code_map.md](code_map.md), [DOCUMENT_INDEX.md](../DOCUMENT_INDEX.md), and
[DOCUMENT_AUDIT_REPORT.md](../DOCUMENT_AUDIT_REPORT.md) for navigation and
trust classification.
