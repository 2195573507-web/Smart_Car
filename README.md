# Smart_Car

Smart_Car is a staged robotics platform. The current implementation boundary
is a deterministic STM32H757 controller connected to an ESP32-S3 gateway and an
iOS SwiftUI control application. Radar acquisition is assigned to the S3;
ROS2, SLAM, navigation, and autonomous mission behavior are future work.

## Current Stage

The 2026-08-07 context baseline is an integration and documentation stage, not
a released vehicle build. Firmware, gateway, BLE, UART, radar, and app source
exists in the workspace, but the full physical chain is not accepted. The
current state and risks are in [PROJECT_STATUS.md](PROJECT_STATUS.md).

## Read This First

1. [.codex/BOOT.md](.codex/BOOT.md)
2. [.codex/MEMORY.md](.codex/MEMORY.md)
3. [.codex/RULES.md](.codex/RULES.md)
4. [PROJECT_STATUS.md](PROJECT_STATUS.md)
5. [docs/architecture/system.md](docs/architecture/system.md)

## Workspace Map

| Area | Path | Role |
| --- | --- | --- |
| STM32 controller | `STM32H757/` | Sensor, calibration, attitude, safety, actuator authority |
| ESP32-S3 gateway | `ESPS3/` | STM UART, BLE GATT, radar UART/PWM, gateway services |
| iOS control app | `IOS-APP/` | SwiftUI operator and telemetry UI |
| serial logger | `Tools/SmartCar_Logger_MAC/` | Independent STM32 USART1 log viewer |
| ROS2 material | `ROS2_WIN/`, `DOCS/ROS2*` | Planned host-side radar/SLAM/navigation work |
| canonical context | `.codex/`, `DOCS/` (`docs/`) | AI navigation and engineering records |

## Evidence Rule

Source inspection, Markdown checks, host builds, firmware builds, device logs,
and hardware/integration behavior are separate evidence levels. A green build
does not prove UART wiring, BLE delivery, sensor identity, motor response,
radar rotation, or vehicle safety.

## STM32 Build Output

For every CM7 firmware configuration, use only
`STM32H757/CM7/build/Debug`. The STM32CubeProgrammer input is always
`STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf`; do not create or use
task-specific STM32 build directories.

## Protected Boundaries

Do not change GPIO, PCB assumptions, CubeMX/IOC, framing contracts, safety
ownership, or cross-module behavior without an explicit task. Read the context
files before editing and keep changes inside the named module.

## Documentation Navigation

- [Document index](DOCUMENT_INDEX.md)
- [Module index](MODULE_INDEX.md)
- [Code map](docs/code_map.md)
- [Communication architecture](docs/architecture/communication.md)
- [Protocol boundary](docs/protocol/protocol.md)
- [Decision log](DECISION_LOG.md)
- [Document audit](DOCUMENT_AUDIT_REPORT.md)
