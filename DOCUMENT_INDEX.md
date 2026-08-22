# Smart_Car Document Index

This page is the concise navigation index. The complete index of every
Markdown/README-like workspace document, including ownership, status, trust,
and handling guidance, is the [File List in the document audit](DOCUMENT_AUDIT_REPORT.md#file-list).

## Core Context

| Document | Purpose |
| --- | --- |
| [README.md](README.md) | Human entry point |
| [.codex/BOOT.md](.codex/BOOT.md) | Codex first-read file |
| [.codex/MEMORY.md](.codex/MEMORY.md) | Stable facts |
| [.codex/RULES.md](.codex/RULES.md) | Development rules |
| [.codex/WORKFLOW.md](.codex/WORKFLOW.md) | Analysis and verification flow |
| [.codex/INDEX.md](.codex/INDEX.md) | AI navigation |
| [PROJECT_STATUS.md](PROJECT_STATUS.md) | Current status, risks, TODO |
| [DECISION_LOG.md](DECISION_LOG.md) | Accepted design choices |

## Canonical Engineering Docs

| Area | Documents |
| --- | --- |
| Architecture | [system](docs/architecture/system.md), [communication](docs/architecture/communication.md), [data flow](docs/architecture/data_flow.md) |
| Hardware | [hardware facts](docs/hardware/facts.md) |
| STM32 | [STM32H757](docs/stm32/stm32h757.md), [boot](docs/stm32/boot.md), [FreeRTOS](docs/stm32/freertos.md) |
| ESP32-S3 | [S3](docs/esp32s3/esp32-s3.md), [BLE](docs/esp32s3/ble.md) |
| Protocol/UART | [protocol](docs/protocol/protocol.md), [STM-S3 command reference](docs/protocol/stm32-s3-command-reference.md), [App BLE](docs/protocol/app-ble-protocol-v1.md), [STM-S3 transport](docs/protocol/stm32-s3-transport.md), [UART](docs/protocol/uart.md) |
| IMU | [pipeline](docs/imu/imu-pipeline.md), [LSM303](docs/imu/lsm303.md), [BMI323](docs/imu/bmi323.md), [calibration](docs/imu/calibration.md), [attitude](docs/imu/attitude.md), [filter](docs/imu/filter.md) |
| Radar/motion | [radar](docs/radar/radar.md), [motor](docs/motor/motor.md), [encoder](docs/motor/encoder.md) |
| Motor board | [four-way motor-board guide](docs/motor/FOUR_WAY_MOTOR_BOARD_SMARTCAR_GUIDE.md) |
| App/tools | [macOS App](docs/app/mac-control-app.md), [logger](docs/debug/logger.md) |
| ROS2 | [ROS2](docs/ros2/ros2.md) |
| Mapping | [code map](docs/code_map.md) |

## History and Reference

- [History](docs/history/README.md) contains superseded root-owned central
  documents and compatibility notes.
- Module-local README files remain beside their source and are classified in
  [DOCUMENT_AUDIT_REPORT.md](DOCUMENT_AUDIT_REPORT.md).
- Nested repositories, extracted material, vendor manuals, and third-party
  SDK docs remain at their original paths and are reference-only.
