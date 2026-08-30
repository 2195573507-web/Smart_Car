# Smart_Car Codex Index

## Startup

| Need | Read |
| --- | --- |
| First orientation | [BOOT.md](BOOT.md) |
| Stable facts | [MEMORY.md](MEMORY.md) |
| Rules | [RULES.md](RULES.md) |
| Current risks/status | [../PROJECT_STATUS.md](../PROJECT_STATUS.md) |
| All module links | [MODULE_INDEX.md](../MODULE_INDEX.md) |
| All document links | [DOCUMENT_INDEX.md](../DOCUMENT_INDEX.md) |

## Module Routing

| Module | Canonical document | Source entry | Key interface |
| --- | --- | --- | --- |
| STM32H757 | [docs/stm32/stm32h757.md](../docs/stm32/stm32h757.md) | `STM32H757/CM7/Core/Src/main.c` | `imu_*`, `uart_link_*`, `s3_service_*` |
| ESP32-S3 | [docs/esp32s3/esp32-s3.md](../docs/esp32s3/esp32-s3.md) | `ESPS3/main/main.c` | `stm_uart_*`, `s3_ble_*` |
| BLE | [docs/esp32s3/ble.md](../docs/esp32s3/ble.md) | `ESPS3/components/s3_ble/s3_ble.c` | `s3_ble_notify_send`, `BLEManager` |
| UART | [docs/protocol/uart.md](../docs/protocol/uart.md) | `STM32H757/.../uart_link.c`, `ESPS3/components/stm_uart/stm_uart.c` | `send/read/get_stats` |
| Protocol | [docs/protocol/protocol.md](../docs/protocol/protocol.md) | `Common/SRP`, `SmartCarProtocol.swift` | `srp_encode/decode`, App BLE parser |
| IMU | [docs/imu/imu-pipeline.md](../docs/imu/imu-pipeline.md) | `STM32H757/Middleware/Sensor/imu_manager.c` | `imu_init/update/get_data` |
| Radar | [docs/radar/radar.md](../docs/radar/radar.md) | `ESPS3/main/radar/radar_uart.c` | `radar_uart_init`, `radar_parser_feed` |
| App | [docs/app/mac-control-app.md](../docs/app/mac-control-app.md) | `IOS_APP/SmartCar_Control_MAC/Sources/.../BLEManager.swift` | `BLEManager`, `TelemetryStore` |
| ROS2 | [docs/ros2/ros2.md](../docs/ros2/ros2.md) | `ROS2_WIN/` and `DOCS/ROS2*` | Future gateway/ROS interfaces |

## Before Editing

Use the canonical page, then [docs/code_map.md](../docs/code_map.md) to locate
the exact function/task/interface. Check [DECISION_LOG.md](../DECISION_LOG.md)
for already-set boundaries and [PROJECT_STATUS.md](../PROJECT_STATUS.md) for
unverified or paused paths.
