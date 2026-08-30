# Smart_Car Project Status

Status date: 2026-08-30. This is a source/documentation snapshot, not a
release tag and not a hardware acceptance report.

## Version

| Item | Value | Evidence |
| --- | --- | --- |
| Knowledge base | `SRPv4 migration baseline 2026-08-30` | Source/build audit |
| Product/firmware release | Not formally tagged | Repository inspection |
| App package | SwiftPM macOS package | `IOS_APP/SmartCar_Control_MAC/Package.swift` |
| S3 firmware | ESP-IDF project | `ESPS3/CMakeLists.txt`, `ESPS3/main/main.c` |
| STM firmware | CM7/CM4 CMake project | `STM32H757/CM7`, `STM32H757/CM4` |

## Status Legend

- `完成`: source/documentation is present and its stated static boundary is
  identifiable; it does not imply hardware or end-to-end acceptance.
- `进行中`: an implementation boundary exists but a required path, contract,
  or verification layer is incomplete.
- `计划`: reserved future work with no current implementation proof.
- `暂停`: deliberately retained but not active in the current runtime path.
- `废弃`: retained only as historical documentation or an explicitly rejected
  design; it must not be used for current implementation behavior.

## Completed or Source-Established

| Area | Status | Current evidence | Acceptance limit |
| --- | --- | --- | --- |
| STM32 boot/log/RTOS scaffolding | 完成 | `main.c`, `boot_log.c`, `imu_runtime.c`, `rtos_health.c` | No current reset capture in this audit |
| LSM303 driver and manager path | 完成 | `lsm303.c`, `imu_manager.c` | I2C electrical behavior unverified |
| Calibration/filter/attitude layers | 完成 | `imu_calibration.c`, `imu_filter.c`, `attitude.c` | Full reset-to-ready capture unverified |
| STM32-S3 UART transport | 完成 | `uart_link.c`, `stm_uart.c`, USART2/UART2 constants | Physical link unverified |
| STM32-S3 frame codec/parser | 完成 | `Common/SRP` (`srp_codec`, `srp_link`) on both endpoints | Active UART2 contract is SRPv4; physical link unverified |
| S3 BLE GATT transport | 完成 | `s3_ble.c`, FFE0-FFE3 definitions | BLE session/notifications unverified |
| S3 radar raw transport/PWM | 完成 | `radar_uart.c`, `radar_parser.c`, `radar_control.c` | Radar protocol and motor behavior unverified |
| macOS control/developer UI | 完成 | SwiftUI views, `SmartCarViewModel` | App-to-device behavior unverified |
| standalone serial logger | 完成 | `Tools/SmartCar_Logger_MAC/Sources` | Live serial delivery unverified |

## In Progress / Incomplete Chains

| Area | Current gap | Evidence |
| --- | --- | --- |
| App -> S3 command admission | `s3_ble_set_rx_callback()` has no current caller; no BLE frame parser bridge is proven | S3 source search |
| S3 -> BLE telemetry relay | S3 internal service handles STM frames, but a current telemetry-to-BLE relay is not proven | `smartcar_service`, `imu_bridge.c` |
| IMU bridge | `imu_bridge_handle()` is an empty function | `ESPS3/components/smartcar_service/imu_bridge.c` |
| Calibration chain | STM/S3 radar calibration state machine exists, but full reset/ACK/event/hardware evidence is absent | source and history records |
| Protocol boundaries | STM32-S3 UART2 is unified on SRPv4; App BLE intentionally remains a separate envelope | `DOCS/protocol/protocol.md`, `DOCS/architecture/communication.md` |
| UART documentation | current source uses PA2/PA3 while IOC labels PD3/PD4 as legacy connector GPIO | IOC/MSP/header comparison |

## Paused

- BMI323 runtime use. The driver exists, but current manager startup logs
  `BMI323 SKIPPED`; LSM303 is the active path.
- Automatic control, ROS2 autonomy, SLAM, navigation, and mission behavior.
- Full four-wheel timer encoder mode: RF/RB pairs are available; LF/LB frozen
  nets do not satisfy TI1/TI2 requirements.

## Planned

- Explicit App/S3 command parser and relay contract.
- Explicit S3 telemetry/log relay and end-to-end frame tests.
- ROS2 gateway, LaserScan, SLAM, localization, navigation, and authority
  arbitration after L1 control is accepted.
- Hardware validation for UART, BLE, IMU buses, radar, PWM, encoders, and safe
  stop behavior.

## Deprecated / Abandoned

- A5/5A plus CRC-CCITT protocol planning, retained only in
  `docs/history/protocol/`.
- `Common/SCBP_CAN` executable sources and tests; removed from the active
  source/build graph. Remaining SCBP references are historical audit records.
- The former unified V1 and V2 protocol tables as cross-boundary authority;
  current source requires separate App BLE and STM-S3 frame pages.
- Any claim that the old PD3/PD4 connector GPIO labels are the current USART2
  transport route.

## Known Problems and Risks

1. Two frame envelopes are called AA55 in different documents and source
   modules; do not merge them by name alone.
2. Legacy IOC PD3/PD4 labels conflict with current generated USART2 PA2/PA3
   source and S3 GPIO17/18 route.
3. App BLE RX is transport-only in current S3 source; command admission is not
   demonstrated.
4. `imu_bridge_handle()` is empty, so telemetry forwarding cannot be claimed.
5. A successful host/firmware build cannot prove a physical sensor, UART, BLE,
   radar, motor, or vehicle result.
6. Root worktree contains unrelated dirty and untracked artifacts; preserve
   them and use target-file checks.

## TODO / Human Confirmation

- Confirm both flashed endpoints report SRPv4 sync and capture the live UART2
  frame/CRC exchange; source/build evidence alone is insufficient.
- Confirm the physical STM32 USART2 PA2/PA3 <-> S3 GPIO17/18 wiring and the
  legacy IOC labels.
- Confirm App command/telemetry acceptance criteria and stop semantics.
- Confirm BMI323 reactivation decision and required hardware evidence.
- Confirm ROS2 transport, DDS, time, odometry, and safety authority contracts.
