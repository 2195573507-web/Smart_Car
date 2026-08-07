# ESP32-S3 Development Log

## 2026-07-30: Architecture Scaffold

### Scope

Initialized the ESP32-S3 high-level gateway software tree requested by the
Smart_Car architecture. This stage is limited to directory governance,
component boundaries, C/H templates, README files, and ESP-IDF project
registration. It does not implement BLE, UART, the packet protocol, STM32
control, motor behavior, radar parsing, ROS2, SLAM, or navigation.

### Delivered Tree

- `ESPS3/` project entry (`CMakeLists.txt`, `sdkconfig.defaults`, `README.md`).
- `main/` with a reserved `app_main()` entry point.
- `components/bsp/`: `bsp_gpio`, `bsp_uart`, `bsp_ble`, and `bsp_timer`.
- `components/communication/`: `protocol`, `packet`, `crc`, and `heartbeat`.
- `components/app/`: `remote_control`, `command`, `state`, and `safety`.
- `components/system/`: `task_manager`, `logger`, and `watchdog`.
- `drivers/radar/README.md` as a future radar boundary only.
- `ESPS3/docs/README.md` pointing back to repository-level `DOCS/` authority.

Each current C implementation is an initialization/deinitialization placeholder
returning `ESP_ERR_NOT_SUPPORTED`; no transport or business behavior is hidden
behind the scaffold.

### Static Checks and Build Evidence

The workspace checks found 15 placeholder module directories, 15 matching
module C/H pairs, 15 module README files, and six CMake registration files.
The repository root has no Git metadata, so a repository `git diff` was not
available; changed-file review uses explicit path listings. An isolated
ESP-IDF 5.5.4 build for target `esp32s3` passed at
`/tmp/smart-car-esps3-final-20260730`, producing `esps3_gateway.bin` with 82% of the
smallest app partition free. This is build evidence only, not hardware,
runtime, transport, or vehicle acceptance.

### Boundary Review Notes

- S3 owns iOS/BLE session-facing gateway work, command admission, state
  aggregation, and the planned STM32 bridge.
- STM32H757 retains final actuator authority, low-level timing, and local
  interlocks. The documented `PD3`/`PD4` net mismatch remains an unresolved
  UART prerequisite; no pin or STM32 file was changed.
- Radar and ROS2 are reserved as isolated future adapters. Their drivers,
  parsers, transports, and autonomy behavior are not present.

### Next Authorized Steps

1. Freeze the physical STM32-S3 UART route and approved protocol/security
   contract before implementing transport code.
2. Configure and build the minimal ESP-IDF project in a target environment;
   add host/parser tests without connecting an actuator.
3. Implement BLE session admission and UART bridge behavior with bounded
   queues, heartbeat/timeout handling, and explicit fault reporting.
4. Validate staged bench behavior before any radar or ROS2 integration.

All items above are proposals for later authorized work. This log is a static
development record and does not claim runtime, hardware, or end-to-end safety
evidence.
