# Smart_Car Development Index

## Current Stage

The project is in infrastructure initialization. The authorized deliverables are directory governance, documentation, STM32H757 base project structure, and a CubeMX IOC baseline. No vehicle-control, sensing, SLAM, navigation, app, or ESP32 feature implementation is included.

## Authoritative Documents

| Area | Document | Purpose |
| --- | --- | --- |
| System | [PROJECT_ARCHITECTURE.md](PROJECT_ARCHITECTURE.md) | System chain, component ownership, and evidence limits |
| Governance | [CODEX_WORKFLOW.md](CODEX_WORKFLOW.md) | Required task sequence and verification rules |
| Governance | [MULTI_AGENT_RULES.md](MULTI_AGENT_RULES.md) | Three-agent responsibilities and separation of duties |
| STM32H757 | [STM32_ARCHITECTURE.md](STM32H757/STM32_ARCHITECTURE.md) | Base-project structure and dual-core baseline |
| STM32H757 | [STM32_PIN_MAP.md](STM32H757/STM32_PIN_MAP.md) | Hardware net to MCU pin record |
| STM32H757 | [MOTOR_INTERFACE_MAP.md](STM32H757/MOTOR_INTERFACE_MAP.md) | Frozen motor and encoder net allocation, TIM candidates, and constraints |
| STM32H757 | [STM32_BUILD.md](STM32H757/STM32_BUILD.md) | IOC, generation, and build validation procedure |
| STM32H757 | [ARCHITECTURE.md](../STM32H757/Docs/ARCHITECTURE.md) | STM32H757 ownership and dependency direction |
| STM32H757 | [CODE_STRUCTURE.md](../STM32H757/Docs/CODE_STRUCTURE.md) | Scaffold paths and naming conventions |
| STM32H757 | [S3_COMMUNICATION.md](../STM32H757/Docs/S3_COMMUNICATION.md) | STM32 to ESP32-S3 command/state boundary |
| STM32H757 | [MODULE_STATUS.md](../STM32H757/Docs/MODULE_STATUS.md) | Current scaffold status and evidence limits |
| STM32H757 | [ARCHITECTURE_AUDIT.md](../STM32H757/Docs/ARCHITECTURE_AUDIT.md) | Independent scaffold layering audit |
| ESP32-S3 | [ESP32_ROLE.md](ESP32/ESP32_ROLE.md) | Gateway responsibility boundary |
| ROS2_WIN | [ROS2_ARCHITECTURE.md](ROS2/ROS2_ARCHITECTURE.md) | ROS2 responsibility boundary |
| ROS2_WIN | [ROS2_ARCHITECTURE_AUDIT.md](ROS2_WIN_Radar/ROS2_ARCHITECTURE_AUDIT.md) | Independent Windows Docker / DDS / device-transfer audit |
| ROS2_WIN | [RADAR_SOURCE_ANALYSIS.md](ROS2_WIN_Radar/RADAR_SOURCE_ANALYSIS.md) | Read-only local radar-material and source analysis evidence |
| ROS2_WIN | [RADAR_LEARNING_GUIDE.md](ROS2_WIN_Radar/RADAR_LEARNING_GUIDE.md) | YDLIDAR and LaserScan learning path |
| ROS2_WIN | [RADAR_DRIVER_PORTING.md](ROS2_WIN_Radar/RADAR_DRIVER_PORTING.md) | ROS1-to-ROS2 driver porting preparation |
| ROS2_WIN | [YDLIDAR_X3_ROS2_PORTING.md](ROS2_WIN_Radar/YDLIDAR_X3_ROS2_PORTING.md) | X3/X3 Pro SDK, driver, and integration preparation |
| ROS2_WIN | [DOCKER_ROS2_SETUP.md](ROS2_WIN_Radar/DOCKER_ROS2_SETUP.md) | Windows Docker Desktop ROS2 deployment preparation |
| IOS_APP | [IOS_ARCHITECTURE.md](IOS_APP/IOS_ARCHITECTURE.md) | Mobile responsibility boundary |
| System | [SMART_CAR_SYSTEM_ARCHITECTURE.md](SMART_CAR_SYSTEM_ARCHITECTURE.md) | IOS_APP/S3 L1-L3 system ownership and data flows |
| Protocol | [SMART_CAR_PROTOCOL.md](SMART_CAR_PROTOCOL.md) | APP-S3 and S3-STM32 versioned frame target |
| Protocol | [app-ble-protocol-v2.md](protocol/app-ble-protocol-v2.md) | Canonical App-BLE V1/V2 GATT, session, ACK, and safety boundary |
| Protocol | [SRP_v4_Spec.md](SRP_v4_Spec.md) | Active STM32-S3 SRP v4 wire contract |
| Protocol | [UART_Config_Guide.md](UART_Config_Guide.md) | UART2 DMA/IDLE, queue isolation and baud changes |
| Protocol | [Integration_Manual.md](Integration_Manual.md) | SRP message extension and verification procedure |
| IOS_APP | [IOS_APP_ARCHITECTURE.md](IOS_APP/IOS_APP_ARCHITECTURE.md) | SwiftUI, BLE/Wi-Fi, control modes, and L3 boundary |
| IOS_APP | [IOS_CODE_STRUCTURE.md](IOS_CODE_STRUCTURE.md) | Implemented Swift Package source tree and dependency boundaries |
| IOS_APP | [IOS_DEVELOPMENT_LOG.md](IOS_DEVELOPMENT_LOG.md) | L1 scaffold delivery record and verification limits |
| Tools | [SmartCar_Logger_MAC README](../Tools/SmartCar_Logger_MAC/README.md) | Standalone macOS SwiftUI STM32 USART1 serial-log viewer scope and usage |
| ESP32-S3 | [S3_ARCHITECTURE.md](ESP32/S3_ARCHITECTURE.md) | ESP-IDF, FreeRTOS, dual-core, gateway and radar plan |
| ESP32-S3 | [S3_CODE_STRUCTURE.md](S3_CODE_STRUCTURE.md) | ESP32-S3 scaffold tree, naming, and dependency direction |
| ESP32-S3 | [S3_DEVELOPMENT_LOG.md](S3_DEVELOPMENT_LOG.md) | S3 scaffold delivery record and evidence limits |
| ESP32-S3 | [S3_ARCHITECTURE_AUDIT.md](S3_ARCHITECTURE_AUDIT.md) | Independent S3 boundary and extension audit |
| Safety | [SAFETY_REVIEW_IOS_S3.md](SAFETY_REVIEW_IOS_S3.md) | Independent safety risks and required controls |
| Performance | [S3_PERFORMANCE_PLAN.md](S3_PERFORMANCE_PLAN.md) | S3 resource and latency measurement plan |
| Testing | [SMART_CAR_TEST_PLAN.md](SMART_CAR_TEST_PLAN.md) | Layered host, bench, and controlled vehicle tests |

## Recommended Reading Order

1. Read [PROJECT_ARCHITECTURE.md](PROJECT_ARCHITECTURE.md) for ownership and system boundaries.
2. Read [CODEX_WORKFLOW.md](CODEX_WORKFLOW.md) and [MULTI_AGENT_RULES.md](MULTI_AGENT_RULES.md) before any change.
3. For STM32H757 work, read the four STM32H757 documents together before opening CubeMX or source files.
4. For a later subsystem task, read the subsystem document and define an explicit interface contract before implementation.

## ROS2_WIN Development Record

### 2026-07-30: STM32H757 software architecture scaffold

- Added the STM32H757 BSP, project-driver, Middleware, Application, System,
  and Config directory boundaries with paired C/header templates where
  requested.
- Added the S3 command, packet, protocol, and state interface boundary without
  implementing transport or protocol behavior.
- Added the STM32H757 architecture, code structure, communication boundary,
  module status, development log, and independent audit documents.
- Preserved CubeMX/HAL/Core/IOC/CMake files and kept external sensing, Wi-Fi,
  phone App, and ROS2 gateway ownership in ESP32-S3.
- Static C syntax and ASCII checks passed; clean CM7 and CM4 baseline builds
  passed. Hardware, sensors, motors, serial link, and runtime behavior remain
  unverified.

### 2026-07-30: STM32H757 motor and encoder resource allocation

- Added the frozen extension-board motor and encoder interface map. It records
  TIM3 CH1-CH4 as the four connected PWM candidates and keeps each companion
  direction line as a GPIO output.
- Static alternate-function review supports RF on TIM1 CH1/CH2 and RB on TIM2
  CH1/CH2. It records LF and LB as unsupported for TIM Encoder Mode: LF uses
  PA13, which must remain SWDIO and has no timer alternate function; LB is
  connected only to TIM4 CH3/CH4, while Encoder Mode requires CH1/CH2.
- This is a documentation/static-review result. No PCB net, IOC, source,
  generated project, firmware, or hardware behavior was changed or verified by
  this documentation entry.

### 2026-07-29: Radar learning and Docker preparation

- Created a read-only learning and porting document set for the local YDLIDAR X3/X3 Pro materials, ROS2 driver/SDK archive structure, ROS `LaserScan` flow, ROS2 lifecycle concepts, and Docker deployment preparation.
- Recorded the current ownership boundary: the project baseline assigns lidar acquisition/parsing to ESP32-S3 and ROS2_WIN receives gateway-provided information through Wi-Fi. Windows Docker direct-driver work is a separate, not-yet-authorized route.
- Recorded Windows Docker Desktop risks: `COMx` is not automatically a container `/dev/tty*` device; WSL2 USB transfer, device permissions/replug, DDS discovery, TF, odometry, and vehicle safety interfaces each require future validation.
- Evidence in this entry is static documentation and archive-listing review only. Docker, ROS2, SDK/driver, SLAM Toolbox, Nav2, hardware, USB/UART, network, map, navigation, and vehicle-control behavior remain unverified.

## Standalone Tools Record

### 2026-08-02: SmartCar Logger Mac

- Added the independent [`Tools/SmartCar_Logger_MAC`](../Tools/SmartCar_Logger_MAC/README.md)
  macOS SwiftUI serial-log viewer for STM32 USART1 over a CH340 USB-UART adapter.
- The tool boundary is receive-only logging at 115200 8N1, with automatic serial-device
  discovery, live text output, and log-file saving. It does not provide vehicle control
  and does not change the STM32H757, ESPS3, or IOS_APP projects.
- Tool build and connected-device behavior must be reported in the tool README; this index
  entry does not promote host compilation or static checks to hardware/runtime acceptance.

## Source-of-Truth Rules

### 2026-07-30: iOS L1 architecture scaffold (historical)

- Initialized the historical `IOS_APP/` Swift Package source tree with
  SwiftUI/MVVM surfaces, a CoreBluetooth transport adapter, protocol packet
  models, and vehicle status state. That tree was removed on 2026-08-23.
- The current iOS implementation lives under `IOS-APP/`; its command, mode,
  telemetry, and build evidence is recorded in `.planning/ios-app/`.
- The implementation remains a software scaffold. BLE UUIDs, S3 admission,
  telemetry decoding, hardware stop behavior, and vehicle movement are not
  verified. Automatic navigation, SLAM, radar display, ROS2, and AI-agent
  control remain deferred.
- See [IOS_CODE_STRUCTURE.md](IOS_CODE_STRUCTURE.md) for the file-level map and
  [IOS_DEVELOPMENT_LOG.md](IOS_DEVELOPMENT_LOG.md) for implementation evidence
  and next steps.

## IOS_APP and ESP32-S3 Planning Record

### 2026-07-30: ESP32-S3 architecture scaffold

- Added the `ESPS3/` ESP-IDF project skeleton with `bsp`, `communication`,
  `app`, `system`, and reserved `drivers/radar` boundaries.
- Added C/H/README placeholders for BLE, UART, protocol, packet, CRC,
  heartbeat, control-session, state, safety, task, logger, and watchdog seams.
- Added [S3_CODE_STRUCTURE.md](S3_CODE_STRUCTURE.md),
  [S3_DEVELOPMENT_LOG.md](S3_DEVELOPMENT_LOG.md), and the independent
  [S3_ARCHITECTURE_AUDIT.md](S3_ARCHITECTURE_AUDIT.md).
- An isolated ESP-IDF 5.5.4 `esp32s3` build passed. This does not prove BLE,
  UART electrical compatibility, STM32 behavior, radar, ROS2, or vehicle
  safety; the documented `PD3`/`PD4` UART route remains unresolved.

### 2026-07-30: L1 remote-control and L3 expansion architecture

- Added system, iOS, ESP32-S3, protocol, safety, performance, and test
  planning documents under the established `DOCS/` hierarchy.
- The architecture keeps STM32H757 as final low-level motion authority, S3 as
  phone/gateway/session boundary, IOS_APP as operator UX, and ROS2_WIN as the
  future autonomy/map domain.
- L1 reserves direction-pad and virtual-joystick control. Automatic mode,
  map, SLAM, and navigation are future capability-gated extensions.
- The protocol and safety documents preserve the existing `PD3`/`PD4` UART
  hardware mismatch as an unresolved prerequisite. No code, project directory,
  STM32H757 file, or ROS2_WIN file was modified.
- Static documentation checks are the only evidence in this record. BLE,
  Wi-Fi, UART, motor, radar, ROS2, SLAM, navigation, and vehicle safety remain
  unverified.

### Directory mapping note

The repository documentation authority remains `DOCS/` (the case-insensitive
workspace also exposes it as `docs/`). The ESP32-S3 scaffold now exists under
`ESPS3/`, with project-local notes in `ESPS3/docs/`; the architecture and
development records remain at repository-level `DOCS/`. `IOS_APP/` is an
independent existing application tree. No STM32H757 or iOS source boundary was
changed by the S3 scaffold.

- The IOC is the source of truth for CubeMX pin/peripheral selection after it is generated and parsed successfully.
- `STM32_PIN_MAP.md` is the human-readable required-net record. It must be updated with the IOC only when the two agree.
- Build output is the source of truth for compilation status; it is not hardware acceptance.
- Hardware images, schematics, and on-board observation are required to establish physical wiring and runtime behavior.
