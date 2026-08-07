# ESP32-S3 Code Structure

## Scope

This document records the architecture-only scaffold under `ESPS3/`. It is a
file and ownership map, not a protocol or runtime implementation. The S3 is
the Smart_Car phone-facing gateway and STM32H757 bridge; STM32H757 remains the
final low-level motion authority.

The authoritative planning inputs are:

- [SMART_CAR_SYSTEM_ARCHITECTURE.md](SMART_CAR_SYSTEM_ARCHITECTURE.md)
- [SMART_CAR_PROTOCOL.md](SMART_CAR_PROTOCOL.md)
- [S3 architecture plan](ESP32/S3_ARCHITECTURE.md)
- [S3 performance plan](S3_PERFORMANCE_PLAN.md)
- [DEVELOPMENT_INDEX.md](DEVELOPMENT_INDEX.md)

## Directory Map

```text
ESPS3/
├── CMakeLists.txt                 # ESP-IDF project entry
├── sdkconfig.defaults             # ESP32-S3 target selection only
├── README.md                      # scaffold scope and limits
├── main/
│   ├── CMakeLists.txt
│   ├── README.md
│   ├── app_main.h
│   └── main.c                     # reserved app_main entry point
├── components/
│   ├── bsp/
│   │   ├── CMakeLists.txt
│   │   ├── bsp_gpio/              # board GPIO boundary
│   │   ├── bsp_uart/              # STM32 link UART boundary
│   │   ├── bsp_ble/               # phone BLE boundary
│   │   └── bsp_timer/             # timer/timeout boundary
│   ├── communication/
│   │   ├── CMakeLists.txt
│   │   ├── protocol/              # logical frame model boundary
│   │   ├── packet/                # bounded packet storage boundary
│   │   ├── crc/                   # integrity-check boundary
│   │   └── heartbeat/             # freshness/lease boundary
│   ├── app/
│   │   ├── CMakeLists.txt
│   │   ├── remote_control/        # phone control-session boundary
│   │   ├── command/               # normalized MOVE/STOP/MODE/PARAM intent
│   │   ├── state/                 # gateway and vehicle-state aggregation
│   │   └── safety/                # admission and safe-stop request boundary
│   └── system/
│       ├── CMakeLists.txt
│       ├── task_manager/          # lifecycle/task ownership boundary
│       ├── logger/                # structured diagnostics boundary
│       └── watchdog/              # task-health/recovery boundary
├── drivers/
│   └── radar/README.md            # reserved radar adapter; no driver yet
└── docs/README.md                 # project-local notes; DOCS/ is authoritative
```

Each implemented component has a matching source file, header, and `README.md`.
The current source/header names follow the requested BSP, application, protocol,
and system prefixes; packet, CRC, and heartbeat files retain their domain names
within `communication/`.

## Dependency Direction

The intended direction is from the entry point into application and system
coordination, then through communication and BSP boundaries:

```text
main/app_main
    -> app + system lifecycle
    -> communication models/adapters
    -> bsp hardware adapters
```

Transport/BSP modules must not send motor commands or mutate application state
directly. `app/safety` and the future session arbiter are the only S3-side
admission path toward the STM32 link. The future radar adapter is isolated
under `drivers/radar`; a future ROS2 gateway adapter must consume a versioned
S3 interface and cannot bypass the same admission boundary.

## Interface Reservations

The scaffold reserves, without implementing, the following surfaces:

| Boundary | Reserved direction | Current status |
| --- | --- | --- |
| iOS to S3 | BLE session and `MOVE`, `STOP`, `MODE`, `PARAM` intent | headers/placeholders only |
| S3 to iOS | connection, battery, and robot-state reporting | aggregation boundary only |
| S3 to STM32H757 | UART `MOTOR_COMMAND` and `CONTROL_MODE` intent | physical route and protocol pending |
| STM32H757 to S3 | odometry, IMU, and motor-state reporting | receive boundary pending |
| radar to S3 | future bounded radar ingress | README reservation only |
| S3 to ROS2_WIN | future gateway/ROS2 adapter | no component created in this stage |

The packet fields, CRC, sequencing, heartbeat, and fault semantics remain
owned by `SMART_CAR_PROTOCOL.md`; this scaffold does not duplicate or freeze
wire-level constants.

## Evidence Limits

This map proves only that the directories, CMake registration files, headers,
sources, and module READMEs exist in the workspace. It does not prove ESP-IDF
configuration, a successful build, BLE pairing or latency, UART electrical
compatibility or traffic, STM32 behavior, motor control, radar ingestion,
ROS2 connectivity, or vehicle safety. Any future build, bench, hardware, and
integration evidence must be recorded separately from this static map.
