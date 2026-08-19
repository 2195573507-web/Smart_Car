# SCBP-V3 Migration Report

Status: source and build verified. Hardware behavior is UNVERIFIED.

## Modified Files

| Area | Files |
| --- | --- |
| STM32 codec | `Middleware/Communication/SmartCar_Frame/sc_frame.h`, `sc_frame.c` |
| STM32 service | `Middleware/Communication/Services/s3_service.h`, `s3_service.c` |
| S3 codec | `components/smartcar_protocol/include/frame.h`, `frame.c`, `include/parser.h`, `parser.c` |
| S3 service | `components/smartcar_service/command_bridge.c`, `log_bridge.c` |
| Documentation | `DOCS/protocol/SCBP_V3_REFERENCE.md`, `protocol.md`, `stm32-s3-transport.md`, this report |

## Frame Comparison

| Item | Previous STM-S3 frame | SCBP-V3 |
| --- | --- | --- |
| Header | `AA 55 01 TYPE LEN` | `AA 55 VER PRIORITY SRC DST MSG_ID SEQ FLAGS LEN` |
| Message key | uint8 TYPE | uint16 MSG_ID LE |
| Addressing | Implicit point-to-point | Explicit SRC and DST |
| Ordering | None | Per-source uint8 SEQ diagnostics |
| ACK | Message-specific type | `0x0005 ACK` with MSG_ID and SEQ correlation |
| Error | Parser callback only | Parser callback plus `0x0006 ERROR` for valid dispatch errors |
| Frame overhead | 8 bytes | 14 bytes |
| CRC coverage | Version through payload | Version through payload |

## Legacy Message Migration

| Legacy Type | SCBP-V3 MSG_ID | Notes |
| ---: | ---: | --- |
| `0x01` PING | `0x0001` | Normal |
| `0x02` PONG | `0x0002` | Normal |
| `0x03` ACK | `0x0005` | Unified ACK |
| `0x10` PWM_READY | `0x0302` | Legacy adapter only |
| `0x13` IMU_CAL_BIAS | `0x0203` | Payload unchanged |
| `0x16` RADAR_PWM_READY | `0x0302` | S3 to STM32; payload speed_percent |
| `0x17` RADAR_PWM_ACK | `0x0005` | ACKs `0x0302` |
| `0x18` CAL_EVENT | `0x0401` | Payload unchanged |
| `0x19` CAL_EVENT_ACK | `0x0005` | ACKs `0x0401` |
| `0x1C` STM_BOOT_READY | `0x0007` | STM32 to S3; state/result payload |
| `0x20` IMU_STATUS | `0x0200` | Payload unchanged |
| `0x21` ATTITUDE | `0x0201` | V3 payload is 30 bytes |
| `0x22` IMU_CAL_STATUS | `0x0202` | Payload unchanged |
| `0x23` RADAR_STATUS | `0x0301` | Payload unchanged |
| `0x24` VIBRATION | `0x0204` | Payload unchanged |
| `0x30` LOG | `0xF000` | Payload unchanged |

`PWM_SET=0x0101` remains the explicit active-PWM command ID and is not used to
represent radar readiness. `CAL_START=0x0400` remains reserved; BOOT_READY has
the independently frozen ID `0x0007`.

## Business Behavior Boundary

The protocol layer preserves calls into the existing calibration manager and
STM IMU boot manager. It does not modify their state-machine source, radar
control source, UART driver source, BLE driver source, RTOS task source,
sensor/attitude algorithm source, or App source. Protocol adapters retain the
old local callback arguments while serializing V3 messages and generic ACKs.

ATTITUDE is intentionally exceptional at the protocol boundary: STM serializes
the required V3 30-byte payload and S3 forwards that payload unchanged in the
BLE envelope. No S3 ATTITUDE translation was added.

## Validation

| Check | Result |
| --- | --- |
| `git diff --check` | Passed |
| STM32H757 CM7 `cmake --preset Debug` and clean build | Passed |
| ESP32-S3 ESP-IDF 5.5.4 `idf.py -B build-scbp-v3 fullclean`, then `build` | Passed |
| Flash, UART capture, BLE capture, sensor/radar/device behavior | Not run by task constraint |

The green builds prove both firmware source trees compile with the V3 protocol
interfaces. They do not prove physical UART directionality, BLE delivery,
calibration progression, radar PWM, sensor data, or vehicle safety.
