# STM32-S3 SCBP-CAN Command Reference (Deprecated)

> Deprecated historical record. The active UART contract is SRP v4. Read
> [`../SRP_v4_Spec.md`](../SRP_v4_Spec.md) and
> [`../Integration_Manual.md`](../Integration_Manual.md). Do not use the
> SCBP-CAN IDs or build commands below for new code.

Status: `CONFIRMED` for the source contract inspected on 2026-08-21.

This is the implementation reference for the binary UART2 link between the
STM32H757 CM7 controller and the ESP32-S3 gateway. When adding a command,
start here, update the shared definitions and both endpoint dispatch paths,
then update this registry in the same change.

This document covers **SCBP-CAN only**. The App BLE envelope and the separate
SmartCar log envelope are not interchangeable with an SCBP-CAN frame.

## Source Of Truth

Use this precedence when documents disagree:

1. `Common/SCBP_CAN/include/scbp_protocol_defs.h` and the shared codec/link implementation.
2. Active STM32 and S3 call sites listed in this document.
3. The canonical overview at [`DOCS/protocol/protocol.md`](protocol.md).
4. Historical V1/V2/V3 documents, which are context only and are not active wire contracts.

Active source modules:

| Concern | STM32H757 | ESP32-S3 |
| --- | --- | --- |
| Shared codec/parser/link | `Common/SCBP_CAN/` | Same module via `components/smartcar_protocol/CMakeLists.txt` |
| UART transport | `Middleware/Communication/UART_Link/` | `components/stm_uart/` |
| SCBP dispatch | `Middleware/Communication/Services/s3_service.c` | `components/smartcar_service/command_bridge.c` |
| Calibration | `Middleware/Calibration/imu_boot_manager.c` | `components/smartcar_service/radar_calibration_manager.c` |
| Telemetry | `Application/RTOS/imu_runtime.c` | `components/smartcar_service/command_bridge.c` |

## Physical Link

| Property | Contract |
| --- | --- |
| STM endpoint | USART2, PA2/PA3 |
| S3 endpoint | UART2, GPIO17/GPIO18 |
| Direction | Full duplex; STM TX -> S3 RX and S3 TX -> STM RX |
| Baud / format | `921600`, `8N1` |
| Flow control | Disabled |
| STM RX | 512-byte DMA buffer in `.dma_buffer`, IDLE/full events, 2048-byte software ring, D-cache maintenance and recovery |
| S3 RX/TX | ESP-IDF driver, 4096-byte driver buffers, 4096-byte newest-data storage, TX mutex |

The mapping is source-confirmed. UART signal integrity, target DMA behavior,
and end-to-end delivery are `UNVERIFIED` until matching images are flashed
and captured.

## Frame Format

Every UART byte stream is parsed as a sequence of these frames. A UART read
does not define a packet boundary.

```text
5A A5 | CAN_ID_LO CAN_ID_HI | FLAGS | LEN | HCS | SEQ |
       PAYLOAD[LEN] | FCS_LO FCS_HI | 0D 0A
```

| Offset | Size | Field | Encoding and rule |
| ---: | ---: | --- | --- |
| 0 | 1 | `SOF0` | `0x5A` |
| 1 | 1 | `SOF1` | `0xA5` |
| 2 | 2 | `CAN_ID` | u16 little-endian packed ID |
| 4 | 1 | `FLAGS` | Bits 0..3 defined; bits 4..7 reserved and rejected |
| 5 | 1 | `LEN` | Payload byte count, 0..255 |
| 6 | 1 | `HCS` | CRC-8-ITU over bytes 2..5 |
| 7 | 1 | `SEQ` | Sender sequence, modulo 256 |
| 8 | `LEN` | `PAYLOAD` | Command bytes; multi-byte fields are little-endian |
| 8+LEN | 2 | `FCS` | CRC16-MODBUS over payload only, little-endian |
| 10+LEN | 2 | `EOF` | `0x0D 0x0A` |

Header size is 8 bytes, trailer size is 4 bytes, and maximum frame size is
267 bytes (`8 + 255 + 4`). Payload starts at offset 8. Packed structures must
not be read by unaligned casts; use explicit byte-wise or `memcpy` access.

### CAN ID

`CAN_ID` is a packed 16-bit identifier, not a classical CAN controller ID on
this UART link.

| Bits | Field | Values |
| ---: | --- | --- |
| 15..14 | Priority | `0` emergency, `1` realtime, `2` normal, `3` debug |
| 13..12 | Source | `1` STM32H757, `2` ESP32-S3, `3` reserved/broadcast identity |
| 11..10 | Destination | `1` STM32H757, `2` ESP32-S3, `3` broadcast |
| 9..0 | Message | `0x000..0x3FF` |

Only nodes 1 and 2 are valid local transmitters. Destination 3 is broadcast.
The App does not address this UART link; `SCBP_NODE_APP=3` is an alias, not a
third UART endpoint.

### Flags

| Bit | Macro | Meaning |
| ---: | --- | --- |
| 0 | `ACK_REQUIRED` | Transaction; retain frame for timeout/retry |
| 1 | `IS_ACK` | Successful fast response; message ID `0x005` |
| 2 | `IS_ERROR` | Failed fast response; message ID `0x006` |
| 3 | `STREAM_DATA` | Non-transactional telemetry/status/log stream |
| 4..7 | Reserved | Must be zero |

Do not combine `IS_ACK`/`IS_ERROR` with `ACK_REQUIRED`. A broadcast request is
never answered by either endpoint.

## Reliability And Recovery

| Rule | Value/behavior |
| --- | --- |
| Pending transactions | 4 slots per endpoint |
| ACK timeout | 500 ms |
| Retransmissions | At most 3 after the initial transmission, with identical CAN_ID, SEQ, payload, and encoded frame |
| Correlation | Exact `(ack_can_id, ack_seq)` pair |
| Fast response payload | `ack_can_id_u16_le | ack_seq_u8 | status_code_u8` |
| Status codes | `0x00` OK, `0x01` HCS, `0x02` FCS, `0x03` busy, `0x04` timeout, `0x05` invalid parameter |
| Parser recovery | HCS failure replays possible header suffix; FCS/EOF failure resumes SOF search |
| Error counters | HCS/FCS add 8 to REC; valid receive decrements REC; transport failure/exhausted transaction add 8 to TEC; successful ACK decrements TEC |
| Link states | `ACTIVE < 32`, `WARNING >= 32`, `PASSIVE >= 128`, `BUS_OFF >= 256` using max(REC, TEC) |
| Bus-off | Flush/rebuild UART RX, wait 100 ms, reset counters/state, release pending callbacks |

Application handlers must be idempotent for retries. Validate source,
destination, message ID, flags, exact length, and semantic values before
admission. Unsupported transactions return `ERROR(0x006, INVALID_PARAM)`;
temporarily inadmissible ones return `ERROR(..., BUSY)`.

## Active Command Registry

`CONFIRMED` means the constants and relevant source path exist. `UNVERIFIED`
means no physical or end-to-end runtime capture has been performed.

| Message | ID | Direction | Flags | Payload | Current handling |
| --- | ---: | --- | --- | --- | --- |
| `CAL_EVENT` | `0x001` | STM -> S3 | `ACK_REQUIRED` | 1 B | STM emits static-done; S3 validates and ACKs/ERRORs |
| `ACK` | `0x005` | Both | `IS_ACK` | 4 B | Shared link consumes exact correlation |
| `ERROR` | `0x006` | Both | `IS_ERROR` | 4 B | Shared link completes pending transaction as remote error |
| `BOOT_READY` | `0x007` | STM -> S3 | `ACK_REQUIRED` | 2 B | STM retries while waiting; S3 admits expected wait-sync state |
| `WHEEL_SPEED_CMD` | `0x110` | S3 -> STM | `ACK_REQUIRED`; all-zero stop uses `STREAM_DATA` | 16 B, four f32 LE | STM validates, updates all four raw targets in wheel-independent mode, and ACKs nonzero transactions; the zero stop bypasses pending ACK slots |
| `PID_PARAMS_CMD` | `0x111` | S3 -> STM | `ACK_REQUIRED` | 16 B, four f32 LE | STM validates and atomically updates all four PID/Ramp instances, then ACKs |
| `WHEEL_SPEED_SINGLE_CMD` | `0x112` | S3 -> STM | `ACK_REQUIRED` | 5 B, wheel_id u8 + f32 LE | Enters wheel-independent mode and updates only the selected raw target |
| `MASTER_SPEED_CMD` | `0x113` | S3 -> STM | `ACK_REQUIRED` | 4 B, scale f32 LE | Updates MasterScale in the range 0..4 without changing raw targets |
| `CHASSIS_SPEED_CMD` | `0x114` | S3 -> STM | `ACK_REQUIRED` | 16 B, base_speed f32 LE + target_yaw_rate f32 LE + 8 B zero reserved | Enters chassis-diff mode and runs HEADING control when Primary IMU is valid |
| `ATTITUDE` | `0x201` | STM -> S3 | `STREAM_DATA` | 80 B | STM sends schema 2; S3 validates and relays App type `0x11` |
| `IMU_CAL_STATUS` | `0x202` | STM -> S3 | `STREAM_DATA` | 11 B | STM sends lifecycle status; S3 relays App type `0x12` |
| `IMU_TELEMETRY` | `0x207` | STM -> S3 | `STREAM_DATA` | 30 B | STM sends one frame per sensor; S3 relays App type `0x27` |
| `POWER_STATUS` | `0x209` | STM -> S3 | `STREAM_DATA` | 4 B, battery voltage f32 LE | STM emits latest finite MotorBoard voltage; S3 relays App type `0x1C` |
| `WHEEL_SPEED_STATUS` | `0x210` | STM -> S3 | `STREAM_DATA` | 16 B, four f32 LE | STM emits calibrated actual wheel speeds every 50 ms; S3 relays App type `0x16` |
| `CHASSIS_STATE` | `0x211` | STM -> S3 | `STREAM_DATA` | 24 B schema 1 | STM emits wheel-speed plus Primary-Yaw odometry and local safety state every 50 ms; S3 validates and relays App type `0x29` |
| `WHEEL_CONTROL_STATUS` | `0x212` | STM -> S3 | `STREAM_DATA` | 44 B schema 1 | STM emits mode, MasterScale, raw targets, and actual speeds every 50 ms; S3 relays App type `0x2C` |
| `RADAR_STATUS` | `0x301` | S3 -> STM and S3 -> App | `STREAM_DATA` | 2 B | S3 emits once per second while radar runs; current STM service has no consumer branch |
| `RADAR_PWM_READY` | `0x302` | S3 -> STM | `ACK_REQUIRED` | 1 B | S3 sends zero PWM; STM admits only during static calibration |
| `LOG` | `0x3F0` | STM -> S3 | `STREAM_DATA` | 8 B header + text | S3 validates and converts to separate log notification |

Do not revive historical `0x0200`, `0x0208`, `0x0401`, `0xF000`, 30-byte
ATTITUDE, `SC_TYPE_*`, PING/PONG, or legacy bias/result records.

## Payload Contracts

All fields are packed in the listed order. `u16`, `u32`, and `f32` mean
little-endian unsigned integers and IEEE-754 binary32 values. Lengths are
exact; reject shorter, longer, or semantically invalid payloads.

### Fast response: `ACK` / `ERROR`, 4 bytes

```text
offset 0: ack_can_id   u16 LE   original request CAN_ID
offset 2: ack_seq      u8       original request SEQ
offset 3: status_code  u8       0x00..0x05
```

### `BOOT_READY`, 2 bytes

```text
offset 0: state   u8   current STM boot transport state
offset 1: result  u8   0 = no error
```

Current startup sends `state=WAIT_SYNC` (stable value 1) and `result=0`. S3
admits only that exact pair while in `RADAR_WAIT_SYNC`; duplicate BOOT_READY is
accepted idempotently while its transport ACK is in flight.

### `CAL_EVENT`, 1 byte

| Value | Macro | Meaning |
| ---: | --- | --- |
| 1 | `SCBP_CAL_EVENT_STATIC_DONE` | Static calibration window completed |
| 2 | `SCBP_CAL_EVENT_VIB_STEP_DONE` | Reserved; no active S3 handler |
| 3 | `SCBP_CAL_EVENT_COMPLETE` | Reserved; no active S3 handler |

Current S3 admits only value 1 while waiting for the static event and PWM is
zero. Values 2 and 3 require a new two-endpoint design and tests.

### `RADAR_STATUS`, 2 bytes

```text
offset 0: online         u8   0 or 1
offset 1: speed_percent  u8   0..100
```

S3 emits this stream once per second while radar control reports running. The
App copy is a newly encoded BLE envelope. The STM wire definition exists, but
the current `s3_service_on_frame()` has no `RADAR_STATUS` consumer; an STM
consumer is an implementation change, not a documentation assumption.

### `RADAR_PWM_READY`, 1 byte

```text
offset 0: speed_percent  u8   current radar calibration PWM, 0..100
```

The current handshake uses exactly zero. STM admits it only during static
calibration and only at zero; S3 waits for `ACK(OK)` before waiting for
`CAL_EVENT`.

### `WHEEL_SPEED_CMD`, 16 bytes

```text
offset 0:  M1 RR target speed   f32 LE, mm/s
offset 4:  M2 RF target speed   f32 LE, mm/s
offset 8:  M3 LR target speed   f32 LE, mm/s
offset 12: M4 LF target speed   f32 LE, mm/s
```

The receiver rejects non-finite values and any length other than 16 bytes.
Nonzero targets are ACK-required transactions. An all-zero target is the
explicit safety stop and uses a realtime non-transactional frame so it cannot
wait for a pending ACK slot; the App ACK is emitted after S3 accepts the
transport send. The STM command watchdog clears all four targets after 1000 ms
without a valid command.

### `PID_PARAMS_CMD`, 16 bytes

```text
offset 0:  kp          f32 LE, 0.0..4.0
offset 4:  ki          f32 LE, 0.0..0.3
offset 8:  kd          f32 LE, 0.0..0.1
offset 12: max_accel   f32 LE, 200..2000 mm/s^2
```

The STM applies the complete tuple to M1=RR, M2=RF, M3=LR, and M4=LF under a
short critical section. A malformed, non-finite, or out-of-range payload is
rejected with `INVALID_PARAM`; runtime values revert to compile-time defaults
after reboot.

### `WHEEL_SPEED_STATUS`, 16 bytes

```text
offset 0:  M1 RR actual speed   f32 LE, mm/s
offset 4:  M2 RF actual speed   f32 LE, mm/s
offset 8:  M3 LR actual speed   f32 LE, mm/s
offset 12: M4 LF actual speed   f32 LE, mm/s
```

The STM emits this stream every 50 ms after applying the fixed motor-board
polarity map `(+1, -1, +1, +1)`.

### `WHEEL_SPEED_SINGLE_CMD`, 5 bytes

```text
offset 0: wheel_id u8: 0=RR, 1=RF, 2=LR, 3=LF
offset 1: raw target speed f32 LE, mm/s, absolute value <= 1000
```

This command enters `MODE_WHEEL_INDEPENDENT`, suspends HEADING control, and
updates only the selected raw target. `WHEEL_SPEED_CMD` uses the same mode and
updates all four raw targets while retaining the current MasterScale.

### `MASTER_SPEED_CMD`, 4 bytes

```text
offset 0: MasterScale f32 LE, range 0..4
```

The final PID target is `raw_target[i] * MasterScale`. Raw targets are retained.

### `CHASSIS_SPEED_CMD`, 16 bytes

```text
offset 0: base_speed       f32 LE, mm/s
offset 4: target_yaw_rate  f32 LE, rad/s
offset 8..15: reserved = 0.0 (all eight bytes zero)
```

The STM derives right-wheel targets as
`base_speed + 0.5 * target_yaw_rate * track_width_mm` and left-wheel targets
as `base_speed - 0.5 * target_yaw_rate * track_width_mm`, then enters
`MODE_CHASSIS_DIFF`. With a valid Primary IMU the runtime calls HEADING control;
when Primary is invalid it keeps the open-loop differential targets and logs
`[WARN] Heading bypassed: IMU invalid`.

### `WHEEL_CONTROL_STATUS`, 44 bytes

```text
offset 0: schema u8 = 1
offset 1: mode u8: 0=MODE_CHASSIS_DIFF, 1=MODE_WHEEL_INDEPENDENT
offset 2..3: reserved = 0
offset 4: timestamp_ms u32 LE
offset 8: MasterScale f32 LE
offset 12: raw_target[RR,RF,LR,LF] four f32 LE
offset 28: actual_speed[RR,RF,LR,LF] four f32 LE
```

### `CHASSIS_STATE`, 24 bytes

```text
offset 0:  schema        u8 = 1
offset 1:  flags         bit 0 attitude safety fused; bit 1 heading lock;
                         bit 2 odometry valid; bit 3 attitude startup ready
offset 2:  reserved      u16 LE = 0
offset 4:  timestamp_ms  u32 LE
offset 8:  x_mm          f32 LE
offset 12: y_mm          f32 LE
offset 16: yaw_deg       f32 LE
offset 20: total_dist_m  f32 LE
```

CM7 integrates the four calibrated `MSPD` actual speeds over 50 ms and uses
Primary DualAHRS Yaw to project the body-forward increment. It does not infer
distance from unscaled `MTEP` pulse counts. S3 rejects any other schema,
nonzero reserved bytes, reserved flag bits, invalid length, or non-finite
float before constructing the App BLE `0x29` frame.

### `POWER_STATUS`, 4 bytes

```text
offset 0: battery_voltage   f32 LE, volts
```

The STM emits the latest finite MotorBoard battery voltage every 500 ms.

### `IMU_CAL_STATUS`, 11 bytes

```text
offset 0:  stage        u8
offset 1:  radar_pwm    u8
offset 2:  sample_count u32 LE
offset 6:  sample_total u32 LE
offset 10: error_code   u8
```

| Stage | Macro | Meaning |
| ---: | --- | --- |
| 0 | `SCBP_IMU_CAL_STAGE_WAIT_RADAR_READY` | Waiting for zero-PWM synchronization |
| 1 | `SCBP_IMU_CAL_STAGE_STATIC_STABLE_WAIT` | Zero PWM admitted; settling |
| 2 | `SCBP_IMU_CAL_STAGE_STATIC_SAMPLE` | Static calibration window active |
| 3 | `SCBP_IMU_CAL_STAGE_COMPLETE` | Lifecycle ready |
| 4 | `SCBP_IMU_CAL_STAGE_ERROR` | Lifecycle failed; inspect error code |

Progress fields are presentation/diagnostic data, not a substitute for the
calibration quality decision.

### `IMU_TELEMETRY`, 30 bytes per sensor

```text
offset 0:  sensor_id   u8
offset 1:  flags       u8
offset 2:  timestamp   u32 LE, milliseconds
offset 6:  accel[3]    f32 LE, x/y/z, m/s^2
offset 18: vector[3]   f32 LE, sensor-specific second vector
```

| Sensor ID | Vector meaning |
| ---: | --- |
| 1 (`LSM303`) | Magnetometer x/y/z, microtesla |
| 2 (`BMI323`) | Gyroscope x/y/z, rad/s |

| Flag | Meaning |
| ---: | --- |
| bit 0 | Acceleration valid |
| bit 1 | Magnetometer (LSM303) or gyro (BMI323) valid |
| bit 2 | Sensor online |

The current STM runtime schedules both frames every 100 ms when a snapshot is
available. Do not silently swap sensor identity.

### `ATTITUDE`, schema 2, 80 bytes

```text
offset 0:  schema              u8   must be 2
offset 1:  flags               u8
offset 2:  reserved            u16 LE, must be 0
offset 4:  timestamp_ms        u32 LE
offset 8:  sample_sequence     u32 LE
offset 12: primary_euler[3]    f32 LE, roll/pitch/yaw, radians
offset 24: primary_quat[4]     f32 LE, w/x/y/z
offset 40: redundant_euler[3]  f32 LE, roll/pitch/yaw, radians
offset 52: redundant_quat[4]   f32 LE, w/x/y/z
offset 68: delta_euler[3]      f32 LE, primary-vs-redundant delta, radians
```

| Flag | Meaning |
| ---: | --- |
| bit 0 | Primary valid |
| bit 1 | Redundant valid |
| bit 2 | LSM303 magnetometer valid |
| bit 4 | Gravity confidence below threshold |
| bit 5 | Magnetic confidence below threshold |
| bit 6 | Input path stale |
| bit 7 | DualAHRS fault |
| bit 3 | Currently unused; preserve as zero |

STM sends this stream every 50 ms when packing succeeds. S3 requires length
80, schema 2, and reserved bytes zero, then puts the same payload bytes into
App BLE type `0x11`. No unit conversion or quaternion reordering occurs.

### `LOG`, variable payload

The SCBP payload is not the final BLE log frame:

```text
offset 0: source       u8   0 = STM32, 1 = S3
offset 1: level        u8   0 = debug, 1 = info, 2 = warn, 3 = error
offset 2: timestamp_ms u32 LE
offset 6: text_length  u16 LE
offset 8: text         UTF-8 bytes, text_length bytes
```

STM limits text to 96 bytes, so the SCBP payload is at most 104 bytes. S3
requires `frame.length == 8 + text_length`, then re-encodes a separate
`AA 55 01 ... CRC16` log notification.

## Calibration Transaction

```text
STM                                  S3
 |-- BOOT_READY(state=1,result=0) -->|
 |<------------- ACK(OK) ------------|
 |                                   | set radar PWM to 0
 |<-- RADAR_PWM_READY(speed=0) ------|
 |--------------- ACK(OK) ----------->|
 |                                   | wait for static event
 |-- IMU_CAL_STATUS(stream) -------->|
 |-- CAL_EVENT(STATIC_DONE=1) ------>|
 |<------------- ACK(OK) ------------|
 |                                   | release calibration lock / running
```

STM does not open the static window until zero-PWM `RADAR_PWM_READY` is
admitted. S3 does not release its calibration lock until `CAL_EVENT=1` is
admitted. Failed zero-PWM transport/ACK or a later static-event timeout
returns S3 to `RADAR_WAIT_SYNC`, clears the calibration gate, and keeps PWM at
zero. Duplicate BOOT_READY and already-completed static events are idempotent
in the current S3 manager.

This is a source-level state contract, not proof of physical PWM, UART, IMU,
radar, or running-state behavior.

## Command Extension Rules

Before calling a new command complete:

1. Assign an unused message value in the 10-bit range. Never truncate a
   historical 16-bit ID into the low 10 bits or reuse a deprecated ID.
2. State owner, source, destination, priority, direction, and stream/transaction type.
3. Define exact length and byte offsets, endianness, float format, units, ranges, reserved values, and versioning.
4. Add shared `SCBP_MSG_ID_*`, payload-size, and enum definitions to `Common/SCBP_CAN/include/scbp_protocol_defs.h`.
5. Add explicit serialization/deserialization on both endpoints; do not rely on packed C layout or unaligned casts.
6. Add both endpoint producers/consumers. Constants alone or one-sided dispatch are incomplete.
7. For transactions, define admission, idempotency, status mapping, retry behavior, and completion callback. ACK only after the required local state commit.
8. For streams, define cadence, freshness/staleness, drop policy, and App BLE re-enveloping behavior.
9. Update this registry, `DOCS/protocol/protocol.md`, and affected module docs together.
10. Add host tests for golden bytes, fragmentation/concatenation, bad HCS/FCS/EOF, invalid flags/nodes, exact length, ACK correlation, retry/timeout, and duplicate delivery as applicable.

## Verification Matrix

| Layer | Minimum evidence | Does not prove |
| --- | --- | --- |
| Static/source audit | Shared constants, both call sites, exact payload tables, no ID collision | Runtime scheduling or physical transfer |
| Host codec tests | CRC, parser, link, fragmentation, retry behavior | MCU DMA/cache or electrical integrity |
| STM build | CM7 clean configure/build and final ELF path | S3 compatibility or board behavior |
| S3 build | ESP-IDF build with shared component and dispatch | STM compatibility or BLE/radar behavior |
| UART capture | Matching flashed images, 921600 8N1 decoded both directions | Sensor quality, motor/radar safety, App rendering |
| End-to-end capture | UART -> S3 validation -> new App BLE frame -> App decode | Vehicle acceptance |

Current host test and endpoint commands:

```bash
cc -std=c11 -Wall -Wextra -Werror -pedantic -ICommon/SCBP_CAN/include Common/SCBP_CAN/scbp_crc.c Common/SCBP_CAN/scbp_wire.c Common/SCBP_CAN/scbp_parser.c Common/SCBP_CAN/scbp_link.c Common/SCBP_CAN/tests/test_scbp_can.c -o /tmp/test_scbp_can
/tmp/test_scbp_can

# From STM32H757/CM7
cmake --preset Debug
cmake --build --preset Debug --clean-first

# With ESP-IDF 5.5.4 loaded
idf.py -B build-s3-bridge build
```

Host tests and builds are source/build evidence only. Do not report hardware
acceptance without matching flash and capture steps.

## Separate Or Deprecated Contracts

- Historical SCBP-V3 `AA 55 | VERSION | TYPE/ID ...` framing is deprecated for
  the STM-S3 UART; it is not a compatibility mode of SCBP-CAN.
- App control/telemetry uses `AA | 01 | TYPE | LEN_LE | PAYLOAD | CRC16-MODBUS_LE | 55`.
- App BLE types are `0x15=WHEEL_SPEED_CMD` (16 B), `0x16=WHEEL_SPEED_STATUS`
  (16 B), `0x1A=RADAR_STATUS` (2 B), `0x1B=RADAR_PWM_SET` (1 B),
  `0x1C=POWER_STATUS` (4 B), `0x1D=PID_PARAMS_CMD` (16 B),
  `0x29=CHASSIS_STATE` (24 B), `0x2A=WHEEL_SPEED_SINGLE_CMD` (5 B),
  `0x2B=MASTER_SPEED_CMD` (4 B), `0x2C=WHEEL_CONTROL_STATUS` (44 B), and
  `0x2D=CHASSIS_SPEED_CMD` (16 B).
- SmartCar log notification uses a separate `AA 55 01 ... CRC16` envelope.
- Historical `SC_TYPE_*`, PING/PONG, `0x0200`, `0x0208`, `0x0401`, `0xF000`,
  legacy 30-byte ATTITUDE, and old bias/result payloads are `DEPRECATED` or
  `RESERVED`.

## Change Record

| Date | Change |
| --- | --- |
| 2026-08-21 | Initial source-based STM32-S3 SCBP-CAN command reference; separates wire definitions, endpoint dispatch, App/log envelopes, and hardware evidence boundaries. |
