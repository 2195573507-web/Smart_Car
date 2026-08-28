# SmartCar App-BLE Protocol V2

Status: implementation baseline, 2026-08-23

This document is the canonical contract for the App-to-ESP32-S3 Bluetooth
link. It covers the GATT transport, the V1-compatible envelope, the V2
session layer, and the mapping boundary to SRP/SCBP-CAN. It does not grant the
App authority over STM32 motor safety.

## 1. GATT Contract

The existing service and characteristic UUIDs remain unchanged:

| UUID | Name | Direction | Use |
| --- | --- | --- | --- |
| `0000FFE0-0000-1000-8000-00805F9B34FB` | FFE0 | service | SmartCar S3 GATT service |
| `0000FFE1-0000-1000-8000-00805F9B34FB` | FFE1 | App -> S3 write | App commands and session frames |
| `0000FFE2-0000-1000-8000-00805F9B34FB` | FFE2 | S3 -> App notify | Telemetry, ACKs, and session frames |
| `0000FFE3-0000-1000-8000-00805F9B34FB` | FFE3 | S3 -> App notify | Independent SmartCar text log envelope |

FFE1 writes use `.withResponse` in the two Swift clients. Each logical frame
may be split at the negotiated ATT write length; the receiver must feed the
bytes into a stream parser and must support fragmentation and coalescing.
FFE3 has its own parser and must not be mixed with FFE2 bytes.

## 2. Common Frame Envelope

Both protocol versions use the same byte layout. Only `VERSION` differs:

```text
AA | VERSION | TYPE | LEN_LO | LEN_HI | PAYLOAD[LEN] | CRC_LO | CRC_HI | 55
```

- `AA` and `55` are fixed head and tail bytes.
- `VERSION` is `0x01` for V1 or `0x02` for V2.
- `LEN` is an unsigned little-endian payload length, maximum 128 bytes.
- CRC is CRC16-MODBUS, initial `0xFFFF`, polynomial `0xA001`, transmitted
  little-endian. It covers `VERSION` through the final payload byte.
- A parser must discard noise, reject an invalid length, reject a bad tail or
  CRC, and continue searching for the next `AA` without unbounded buffering.

## 3. V1 Compatibility

V1 keeps the deployed command and telemetry types. The S3 must continue to
accept V1 clients and return the two-byte ACK payload:

```text
ACK (TYPE=0x06): acknowledged_type_u8 | result_u8
```

Important command types at this boundary are:

| App type | Meaning | Payload |
| ---: | --- | ---: |
| `0x15` | WHEEL_SPEED_CMD | four float32 LE wheel targets, 16 bytes |
| `0x2A` | WHEEL_SPEED_SINGLE_CMD | `wheel_id_u8` + float32 LE, 5 bytes |
| `0x2B` | MASTER_SPEED_CMD | float32 LE scale, 4 bytes |
| `0x2D` | CHASSIS_SPEED_CMD | base speed + yaw rate, two float32 LE plus 8 zero reserved bytes, 16 bytes |
| `0x2E` | CHASSIS_HEADING_CMD | target speed + target yaw, two float32 LE plus zero `uint32_t` flags, 12 bytes |
| `0x1D` | PID_PARAMS_CMD | `kp, ki, kd, max_accel`, four float32 LE, 16 bytes |
| `0x1B` | RADAR_PWM_SET | speed percent, 1 byte |

The all-zero `0x15` tuple is the explicit stop path. It is sent to STM32 as
an immediate non-transactional zero-wheel SRP frame. A BLE disconnect also
causes the S3 service task to serialize a zero-wheel SRP frame. Nonzero motion
uses the existing SRP ACK transaction and STM32 command watchdog.

## 4. V2 Types and Payloads

V2 frames are carried on FFE1/FFE2 with `VERSION=0x02`.

| Type | Name | Direction | Payload length |
| ---: | --- | --- | ---: |
| `0x70` | HELLO | App -> S3 | 6 |
| `0x71` | HELLO_ACK | S3 -> App | 13 |
| `0x72` | HEARTBEAT | App -> S3 | 8 |
| `0x73` | HEARTBEAT_ACK | S3 -> App | 9 |
| `0x74` | COMMAND_ACK | S3 -> App | 11 |
| `0x75` | COMMAND | App -> S3 | 11 + preserved V1 payload |

### HELLO (0x70)

```text
version_min_u8 | version_max_u8 | capabilities_u32_le
```

The current clients send `02 | 02 | 07 00 00 00`. The S3 admits only the
current V2 range (`version_min=2`, `version_max=2`) and assigns a fresh,
nonzero session ID.

### HELLO_ACK (0x71)

```text
version_u8 | session_id_u32_le | heartbeat_ms_u16_le |
session_ttl_ms_u16_le | capabilities_u32_le
```

The current S3 values are version `2`, heartbeat `500 ms`, session TTL
`3000 ms`, and capability bits `0x00000007`.

### HEARTBEAT (0x72) and HEARTBEAT_ACK (0x73)

```text
HEARTBEAT:     session_id_u32_le | heartbeat_sequence_u32_le
HEARTBEAT_ACK: session_id_u32_le | heartbeat_sequence_u32_le | result_u8
```

The App sends a heartbeat every negotiated interval. A valid heartbeat
refreshes the S3 session activity timestamp. An invalid or expired session is
not refreshed and receives no successful ACK.

### COMMAND (0x75)

```text
session_id_u32_le | command_sequence_u32_le | valid_for_ms_u16_le |
original_v1_type_u8 | original_v1_payload[LEN-11]
```

The original V1 type and payload are byte-preserved. `valid_for_ms` is
receiver-relative and must be in the inclusive range `20..1000 ms`.

### COMMAND_ACK (0x74)

```text
session_id_u32_le | command_sequence_u32_le | acknowledged_type_u8 |
result_u8 | stage_u8
```

Result values:

| Value | Name | Meaning |
| ---: | --- | --- |
| `0x00` | OK | Accepted or completed successfully |
| `0x01` | REJECTED | Payload or downstream command rejected |
| `0x02` | SESSION_INVALID | Session absent or wrong |
| `0x03` | EXPIRED | `valid_for_ms` outside range or elapsed before downstream admission |
| `0x04` | STALE_SEQUENCE | Older sequence with no cached result |
| `0x05` | BUSY | Reserved for admission back-pressure |

Stage values:

| Value | Name | Meaning |
| ---: | --- | --- |
| `0x00` | GATEWAY_ADMITTED | S3 validated and admitted the request |
| `0x01` | STM32_ACCEPTED | Existing SRP/STM ACK completed successfully |
| `0x02` | STOP_QUEUED | Explicit zero stop was serialized to SRP |

## 5. Session and Sequence Rules

1. After GATT discovery and FFE2 notify enable, the App sends HELLO and enters
   `negotiating`. No nonzero motion command is emitted before V2 readiness or
   the bounded V1 fallback deadline.
2. A valid HELLO creates a new S3 session. Receiving another HELLO creates a
   new session and invalidates the previous session ID.
3. A command is accepted only when the session ID matches, `valid_for_ms` is
   valid, and `command_sequence` is greater than the last admitted sequence.
4. If a duplicate sequence equals the most recently completed command, the S3
   resends the cached `COMMAND_ACK` without repeating the SRP/STM32 effect.
   Older sequences receive `STALE_SEQUENCE`.
5. S3 session expiry is 3000 ms without valid activity. Expiry clears pending
   motion and queues a serialized zero-wheel SRP frame. BLE disconnect follows
   the same stop path. STM32's watchdog and BUS_OFF recovery remain active.
6. The App distinguishes GATT connected, V2 negotiating, V2 ready, V1
   fallback, and session expired. On expiry it clears unsent nonzero motion,
   sends a best-effort V1 zero frame, and pauses motion until renegotiation.

## 6. Downstream Mapping and Authority

The S3 unwraps a V2 command into the preserved V1 type/payload and then uses
the existing SRP link. Current mappings include:

| App command | SRP message | STM32 contract |
| --- | --- | --- |
| `0x15` wheel tuple | `SRP_MSG_ID_WHEEL_SPEED_CMD` (`0x02`) | nonzero ACK transaction; zero immediate stream stop |
| `0x2A` single wheel | `SRP_MSG_ID_WHEEL_SPEED_SINGLE_CMD` (`0x04`) | existing independent-wheel validation |
| `0x2B` master scale | `SRP_MSG_ID_MASTER_SPEED_CMD` (`0x05`) | existing global scale contract |
| `0x2D` chassis speed | `SRP_MSG_ID_CHASSIS_SPEED_CMD` (`0x06`) | existing chassis/heading contract |
| `0x2E` Target Yaw | `SRP_MSG_ID_CHASSIS_HEADING_CMD` (`0x17`) | ACK-required 12-byte `v, yaw_deg, flags=0` local heading loop |
| `0x1D` PID | `SRP_MSG_ID_PID_PARAMS_CMD` (`0x03`) | ACK-gated parameter update |

STM32 remains the final motion authority. App-BLE V2 cannot bypass encoder
sign rules, local interlocks, attitude safety, BUS_OFF handling, or the
command watchdog.

## 7. Test Vectors

All bytes are hexadecimal and include the envelope.

V2 HELLO, payload `02 02 07 00 00 00`:

```text
AA 02 70 06 00 02 02 07 00 00 00 4D 73 55
```

V1 all-zero wheel stop:

```text
AA 01 15 10 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 15 00 55
```

V2 command with session `0x01020304`, sequence `1`, validity `500 ms`, and a
zero wheel payload:

```text
AA 02 75 1B 00 04 03 02 01 01 00 00 00 F4 01 15 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 FE 3C 55
```

## 8. Verification Boundary

Swift unit tests, app builds, and an ESP-IDF build prove source-level and
link-level consistency only. BLE packet captures, negotiated MTU behavior,
V1 fallback against old firmware, UART propagation, STM32 ACK timing, session
expiry, physical stop behavior, and vehicle acceptance require a matching
flashed image and live hardware tests.
