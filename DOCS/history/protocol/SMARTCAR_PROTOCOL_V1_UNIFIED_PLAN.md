# SmartCar Protocol V1

## Status and Scope

This is the sole normative protocol definition for the implemented SmartCar
AA/55 envelope shared by the App, ESP32-S3 gateway, and STM32 firmware. It
defines bytes on the wire only. BLE and UART are transports for the same frame;
this document does not define their connection, fragmentation, timing, or
retry behavior.

All receivers must reject a payload length greater than `128` bytes. This limit
is identical on all three endpoints. The maximum complete frame size is
`136` bytes (`128 + 8` overhead).

## Frame Format

```text
+--------+---------+------+-----------+-------------+---------+--------+------+
| HEAD   | VERSION | TYPE | LENGTH_LE | PAYLOAD     | CRC_LE  | TAIL   |
+--------+---------+------+-----------+-------------+---------+--------+------+
| 1 byte | 1 byte  | 1    | 2 bytes   | 0..128 byte | 2 bytes | 1 byte |
+--------+---------+------+-----------+-------------+---------+--------+------+
```

| Offset | Field | Value or encoding |
| --- | --- | --- |
| 0 | `HEAD` | `0xAA` |
| 1 | `VERSION` | `0x01` |
| 2 | `TYPE` | Message type listed below |
| 3..4 | `LENGTH_LE` | Unsigned payload byte count, little-endian, `0..128` |
| 5..(4+N) | `PAYLOAD` | `N` bytes, defined by `TYPE` |
| 5+N..6+N | `CRC_LE` | CRC16-MODBUS, low byte first |
| 7+N | `TAIL` | `0x55` |

The total frame length is `8 + LENGTH`. `HEAD` and `TAIL` are framing bytes;
they are not payload data.

## CRC16-MODBUS

`CRC_LE` is CRC16-MODBUS over the consecutive bytes from `VERSION` through the
last `PAYLOAD` byte. `HEAD`, the two CRC bytes, and `TAIL` are excluded.

| Parameter | Value |
| --- | --- |
| Initial value | `0xFFFF` |
| Polynomial, reflected | `0xA001` (normal form `0x8005`) |
| Input/output reflection | Reflected |
| Final XOR | `0x0000` |
| Wire order | CRC low byte, then CRC high byte |

For an empty payload, the CRC input is exactly four bytes:
`VERSION`, `TYPE`, `LENGTH_LO`, `LENGTH_HI`.

## TYPE Definitions

| TYPE | Name | Defined direction | Payload |
| --- | --- | --- | --- |
| `0x01` | `CONTROL` | App -> S3 -> STM32 | Control command |
| `0x02` | `STATUS` | STM32 -> S3 -> App | Vehicle status |
| `0x05` | `PING` | App -> S3 -> STM32 | Empty |
| `0x06` | `ACK` | STM32 -> S3 -> App | Acknowledged type and result |
| `0x10` | `IMU_STATUS` | STM32 -> S3 -> App | IMU sensor status and vectors |
| `0x11` | `ATTITUDE` | STM32 -> S3 -> App | Euler angles and quaternion |

The S3 relays valid frames between its BLE and STM32 UART sides; it does not
redefine the message payloads in this protocol version.

## Payload Definitions

### CONTROL (`0x01`)

| Byte(s) | Field | Encoding |
| --- | --- | --- |
| 0 | `command` | Control command code |
| 1, optional | `speed_u8` | Unsigned speed value; valid only for `SPEED_CONTROL` |
| 1..2, optional | `value_i16_le` | Signed 16-bit little-endian value; valid only for `SPEED_CONTROL` |

Accepted payload lengths are `1`, `2`, or `3`. A length of `2` carries
`speed_u8`; a length of `3` carries `value_i16_le`. The current App generates
a 1-byte command payload, except `SPEED_CONTROL`, which it generates as a
2-byte `command + speed_u8` payload.

| Command | Code |
| --- | --- |
| `STOP` | `0x01` |
| `MOVE_FORWARD` | `0x02` |
| `MOVE_BACK` | `0x03` |
| `TURN_LEFT` | `0x04` |
| `TURN_RIGHT` | `0x05` |
| `SPEED_CONTROL` | `0x06` |

### STATUS (`0x02`)

Payload length is exactly `4` bytes.

| Byte(s) | Field | Encoding |
| --- | --- | --- |
| 0 | `battery` | `uint8` |
| 1 | `motor_state` | `uint8`; STM32 currently reports the last valid control code, or `0` |
| 2..3 | `error_code` | `uint16`, little-endian |

### PING (`0x05`)

Payload length is exactly `0` bytes.

### ACK (`0x06`)

Payload length is exactly `2` bytes.

| Byte | Field | Encoding |
| --- | --- | --- |
| 0 | `acknowledged_type` | The TYPE being acknowledged |
| 1 | `result` | `0x00` accepted; `0x01` rejected by STM32 control-payload validation |

### IMU_STATUS (`0x10`)

Payload length is exactly `38` bytes.

| Byte(s) | Field | Encoding |
| --- | --- | --- |
| 0 | `sensor_id` | `0x01` BMI323, `0x02` LSM303 |
| 1 | `online` | `0x00` offline, `0x01` online |
| 2..5 | `accel_x` | `float32_le` |
| 6..9 | `accel_y` | `float32_le` |
| 10..13 | `accel_z` | `float32_le` |
| 14..17 | `gyro_x` | `float32_le` |
| 18..21 | `gyro_y` | `float32_le` |
| 22..25 | `gyro_z` | `float32_le` |
| 26..29 | `mag_x` | `float32_le` |
| 30..33 | `mag_y` | `float32_le` |
| 34..37 | `mag_z` | `float32_le` |

### ATTITUDE (`0x11`)

Payload length is exactly `28` bytes.

| Byte(s) | Field | Encoding |
| --- | --- | --- |
| 0..3 | `roll` | `float32_le` |
| 4..7 | `pitch` | `float32_le` |
| 8..11 | `yaw` | `float32_le` |
| 12..15 | `quaternion_w` | `float32_le` |
| 16..19 | `quaternion_x` | `float32_le` |
| 20..23 | `quaternion_y` | `float32_le` |
| 24..27 | `quaternion_z` | `float32_le` |

## Little-Endian Rules

Every multi-byte numeric field in a frame or payload uses little-endian byte
order. For an unsigned 16-bit value `V`, transmit `V & 0xFF` first, then
`V >> 8`. This applies to `LENGTH_LE`, `CRC_LE`, `error_code`, and
`value_i16_le`; signed integer fields use the same two's-complement bit layout.

Do not serialize C structures directly. Serialize each field at its documented
offset so compiler padding and host byte order cannot change the wire format.

## float32 Rules

`float32_le` is exactly four bytes containing an IEEE 754 binary32 bit pattern
in little-endian order. The value is serialized and decoded by its raw 32-bit
bit pattern, not as text and not by numeric conversion to an integer. All
three endpoints must preserve the bit pattern, including signed zero, infinity,
and NaN payloads if present.

## Compatibility

The prior A5/5A, CRC-16/CCITT-FALSE proposal is deprecated and is not
interoperable with this protocol. See
[SMART_CAR_PROTOCOL_A5_5A_PLAN.md](SMART_CAR_PROTOCOL_A5_5A_PLAN.md) only for
historical context.
