# App BLE Protocol V1

## Boundary

App BLE is not SRP. Its envelope remains:

```text
AA | 01 | TYPE | LEN_LO | LEN_HI | PAYLOAD[LEN] | CRC_LO | CRC_HI | 55
```

CRC16-MODBUS covers the version through the payload. ESP32-S3 parses this
format for inbound App commands and constructs this format for selected SRP
telemetry. It never relays raw SRP bytes to BLE.

## Current S3 Outbound Mapping

| SRP UART message | App BLE type | Exact payload |
| --- | ---: | --- |
| `ATTITUDE(0x11)` | `0x11` | 80-byte schema-2 DualAHRS, byte-preserving |
| `IMU_CAL_STATUS(0x12)` | `0x12` | 11 bytes |
| `WHEEL_SPEED_STATUS(0x14)` | `0x16` | 16 bytes, four float32 LE mm/s values |
| `CHASSIS_STATE(0x15)` | `0x29` | 24-byte schema-1 pose and safety state, byte-preserving |
| `WHEEL_CONTROL_STATUS(0x16)` | `0x2C` | 44-byte schema-1 mode, MasterScale, raw targets, and actual speeds |
| `RADAR_STATUS(0x20)` | `0x1A` | 2 bytes |
| `POWER_STATUS(0x13)` | `0x1C` | 4 bytes, float32 LE volts |
| `IMU_TELEMETRY(0x10)` | `0x27` | 30 bytes |
| App command response | `0x06` | `acknowledged_type_u8, result_u8` |

App commands use `0x15=WHEEL_SPEED_CMD` with a 16-byte payload,
`0x2A=WHEEL_SPEED_SINGLE_CMD` with `[wheel_id_u8, speed_f32_le]`, and
`0x2B=MASTER_SPEED_CMD` with one `scale_f32_le` value. `0x2D=CHASSIS_SPEED_CMD`
uses two f32 little-endian values (`base_speed_mmps`, `target_yaw_rate_rad_s`)
followed by eight zero reserved bytes and selects chassis-diff mode with
HEADING control eligible. The existing
`0x2E=CHASSIS_HEADING_CMD` uses `[target_v_mm_s_f32_le,
target_yaw_deg_f32_le, flags_u32_le]` (12 bytes). `flags` must be zero; S3
forwards it as ACK-required SRP `0x17` and STM32 applies the local Target Yaw
closed loop. The existing
`0x15` command remains the four-wheel batch command and, like `0x2A`, selects
wheel-independent mode and suspends HEADING control.
The App also receives
`0x2C=WHEEL_CONTROL_STATUS` with the current mode, MasterScale, raw targets,
and actual speeds. A zero-valued `0x15` wheel tuple is the explicit stop path;
S3 sends it as a realtime non-transactional SRP frame and returns the App ACK
after transport admission, while nonzero motion commands retain the normal
STM ACK completion. App commands also include
`0x1B=RADAR_PWM_SET` with one speed-percent byte. PID tuning uses
`0x1D=PID_PARAMS_CMD` with four float32 little-endian values in the order
`kp, ki, kd, max_accel`; the S3 returns App ACK `0x06` only after STM ACK.
Text logs remain on the separate FFE3 notification path in the existing
SmartCar log envelope.

`CHASSIS_STATE` uses `schema=1`; its flags report attitude-safety fuse,
heading-lock state, odometry validity, and attitude-startup readiness. The App
must reject a different schema, nonzero reserved bytes, nonzero reserved flag
bits, a length other than 24 bytes, or non-finite pose/distance values.

## Compatibility Boundary

The UART migration removes V3 compatibility only from the STM32-S3 link. It
does not redefine the App BLE envelope or its independent parser. However,
the S3 bridge no longer emits legacy UART-derived IMU status, dual-status,
bias, calibration-result, or 30-byte attitude payloads. App consumers must
expect the 80-byte schema-2 attitude payload for new UART-originated data.

BLE delivery and App rendering are not demonstrated by firmware builds.
