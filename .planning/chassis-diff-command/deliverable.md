# Deliverable: Chassis Differential Command

## Core Snippets

### STM32 protocol and dispatch

```c
/* Common/SCBP_CAN/include/scbp_protocol_defs.h */
#define SCBP_MSG_ID_CHASSIS_SPEED_CMD UINT16_C(0x114)
#define SCBP_PAYLOAD_CHASSIS_SPEED_CMD_SIZE UINT16_C(16)
```

```c
/* STM32H757/Middleware/Communication/Services/s3_service.c */
if (message_id == SCBP_MSG_ID_CHASSIS_SPEED_CMD) {
    float base_speed = 0.0f;
    float target_yaw_rate = 0.0f;
    bool valid = frame->length == SCBP_PAYLOAD_CHASSIS_SPEED_CMD_SIZE &&
                 frame->payload != NULL;
    if (valid) {
        base_speed = scbp_wire_read_f32_le(&frame->payload[0]);
        target_yaw_rate = scbp_wire_read_f32_le(&frame->payload[4]);
        for (size_t i = 8U; i < SCBP_PAYLOAD_CHASSIS_SPEED_CMD_SIZE; ++i) {
            if (frame->payload[i] != 0U) { valid = false; break; }
        }
        valid = valid && isfinite(base_speed) && isfinite(target_yaw_rate);
    }
    if (valid) {
        valid = chassis_runtime_set_mode(MODE_CHASSIS_DIFF) &&
                chassis_runtime_apply_diff_command(base_speed, target_yaw_rate);
    }
    if (ack_required != 0U) {
        s3_service_send_response_locked(
            frame, valid ? 0U : 1U,
            valid ? SCBP_FAST_RESP_OK : SCBP_FAST_RESP_INVALID_PARAM);
    }
    return;
}
```

```c
/* STM32H757/Application/Chassis/chassis_runtime.c */
if (control_mode == MODE_WHEEL_INDEPENDENT) {
    chassis_heading_control_reset();
} else if (primary_valid) {
    chassis_heading_control_step(target_vx, target_wz_rad_s, yaw_deg,
                                 gyro_z_degps, dt_s,
                                 &controlled_left_vx, &controlled_right_vx);
    output_speeds[0] = output_speeds[1] = controlled_right_vx;
    output_speeds[2] = output_speeds[3] = controlled_left_vx;
} else {
    chassis_heading_control_reset();
    chassis_runtime_log_imu_degraded();
}
```

### ESP32-S3 bridge

```c
/* ESPS3/components/smartcar_protocol/include/app_parser.h */
#define SC_APP_TYPE_CHASSIS_SPEED_CMD 0x2DU
```

The bridge requires `frame->length == 16`, finite f32 values, and zero reserved
bytes before sending SCBP `0x114` with `ACK_REQUIRED`; the STM response is
returned as App ACK for type `0x2D`.

### Active macOS App

```swift
// IOS_APP/SmartCar_Control_MAC/.../BLE/BLEManager.swift
func sendChassisSpeed(baseSpeed: Float, yawRate: Float) {
    guard baseSpeed.isFinite, yawRate.isFinite else { return }
    var payload = Self.floatPayload([baseSpeed, yawRate])
    payload.append(Data(repeating: 0, count: 8))
    _ = sendFrame(SmartCarProtocol.encode(type: .chassisSpeedCommand,
                                          payload: payload))
}
```

`driveStraight()` stops the old single-wheel timers before sending
`baseSpeed: 190.0`; individual wheel sliders continue to call
`sendWheelSpeeds()` (`App 0x15 -> SCBP 0x110`). The `0x2C`
decoder reads `bytes[1]` as `ChassisControlMode`; the UI renders chassis diff
green and wheel-independent yellow.

## Validation Boundary

Completed: shared host protocol test, macOS Swift build, CM7 clean build,
ESP-IDF bridge build, and whitespace check. Not completed: flashing, live BLE
or UART capture, IMU-primary runtime observation, motor response, or vehicle
straight-line acceptance.
