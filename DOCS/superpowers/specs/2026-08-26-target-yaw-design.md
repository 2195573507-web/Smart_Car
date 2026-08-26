# Target Yaw 定向巡航设计

## 目标

在现有 STM32H757 CM7 底盘控制链路中增加目标偏航角闭环巡航。手机通过
BLE 将 12 字节目标命令交给 ESP32-S3，S3 以 SRP v4 `0x17` 事务转发，CM7
在 10 ms 底盘周期内使用 DualAHRS Primary yaw 和校正后的 Z 轴角速度产生
差速修正。

## 协议与数据流

```text
App BLE V1/V2
    -> SC_APP_TYPE_CHASSIS_HEADING_CMD (0x2E, 12 B)
ESP32-S3 command_bridge
    -> SRP_MSG_ID_CHASSIS_HEADING_CMD (0x17, ACK_REQUIRED, 12 B)
STM32 CM7 s3_service
    -> chassis_task_set_heading_target(v_mm_s, target_yaw_deg)
底盘周期
    -> DualAHRS heading state -> PD -> chassis_kinematics_compute -> MotorBoard
```

Payload is little-endian `float target_v_mm_s`, `float target_yaw_deg`, and
`uint32_t flags`. The only accepted flags value is zero. Values must be finite;
the linear speed must be within the existing chassis wheel-speed contract.
Target yaw is normalized to the estimator's `[-180, 180)` degree convention.

## CM7 控制

The heading target is stored separately from the existing direct `v/omega`
target. While active, the 10 ms task reads the latest DualAHRS primary yaw and
the last filtered, bias-corrected, body-frame gyro Z. It computes the shortest
angle error, then applies `w_corr = clamp(0.06 * error_rad - 0.002 * gyro_z,
-2, 2)`. The resulting linear speed and angular correction are passed through
the existing kinematics and safety gate. If attitude data is invalid, the
output is zero and the target is cleared by the existing gate-revocation path.

The control diagnostic is rate limited to 500 ms:
`[HEADING_CTRL] target_yaw=%.2f, cur_yaw=%.2f, err=%.2f, w_corr=%.4f`.

## 安全与生命周期

Heading and direct velocity targets are mutually exclusive: receiving either
command replaces the other mode. S3 sync timeout, BUS_OFF recovery, BLE
disconnect/session expiry, local emergency stop, or a revoked CM7 output gate
clears both target modes and sends zero wheel targets. No target is restored
automatically after a link or IMU recovery.

## 验证

- Shared SRP registry and host codec tests cover ID, exact 12-byte length, and
  little-endian payload handling.
- S3 and CM7 compile with strict payload and flags validation.
- `STM32H757/CM7/build/Debug` clean build is the firmware integration check.
- Live UART/BLE captures and physical straight-line heading retention remain
  separate hardware acceptance evidence.
