# Dual IMU Calibration

`imu_boot_manager` is the STM32 lifecycle authority:

```text
IDLE -> INIT -> SELF_TEST -> STATIC_CALIBRATION -> READY
                                           \\-> FAILED
```

`INIT` releases the existing LSM303/I2C4 and BMI323/SPI1 workers. The manager
observes samples in its configured time windows and preserves each sensor's
timestamps and diagnostic sample counts. LSM303 remains the calibrated
filter/AHRS source; BMI323 remains independently acquired and exported through
source-tagged telemetry.

Before the static window, STM sends transactional SCBP-CAN
`BOOT_READY(0x007)`. S3 sets calibration PWM to zero, then sends transactional
`RADAR_PWM_READY(0x302, speed_percent=0)`. Static calibration never releases
the zero-PWM gate before that message is admitted. When the window completes,
STM emits `CAL_EVENT(0x001, STATIC_DONE=1)` transactionally and S3 releases
its calibration lock after admission.

The only lifecycle status payload on UART2 is 11-byte
`IMU_CAL_STATUS(0x202)`. Runtime output additionally consists of source-tagged
30-byte `IMU_TELEMETRY(0x207)` payloads and 80-byte schema-2
`ATTITUDE(0x201)` payloads. There is no active `0x0208` dual-status payload,
legacy 30-byte attitude payload, calibration bias/result transport, or V3
adapter frame.

Builds and logs do not prove physical sensor behavior, radar PWM response,
STM-S3 UART transfer, BLE notification, or App rendering.
