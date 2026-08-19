# Attitude Middleware

The Attitude layer consumes filtered IMU data and publishes the vehicle
orientation state for higher-level consumers. Its public state contains roll,
pitch, yaw, and a quaternion (`w`, `x`, `y`, `z`).

The implementation is LSM303 ACC + MAG attitude parsing. After accelerometer
calibration, `attitude_update()` collects 500 stationary samples for a startup
zero, then consumes only `imu_filter_get_output()` and publishes internal
radians using:

```text
roll  = atan2(ay, az)
pitch = atan2(-ax, sqrt(ay * ay + az * az))
yaw   = atan2(my, mx)
```

The module owns no RTOS task and does not access HAL, BSP, IMU drivers, the
manager, calibration, or Application modules. Roll and pitch use a 0.9/0.1
low pass; yaw uses a wrapped 0.95/0.05 low pass. Quaternion estimation remains
outside this phase, so the existing quaternion fields stay zero-initialized.

The public Euler state is radian-only. The communication sender publishes
`roll_rad`, `pitch_rad`, `yaw_rad` and derived `roll_deg`, `pitch_deg`,
`yaw_deg` fields plus `timestamp_ms`, `source`, and `valid` in the 30-byte
ATTITUDE payload; the Attitude middleware does not perform wire-unit
conversion.
