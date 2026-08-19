# Dual IMU Calibration

`imu_boot_manager` is the single `DUAL_IMU_BOOT` lifecycle authority. The
legacy `imu_boot_state_t` remains a read-only compatibility view for existing
calibration status, logging, and filter gating; it cannot advance the new
lifecycle.

```text
IDLE -> INIT -> SELF_TEST -> STATIC_CALIBRATION -> VIBRATION_CAPTURE
     -> VERIFY -> READY
                    \-> FAILED
```

`INIT` creates LSM303/I2C4 and BMI323/SPI1 worker tasks, then releases both
through one FreeRTOS task-notification gate. The two drivers are not started
sequentially by the lifecycle. `SELF_TEST` is a shared valid-data observation
window, not a sensor on-chip self-test command.

`STATIC_CALIBRATION` retains PWM=0 until the existing 2000 ms settle delay,
then opens one 10,000 ms shared window. LSM303 acceleration and BMI323 accel
and gyro data are accumulated independently. At the time boundary both must
meet the 90% sampling-quality floor before their biases are accepted. For each
stream, the floor is
`ceil(configured_rate_hz * window_duration_s * 0.90)`. LSM uses 100 Hz and BMI
uses its active 100, 200, 400, or 800 Hz ODR. `IMU_CAL` reports actual LSM,
BMI-accel, and BMI-gyro counts plus the minimum and quality bits at closure.

`VIBRATION_CAPTURE` uses PWM levels 20, 40, 60, 80, and 100. After the
per-level 2000 ms settle delay, each level opens one 10,000 ms common
microsecond interval. Only samples timestamped inside that interval enter the
LSM303 and BMI323 datasets. The phase cannot advance until both datasets have
completed their shared window and meet the same per-source 90% floor. Each
dataset retains captured attempts, invalid samples, actual rate, and
`quality_ok`; `IMU_VIB` logs those values at the boundary.

All acquisition timestamps and capture windows use `imu_time` DWT-derived
`timestamp_us`. Legacy millisecond views are derived from it. HAL tick is not
used for BMI323 waits, sample timestamps, or calibration windows.

BMI capture statistics expose configured rate, measured rate, and successful
sample count for 100/200/400/800 Hz. The IMU debug task also logs FreeRTOS free
heap and the `imu_task`/debug/BMI stack high-water marks every five seconds.

`VERIFY` independently rechecks the retained LSM303 and BMI323 calibration
and final vibration results. It sends `CAL_EVENT_COMPLETE` only after both
local result flags pass, retries the correlated ACK a bounded number of times,
and accepts the local results for `READY` if that final notification ACK is
lost. A missing local result still fails VERIFY.

`dual_imu_manager_t` retains the phase, per-sensor and overall progress,
current phase timestamps, per-phase start/end timestamps, and error state.
`SCBP_MSG_ID_DUAL_IMU_STATUS` (`0x0208`) transports those current values to
the S3 relay and macOS App.

The existing LSM303 calibration/filter/attitude path stays intact. BMI323 data
does not enter `attitude.c`, AHRS, fusion, or attitude parsing.

No source/build evidence establishes physical sensor behavior, PWM response,
STM-S3 UART delivery, BLE notification, or App rendering.
