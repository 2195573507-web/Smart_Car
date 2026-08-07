# IMU Calibration

This layer receives complete `imu_raw_data_t` snapshots from the IMU manager
and publishes `imu_calibrated_data_t` snapshots to the filter.

The single runtime state is `imu_cal_state_t`: `IDLE`, `SET_PWM`,
`WAIT_STABLE`, `SAMPLE`, `COMPLETE`, or `ERROR`. Sampling is represented by
`sample_mode`, not by a second state machine. Static sampling uses PWM 0 and
collects `IMU_CALIBRATION_ACCEL_SAMPLES` (5000) accelerometer samples at 100 Hz
after a 2000 ms zero-PWM stabilization wait before computing
the accelerometer bias and noise. Vibration sampling uses PWM levels 20, 40,
60, 80, and 100, with a 2000 ms local stabilization wait before each 1000
sample window.

Static sampling uses streaming accumulators and excludes magnetometer samples.
Z bias is the mean minus `9.80665 m/s^2`.

Magnetometer hard-iron calibration collects per-axis min/max values for the
bounded calibration window and computes `(max + min) / 2` for each bias.

The calibration layer logs `SAMPLE`, `SET_PWM`, `WAIT_STABLE`, and `COMPLETE`.
The manager does not pass samples to the filter until the state reaches
`COMPLETE`. `PWM_APPLIED` is an S3 LEDC duty-update confirmation; it is not
radar measurement feedback.
