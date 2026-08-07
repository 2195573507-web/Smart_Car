# Filter Middleware

The Filter layer sits between `Middleware/Sensor` and the estimation layers.
It accepts an `imu_calibrated_data_t` snapshot and exposes a filtered
accelerometer/magnetometer snapshot to Attitude or Odometry. The
implementation applies a five-sample median stage followed by a 0.95 IIR low
pass to both acceleration and magnetometer values. The IMU manager gates
updates until calibration reaches `COMPLETE`; the first accepted calibrated
sample seeds the filter to avoid a startup spike.

`imu_filter.c` owns no RTOS task and does not access HAL, BSP, or IMU drivers.
The input/output structures retain the Sensor timestamp and online flag so a
future filter can preserve sample provenance. The update and snapshot APIs use
the existing FreeRTOS mutex boundary when the CM7 target is built with
FreeRTOS.
