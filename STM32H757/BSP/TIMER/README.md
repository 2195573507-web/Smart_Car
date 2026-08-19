# Timer BSP

Board-support boundary for periodic timing and timer peripherals. The BSP
provides DWT-backed microseconds and legacy HAL-tick milliseconds; scheduling
policy belongs to System and timing consumers belong to higher layers.

IMU acquisition must use `imu_time.h`: `imu_time_init()`,
`imu_time_now_us()`, and `imu_time_now_ms()`. Its microsecond source is the
DWT counter extended to 64 bits with wrap accounting protected from concurrent
LSM303 and BMI323 task reads. The millisecond helper is derived from that same
microsecond value. `bsp_timer_get_ms()` remains available only for legacy
non-IMU users; it must not be used as an IMU sample or capture-window
timestamp.
