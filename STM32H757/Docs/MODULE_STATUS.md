# Module Status

| Module | Status | Scope |
| --- | --- | --- |
| BSP | IMPLEMENTED | HAL-wrapping GPIO, SPI1, I2C4, UART reservation, TIM3 PWM, DWT/HAL timer, and ADC framework |
| IMU drivers | SCAFFOLD | BMI323 and LSM303 file boundaries only |
| Motor driver | SCAFFOLD | Four-channel motor boundary only |
| Encoder driver | SCAFFOLD | Encoder acquisition boundary only |
| S3 link | INTERFACE | Command/state types and deferred entry points |
| Filter | PLANNED | README and dependency boundary |
| Attitude | PLANNED | README and dependency boundary |
| Odometry | PLANNED | README and dependency boundary |
| Control | PLANNED | README and dependency boundary |
| Application | FRAMEWORK | Chassis, Motion, Remote, Safety READMEs |
| System | FRAMEWORK | Task, Logger, Watchdog, Memory READMEs |
| CM7 | BASELINE | Existing CubeMX/CMake build target |
| CM4 | BASELINE | Existing CubeMX/CMake build target |
| ESP32-S3 | EXTERNAL | Lidar, Wi-Fi, App, ROS2 gateway remain outside this tree |
