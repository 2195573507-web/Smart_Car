# Smart_Car Module Index

| Module | Canonical doc | Source root | Primary entry |
| --- | --- | --- | --- |
| STM32H757 | [stm32h757.md](docs/stm32/stm32h757.md) | `STM32H757/` | `CM7/Core/Src/main.c::main` |
| ESP32-S3 | [esp32-s3.md](docs/esp32s3/esp32-s3.md) | `ESPS3/` | `main/main.c::app_main` |
| BLE | [ble.md](docs/esp32s3/ble.md) | `ESPS3/components/s3_ble`, App `BLE/` | `s3_ble_init`, `BLEManager` |
| UART | [uart.md](docs/protocol/uart.md) | STM `UART_Link`, S3 `stm_uart` | `uart_link_task`, `stm_uart_task` |
| Protocol | [protocol.md](docs/protocol/protocol.md) | STM/S3 `sc_frame`, App model | `sc_frame_encode`, `SmartCarProtocol.Parser.feed` |
| IMU | [imu-pipeline.md](docs/imu/imu-pipeline.md) | `STM32H757/Middleware/Sensor` | `imu_init`, `imu_update` |
| LSM303 | [lsm303.md](docs/imu/lsm303.md) | `STM32H757/Drivers/IMU/LSM303` | `lsm303_init`, `lsm303_read_acc/mag` |
| BMI323 | [bmi323.md](docs/imu/bmi323.md) | `STM32H757/Drivers/IMU/BMI323` | `bmi323_init_diag` |
| Calibration | [calibration.md](docs/imu/calibration.md) | `STM32H757/Middleware/Calibration` | `imu_boot_manager_step` |
| Attitude | [attitude.md](docs/imu/attitude.md) | `STM32H757/Middleware/Attitude` | `attitude_update` |
| Filter | [filter.md](docs/imu/filter.md) | `STM32H757/Middleware/Filter` | `imu_filter_update` |
| Radar | [radar.md](docs/radar/radar.md) | `ESPS3/main/radar`, `radar_control` | `radar_uart_init`, `radar_parser_feed` |
| Motor | [motor.md](docs/motor/motor.md) | `STM32H757/Drivers/Motor` | `motor.c` placeholder API |
| Encoder | [encoder.md](docs/motor/encoder.md) | `STM32H757/Drivers/Encoder` | `encoder.c` placeholder API |
| Logger | [logger.md](docs/debug/logger.md) | `Tools/SmartCar_Logger_MAC` | `LoggerSession`, `SmartCarLogParser` |
| Boot | [boot.md](docs/stm32/boot.md) | `STM32H757/Middleware/Boot` | `boot_log_start`, `boot_log` |
| FreeRTOS | [freertos.md](docs/stm32/freertos.md) | `STM32H757/Middleware/FreeRTOS`, task code | `vTaskStartScheduler` |
| App | [mac-control-app.md](docs/app/mac-control-app.md) | `IOS_APP/SmartCar_Control_MAC` | `BLEManager`, `SmartCarViewModel` |
| ROS2 | [ros2.md](docs/ros2/ros2.md) | `ROS2_WIN`, `DOCS/ROS2*` | No current runtime entry |
