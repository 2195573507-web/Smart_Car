# Smart_Car Code Map

This map points Codex from a module to its source root, entry function, tasks,
and public interfaces. Paths are source paths, not claims of runtime success.

| Module | Source path | Entry/task | Key functions/interfaces | Output |
| --- | --- | --- | --- | --- |
| STM32 boot | `STM32H757/CM7/Core/Src/main.c` | `main` | `HAL_Init`, peripheral init, `vTaskStartScheduler` | Startup and task graph |
| STM32 UART log | `STM32H757/BSP/UART`, `Middleware/Boot` | `bsp_uart_init`, `boot_log` | `bsp_uart_transmit`, `log_service_start` | USART1 text/structured logs |
| STM-S3 UART | `STM32H757/Middleware/Communication/UART_Link` | `uart_link_task_start` | `uart_link_send/read/get_stats` | Raw STM-S3 bytes |
| Shared SRP v4 | `Common/SRP` | parser/link callbacks | `srp_encode/decode`, `srp_parser_feed`, `srp_link_send/receive/tick` | Shared validated UART frame views and link health |
| STM S3 service | `STM32H757/Middleware/Communication/Services/s3_service.c` | `s3_service_start` | `s3_service_step`, `s3_service_send_boot_message` | SRP calibration/telemetry/config service events |
| IMU runtime | `STM32H757/Application/RTOS/imu_runtime.c` | `imu_runtime_start` | `imu_task`, `imu_debug_task` | Sample cadence and logs |
| IMU manager | `STM32H757/Middleware/Sensor/imu_manager.c` | `imu_init`, `imu_update` | `imu_get_data`, `imu_lsm_is_online` | Complete raw snapshot |
| LSM303 | `STM32H757/Drivers/IMU/LSM303` | `lsm303_init` | `lsm303_read_acc/mag` | Accel/mag vectors |
| BMI323 | `STM32H757/Drivers/IMU/BMI323` | `bmi323_init_diag` | `bmi323_read_acc/gyro/temp` | Paused sensor diagnostics |
| Calibration | `STM32H757/Middleware/Calibration` | `imu_boot_manager_step` | `imu_boot_manager_update`, ACK/event handlers | Readiness and bias |
| Filter | `STM32H757/Middleware/Filter` | `imu_filter_update` | `imu_filter_get_output`, readiness | Filtered IMU |
| Attitude | `STM32H757/Middleware/Attitude` | `attitude_update` | `attitude_get_state/status` | Euler/quaternion state |
| Motor | `STM32H757/Drivers/Motor`, `BSP/PWM` | placeholder `motor.c` | TIM3 PWM/BSP interfaces | Future actuator output |
| Encoder | `STM32H757/Drivers/Encoder`, generated TIM1/TIM2 | placeholder `encoder.c` | timer allocation only | Future counts/odometry |
| S3 main | `ESPS3/main/main.c` | `app_main` | NVS, STM UART, BLE, radar, service init | Gateway startup |
| S3 STM UART | `ESPS3/components/stm_uart` | `stm_uart_init` | `stm_uart_send/receive_nonblock` | Raw gateway bytes |
| S3 protocol | `ESPS3/components/smartcar_protocol` | shared parser callbacks | `srp_parser_feed`, `srp_link_send/receive` | SRP source frame views |
| S3 service | `ESPS3/components/smartcar_service` | `smartcar_service_init` | command/radar/log bridges | Gateway state/events |
| S3 BLE | `ESPS3/components/s3_ble` | `s3_ble_init` | GATT callbacks, notify, RX callback API | BLE transport |
| Radar | `ESPS3/main/radar`, `components/radar_control` | `radar_uart_init` | `radar_parser_feed`, `radar_control_set_calibration_pwm` | Radar bytes/state/PWM |
| Shared App protocol | `Shared/SmartCarAppCore/Sources/SmartCarAppCore` | `AppBLEFrameParser`, `AppBLESession` | V1/V2 encode, CRC, session, scheduler, bounded buffers | Shared App frame/session model |
| App protocol aliases | `IOS-APP/Sources/SmartCarIOS/Core/Model`, `SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/Model` | `SmartCarProtocol` typealias | Source-compatible target names | Platform model compatibility |
| App BLE | `IOS-APP/Sources/SmartCarIOS/Core/BLE` | `BLEManager` | scan/connect/send/receive pipelines | User transport state |
| App state/UI | `IOS-APP/Sources/SmartCarIOS/Core/{Stores,ViewModels}`, `UI` | `SmartCarViewModel` | telemetry/log stores, control views | Operator/developer UI |
| macOS app protocol/BLE | `SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/{Model,BLE}` | `SmartCarProtocol`, `BLEManager` | Same App-BLE framing, mode commands, serial write queue | macOS transport state |
| macOS app state/UI | `SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/{Stores,ViewModels,UI}` | `SmartCarViewModel` | Same chassis/independent control and telemetry model | macOS operator/developer UI |
| Logger | `Tools/SmartCar_Logger_MAC/Sources` | `LoggerSession` | `SerialPortService`, `SmartCarLogParser` | Receive-only log viewer |
| ROS2 | `ROS2_WIN`, `DOCS/ROS2*` | no current runtime entry | future driver/bridge/SLAM APIs | Planned autonomy domain |

## Navigation Rule

Read the public header and owning task before touching an implementation. For
transport work, inspect both raw transport and parser/service consumers. For
calibration or safety work, trace state transitions and ACK/error callbacks all
the way to the owner.
