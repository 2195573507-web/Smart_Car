# Findings

- `main.c` currently withholds `attitude_startup_coordinator_start()` and
  `chassis_runtime_start()` when `imu_runtime_is_start_ok()` is false.
- `chassis_runtime_task()` already holds PWM at zero while
  `g_attitude_is_ready == 0` and publishes chassis/wheel status after SRP sync.
- `s3_service_task()` already emits a synchronization-gated 50 ms
  `IMU_CAL_STATUS` fallback independently of the IMU worker.
- `imu_update()` is safe before sensor initialization: it returns
  `BSP_STATUS_NOT_READY` when unprepared and `BSP_STATUS_OK` while the dual
  initialization workers are pending. Therefore the business tasks can run
  without unlocking motor output.
- Early log delivery through SRP remains sync-gated; USART1 boot logs are the
  source-of-truth before the S3 session is established.
