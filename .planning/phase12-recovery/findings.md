# Phase 1-2 Recovery Findings

- Active STM32-S3 transport is Common/SRP SRP v4; the CM7 service already gates
  motion before sync and has ACK/retry and BUS_OFF recovery paths.
- `STM32H757/Middleware/Communication/Services/s3_service.c` now uses a
  200 ms post-sync S3-frame safety timeout and performs wheel-command clear,
  heading reset, host-state demotion, and `motor_board_force_stop()`.
- Existing `SRP_S3` diagnostics include sync counters, parser errors, REC/TEC,
  UART RX/TX, and stack data but omit timeout/BUS_OFF/force-stop result totals
  and the last timeout snapshot.
- Existing IMU source assigns BMI323 to the DualAHRS primary path and LSM303 to
  the redundant path; static-calibration and 80-byte schema-2 attitude paths
  are already present and are not to be redesigned in this task.
- The prior telemetry path was owned by `imu_debug_task` and gated by
  `s3_service_is_synced()`. `imu_runtime_start()` can return before creating
  that task when sensor initialization or task creation fails, so a transport
  task-owned fallback is required for continuous link evidence.
- `s3_service_task` is created from `CM7/Core/Src/main.c` before the scheduler
  starts and already services the UART ring every 1 ms. The repair keeps that
  RX cadence and adds a 50 ms `IMU_CAL_STATUS` send deadline rather than
  blocking on calibration or replacing the RX service loop with a slower tick.
- STM already rearmed `HAL_UARTEx_ReceiveToIdle_DMA` in both
  `HAL_UARTEx_RxEventCallback` and `HAL_UART_ErrorCallback`; the source path
  includes fallback `s_restart_requested` recovery via `uart_link_task`.
- The final source audit found and corrected one compile-risk in the new
  telemetry startup diagnostic: `LOG_INFO` accepts one string argument in this
  tree, so the formatted message is now built with `snprintf` first.
- The current CM7 startup order is transport first (`uart_link_task_start`,
  `s3_service_start`), then optional IMU runtime creation. This reduces the
  chance that the 32 KB FreeRTOS heap prevents the SRP task from existing.
- The periodic fallback is intentionally `IMU_CAL_STATUS` (11-byte legal SRP
  payload) at 50 ms while synchronized. It is a transport heartbeat and does
  not replace the calibration authority or claim live BMI323 data.
