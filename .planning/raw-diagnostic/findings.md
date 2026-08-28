# Findings

- `bsp_uart_log_write_level()` uses a FreeRTOS mutex and HAL transmit; it is
  unsuitable inside a fault handler or as proof that the scheduler is alive.
- `HardFault_Handler`, `MemManage_Handler`, `BusFault_Handler`, and
  `UsageFault_Handler` already funnel into retained-register capture.
- FreeRTOS malloc-failed and stack-overflow hooks already retain state and halt,
  but do not emit an immediate USART1 marker.
- `s3_service_task`, `log_service_task`, `attitude_startup_coordinator_task`,
  `chassis_runtime_task`, and `uart_link_task` all run periodic loops; raw
  diagnostics must be first-entry plus rate-limited loop markers.
- `uart_link_send()` takes `s_tx_mutex`, performs blocking HAL TX, and clears
  `s_tx_active` before releasing the mutex. Instrument both lock and HAL
  boundaries without changing this ownership model.
- The current raw diagnostic build is dependency-free at the output boundary:
  it writes USART1 registers directly and rate-limits repeated markers. The
  production/default build keeps the calls compiled as no-ops and retains the
  original stop behavior for asserts and unhandled interrupts.
- A hardware result must be interpreted by marker order: `BEFORE_DELAY` without
  `AFTER_DELAY` points to the tick/PendSV path; `HAL_BEGIN` without `HAL_DONE`
  points to the blocking UART2 transmit; malloc/assert/default-handler markers
  identify their own failure hooks. These are diagnostic branches, not source
  proof of the physical root cause.
