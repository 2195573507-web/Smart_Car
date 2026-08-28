# Findings

- The captured CM7 LR `0x08026E17` is the return address after
  `vTaskSwitchContext` in `xPortPendSVHandler`; the bad PC is inside FreeRTOS
  `ucHeap`, so the first fault boundary is saved task context/heap corruption.
- The active wire protocol is SRP v4. Header byte 6 is the message type.
- Current STM motor output is an external MotorBoard over USART6. TIM1/TIM2 are
  encoder timers and PC6/PC7 belong to USART6, so no independent PWM-enable
  register may be invented for the fault path.
- Existing heading controller already exposes `chassis_heading_control_reset()`
  and clears its integral state.
- SRP v4 uses `CMD_SYNC_REQ` type `0x08` with payload `{4,0,0,0}` and STM32
  responds with `RSP_BOOT_INFO` type `0x09`, echoing the request sequence in
  payload byte 4.
- The original S3 dispatch did not include `RSP_BOOT_INFO`, so a valid STM32
  response was silently ignored and the gateway stayed in `WAITING`.
- UART2 source configuration is identical at both endpoints: `921600 8N1`,
  parity disabled, hardware flow control disabled. S3 uses GPIO17/18 and
  STM32 uses USART2 PA2/PA3. S3 radar UART1 keeps RX on GPIO44; the isolated
  build confirms USB Serial/JTAG console and `UART_NUM=-1`.
- Host SRP codec test, isolated ESP-IDF build, and CM7 Debug build pass after
  the callback null-handle guard. Flashing and live UART/logic-analyzer
  capture remain unverified.
