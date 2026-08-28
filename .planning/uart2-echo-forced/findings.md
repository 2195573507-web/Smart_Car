# Findings

- The CM7 test route is USART2 PA2/PA3 and the S3 test route is UART2
  GPIO17/GPIO18, both at 115200-8-N-1.
- The prior test branch skipped service startup but still compiled SRP,
  `uart_link`, motor transport, and DMA-related production source files.
- The S3 `main` component can be reduced to `freertos`, `log`,
  `esp_driver_uart`, and `esp_timer`; no project service component is needed.
- `sdkconfig.defaults` has no `SMARTCAR_UART2_ECHO_TEST` Kconfig entry. A C
  preprocessor/CMake force is required; adding a nonexistent Kconfig symbol
  would not configure the build.
