# BSP Driver Guide

Device drivers depend on BSP interfaces, never on CubeMX handles or HAL calls.
The driver sequence is:

1. Let the CM7 startup path finish its generated clock and peripheral setup.
2. Call the relevant `bsp_*_init` function and check its `bsp_status_t` result.
3. For BMI323, assert `BSP_GPIO_BMI323_CS` low, call `spi_write_read` on SPI1,
   then deassert CS. The SPI bus is configured for 8-bit, mode 0 operation.
4. For LSM303, use the I2C4 helpers with a 7-bit device address. Register
   reads use `i2c_write_read`, which performs a transmit followed by a receive.
5. Use `timer_get_us` for elapsed intervals and `timer_get_ms` for coarse
   scheduling. Do not read DWT or HAL tick registers from a driver.

`CM7/BSP_TEST/bsp_test.h` exposes optional board-test entry points for SPI
loopback, I2C address scanning, UART output, and PWM output. They are not
called from `main`; invoke them only from an authorized hardware test harness.

## UART and unavailable resources

The API carries `BSP_UART_USART1` and `BSP_UART_USART6` so a communication
driver does not need to change when the board mapping is enabled. The current
IOC/generated CM7 source only owns USART2 on PA2/PA3; USART1/USART6 operations
and DMA operations therefore return `BSP_STATUS_UNSUPPORTED`. Logging uses a
separate `uart_log_write` entry point and does not replace `printf`.

ADC calls are intentionally framework-only and return `BSP_STATUS_UNSUPPORTED`
until an ADC instance and channel are assigned in the IOC. No BMI323, LSM303,
attitude, motor-control, or ROS implementation belongs in this layer.

## Error handling

Drivers must propagate non-`BSP_STATUS_OK` results. Retry policy, sensor reset,
and bus recovery belong to the device-driver or system owner, not to the low-
level transaction wrappers.
