# BSP Interface

The board-support package is the only layer below device drivers. Application
and Middleware code must include the `BSP/<module>/bsp_<module>.h` header and
must not call `HAL_*` directly.

## Status and ownership

All interfaces return `bsp_status_t` where an operation can fail. `OK`,
`INVALID_ARG`, `NOT_READY`, `TIMEOUT`, `UNSUPPORTED`, and `ERROR` are the
stable status values. The public headers do not expose HAL handle types.

| Module | Public operations | Current hardware ownership |
| --- | --- | --- |
| GPIO | `gpio_init`, `gpio_write`, `gpio_read`, `gpio_toggle` | CS, direction outputs, and named interrupt inputs |
| SPI | `spi_init`, `spi_transmit`, `spi_receive`, `spi_write_read` | SPI1, PA5/PA6/PA7, software CS on PC4 |
| I2C | `i2c_init`, `i2c_write`, `i2c_read`, `i2c_write_read`, `i2c_probe` | I2C4, PD12/PD13; 7-bit addresses are converted internally |
| UART | blocking TX/RX, DMA TX/RX reservation, `uart_log_write` | Current generated transport is USART2; USART1/USART6 return `UNSUPPORTED` until IOC resources exist |
| PWM | `pwm_start`, `pwm_stop`, `pwm_set_duty` | TIM3 CH1-CH4 on PC6-PC9; duty is 0-100 percent |
| TIMER | `timer_get_us`, `timer_get_ms` | DWT cycle counter for microseconds and HAL tick for milliseconds |
| ADC | `adc_init`, `adc_read` | Framework only; no ADC instance or battery channel is configured |

`bsp_spi_init`, `bsp_i2c_init`, and `bsp_pwm_init` must be called after the
CubeMX-generated system and peripheral initialization has completed. The
short names in each header are inline aliases for the `bsp_` functions.

The `BSP_GPIO_LF_INT2` name maps to PA13 for documentation and readback, but
`gpio_init` deliberately leaves PA13 in the IOC's Serial-Wire mode. It cannot
serve as a live interrupt input while SWD is retained.

## Hardware validation boundary

`CM7/BSP_TEST` is a compile-only API smoke test. It proves that a future
driver can include every BSP header and that the transaction signatures are
linkable. It does not prove SPI loopback, an I2C device scan, UART output, or a
PWM waveform. Those checks require a connected board and a separately recorded
runtime test.
