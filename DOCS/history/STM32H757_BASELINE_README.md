# Smart Car H757 Baseline

This directory contains a no-application STM32H757XIH6 configuration baseline.
The `CM7` and `CM4` CMake targets validate the project skeleton only; they do
not compile or generate firmware.

The IOC uses SPI1 for BMI323, I2C4 for LSM303, and USART2 for GPS. GPIO labels
describe every supplied motor, encoder, and communication connection.

## Hardware Constraints Recorded in the IOC

- `PD3` has no UART TX alternate function and `PD4` has no UART RX alternate
  function on STM32H757XIH6. They stay as named GPIO for the requested STM
  connector; no UART1 instance is enabled.
- `PA13` is used as `LF_INT2` by the supplied wiring, so SWD is disabled in
  the IOC. Restore SWD only after moving that encoder connection.
- The connector list supplies no PWM/enable net. TIM6 is enabled only as an
  internal base timer; it has no output pin and is not a physical PWM channel.

