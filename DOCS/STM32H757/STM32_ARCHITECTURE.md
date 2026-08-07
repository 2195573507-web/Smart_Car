# STM32H757 Base Architecture

## Baseline

| Item | Required baseline |
| --- | --- |
| MCU | `STM32H757XIH6` |
| Project root | `STM32H757/` |
| IOC | `STM32H757/Smart_Car_H757.ioc` |
| CM7 target clock | 480 MHz |
| CM4 target clock | 240 MHz |
| Base resources | GPIO, BMI323 SPI, LSM303 I2C, GPS UART, and timer PWM reservation; STM communication UART is pending a valid TX/RX route |

## Directory Intent

```text
STM32H757/
├── Core/       CubeMX/common core startup and HAL integration artifacts
├── Drivers/    STM32Cube-provided driver layer; no application algorithms in this task
├── CM7/        CM7-target-specific generated/build artifacts
├── CM4/        CM4-target-specific generated/build artifacts
├── Docs/       STM32-local reference material
└── Smart_Car_H757.ioc
```

The exact generated subdirectory layout is controlled by CubeMX/project-generator output. The listed structure is the required engineering boundary, not a claim that generated sources have already been exercised on target hardware.

## Peripheral Ownership

| Resource | Intended device/interface | Initialization action |
| --- | --- | --- |
| SPI | BMI323 | Configure the connected bus pins and chip-select/interruption GPIO; no sensor driver or register transaction |
| I2C | LSM303 | Configure the connected bus pins; no sensor driver or bus transaction |
| UART | STM32-to-ESP32 communication | Do not configure on `PD3`/`PD4`: these pins cannot form a UART TX/RX pair on this MCU. A hardware-valid replacement route is required; protocol is deferred. |
| UART | GPS | Configure the connected UART route; NMEA/other parsing deferred |
| Timers | Motor PWM reservation | No PWM output pin was supplied, so do not enable an unconnected PWM channel. Later reserve only a connected hardware-valid resource; no duty-cycle/control logic. |
| GPIO | Motor direction and encoder nets | Configure electrical direction/input as specified; no motor/encoder processing |

## Dual-Core Boundary

The configured clock targets define the CM7/CM4 baseline only. This task does not assign run-time ownership, inter-core communication, RTOS tasks, interrupts, cache policy, shared memory, or startup sequencing beyond what CubeMX needs to create a parseable base project.

## UART Log Architecture

CM7 diagnostic output uses USART1 through `bsp_uart_log_write()`. The BSP owns a
FreeRTOS mutex around every blocking `HAL_UART_Transmit()` call, so task-context
writers cannot interleave a log block. The caller-supplied 100 ms timeout is a
total budget: time spent waiting for the mutex is deducted before the remaining
budget is passed to HAL.

`bsp_uart_get_log_stats()` reports RAM-only log counters: `tx_count` is the
number of successful log blocks, `tx_fail` is the number of failed blocks, and
`tx_busy` is the subset that failed because the mutex or UART was busy. The IMU
runtime separately exposes `imu_runtime_get_log_fail_count()` for failed calls
made by its debug logger. These counters make loss observable; they do not
retry, persist, or write to Flash.

This synchronization does not add, resize, or otherwise alter any Ring Buffer
or its 500-line design. The API is for task context; ISR log output remains
outside this blocking mutex/HAL transmit path.

## Configuration Blockers

- The requested STM communication nets `PD3` and `PD4` are alternate functions for USART2 flow-control/direction signals, not a UART TX/RX pair. They cannot meet the requested UART communication link without a hardware-net change or an explicitly authorized interface change.
- `PA13` is the default SWDIO debug pin. Assigning it to `LF_INT2` removes SWD access unless debugging is moved or disabled by an explicit hardware/debug decision.

## Validation Boundary

An IOC that opens and a project that compiles demonstrate configuration/build status only. They do not demonstrate clock measurement, CM4 boot, GPIO voltage levels, PWM waveform, sensor bus traffic, UART traffic, motor operation, or encoder input operation.
