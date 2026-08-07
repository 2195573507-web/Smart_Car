# STM32H757 Controller

This directory is the vehicle-side STM32H757 controller project. It owns
deterministic sensing, local safety, and final motion authority; it does not
establish a completed vehicle or transport acceptance by itself.

## Start Here

- [Current STM32 module boundary](../docs/stm32/stm32h757.md)
- [Source/IOC-visible hardware facts](../docs/hardware/facts.md)
- [UART ownership and transport boundary](../docs/protocol/uart.md)
- [Code map](../docs/code_map.md)

## Source Areas

| Area | Purpose |
| --- | --- |
| `CM7/` | Primary application core, generated startup, and build entry |
| `CM4/` | Secondary-core project material |
| `BSP/`, `Drivers/` | Board and device abstraction layers |
| `Middleware/` | Sensors, calibration, filtering, attitude, communication, and RTOS services |
| `Application/`, `System/` | Application tasks and shared system services |
| `Config/`, `Docs/` | Project-local configuration and detailed reference material |

## Modification Boundary

Read the canonical STM32 page and the relevant public header/task before any
change. Preserve generated HAL/IOC boundaries, frozen pin and timer allocations,
and local safety authority. Build, source inspection, and text logs are not
evidence of physical UART, sensor, motor, or vehicle behavior.

## History

The prior root README is retained as a dated baseline in
[docs/history/STM32H757_BASELINE_README.md](../docs/history/STM32H757_BASELINE_README.md).
It is historical reference, not the current module entry point.
