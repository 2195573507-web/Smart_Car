# Motor Module

## Function

Provide the STM32 motor-driver ownership boundary for PWM and direction outputs.

## Source Location

`STM32H757/Drivers/Motor/motor.c/.h`, BSP PWM/GPIO, generated TIM3 setup.

## Entry File

`motor.c` and `bsp_pwm.c`; current `motor.h` is a placeholder interface.

## Inputs

Application motion commands and validated safety state.

## Outputs

TIM3 CH1..CH4 PWM on PC6..PC9 and direction GPIOs PC5, PC1, PB14, PB15 when a
future implementation owns them.

## Public Interfaces

Current public motor driver header is intentionally a placeholder; use the BSP
PWM/GPIO boundary only within an explicitly authorized motion task.

## Dependencies

BSP PWM/GPIO, generated TIM3, motion/chassis/safety layers.

## Current Status

Hardware resource allocation is documented; current driver API is a placeholder
and vehicle behavior is not implemented/accepted here.

## Known Issues

Do not infer motor operation from timer initialization or PWM readiness.

## Modification Notes

Respect safety ownership and frozen IOC allocation. Changes require an explicit
motion/safety task, not a documentation-only update.
