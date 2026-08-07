# Encoder Module

## Function

Own wheel encoder acquisition and count interpretation when implemented.

## Source Location

`STM32H757/Drivers/Encoder/encoder.c/.h`, generated TIM1/TIM2 setup, and the
hardware audit in `DOCS/STM32H757/STM32_RESOURCE_AUDIT.md`.

## Entry File

`encoder.c`; current public header is a placeholder.

## Inputs

RF encoder PA8/PA9 and RB encoder PA15/PB3 timer inputs; LF/LB frozen inputs are
not valid TI1/TI2 pairs for timer encoder mode.

## Outputs

Future counts, direction, and odometry inputs.

## Public Interfaces

Current encoder public header is intentionally a placeholder; no count API is
established by current source.

## Dependencies

TIM1/TIM2 HAL, motion/odometry, safety.

## Current Status

Static allocation only. RF/RB timer pairs exist in IOC/source; LF/LB full
timer-mode acquisition is not available on the frozen nets.

## Known Issues

No current source proof of count API, polarity, filtering, or odometry output.

## Modification Notes

Do not invent a timer mapping or repurpose SWD. Hardware/net decisions require
explicit authorization.
