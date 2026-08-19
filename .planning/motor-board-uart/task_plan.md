# MotorBoard UART Task Plan

## Goal

Add the first-stage STM32H757 MotorBoard UART transport on PC6/PC7 without
inventing the board protocol or changing protected system paths.

## Phases

- [x] Explore project context, current UART ownership, and TIM3 references
- [x] Agree on the USART6 PC6/PC7 design and protocol-missing boundary
- [ ] Write and self-review protocol analysis report
- [ ] Implement UART, protocol hooks, public module, and raw test task
- [ ] Retire active TIM3 PWM output path while preserving history
- [ ] Run static checks and CM7 Debug clean build
- [ ] Write integration report and final evidence summary

## Status

Design approved; implementation is gated on written-spec review per the
brainstorming workflow.
