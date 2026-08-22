# Task Plan: Wheel Closed-Loop Polarity and App Heartbeat

## Goal

Remove the M2 RF positive-feedback polarity inversion and keep nonzero wheel
targets alive across the STM32 1000 ms command watchdog.

## Scope

- [x] Normalize MotorBoard PID feedback/output sign handling.
- [x] Add watchdog PID reset and forced zero PWM stop.
- [x] Add 100 ms App wheel heartbeat and immediate zero-stop behavior.
- [x] Build CM7 and macOS App with zero diagnostics.
- [x] Record source/build evidence separately from hardware acceptance.

## Protected Boundaries

Preserve SCBP/App frame formats, PID parameters, GPIO/IOC ownership, and the
existing S3 1000 ms safety timeout.
