# Task Plan: DualAHRS LSM303 Yaw Alignment

## Goal

Keep BMI323 as the schema-2 primary estimator and LSM303 as the redundant
estimator. Apply the approved -105.0 degree LSM303 installation-yaw correction
without changing the 80-byte telemetry contract or S3/macOS field mappings.

## Phases

- [x] Phase 1: Inspect live estimator, lifecycle, transport, and protocol paths.
- [x] Phase 2: Confirm architecture decision and define the bounded correction.
- [x] Phase 3: Apply STM32 source changes and source-level protocol checks.
- [x] Phase 4: Build CM7 and ESP32-S3, then report device validation separately.

## Evidence Boundary

- Build and source checks do not prove sensor heading, UART traffic, or PWM state.
- No GPIO, IOC, S3 relay, macOS model, message ID, payload offset, or radar PWM
  behavior is changed by this task.

## Current Status

Complete. Device/UART/attitude acceptance remains separately unverified because
this task did not flash either target.
