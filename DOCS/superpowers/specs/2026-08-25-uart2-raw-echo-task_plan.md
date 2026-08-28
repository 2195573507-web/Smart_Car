# Task Plan: UART2 Raw Echo Test

## Goal

Provide a reversible CM7/S3 UART2 raw Echo test that bypasses SRP and DMA
startup for physical-layer diagnosis.

## Phases

- [x] Inspect current UART ownership and generated pin configuration.
- [x] Design an opt-in isolated test mode.
- [x] Implement CM7 and S3 test entry points.
- [x] Build both test configurations and inspect the resulting image paths.
- [x] Record the required hardware capture criteria; flashing and captures are
  outside this source/build task.
