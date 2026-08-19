# Smart_Car Dual IMU Upgrade

## Dual Lifecycle Upgrade (2026-08-18)

### Goal

Replace the legacy calibration-state ownership with one `DUAL_IMU_BOOT`
lifecycle. LSM303 and BMI323 must enter and complete each lifecycle phase as a
pair. Existing LSM303-only filter/AHRS ownership remains unchanged.

### Plan

- [x] Audit boot manager, IMU manager, calibration, vibration, S3 relay, and App
- [x] Define lifecycle phases, time windows, compatibility mapping, and status frame
- [x] Implement STM32 lifecycle FSM and concurrent INIT workers
- [x] Add S3/App lifecycle status propagation and display
- [x] Build CM7, S3, and SwiftPM targets; record source/build-only limits

### Hard Boundaries

- Do not modify `attitude.c`, AHRS, fusion, or attitude parsing.
- Do not use sample count as a lifecycle transition condition.
- Preserve the existing PWM=0 and calibration-event ACK protocol as lifecycle
  substeps, not as a second FSM.

## Goal
Implement the requested first-stage dual IMU acquisition, static calibration,
vibration capture, protocol extension, and App debug display while preserving
the LSM303 filter/attitude path and the existing imu_boot_manager FSM.

## Phases
- [x] Read-only architecture audit and impact inventory
- [x] Unify manager-owned LSM303/BMI323 snapshot
- [x] Upgrade calibration and vibration consumers
- [x] Extend STM/S3 protocol and telemetry
- [x] Add App models, decoding, storage, and debug display
- [x] Build/static verification and final evidence report

## Constraints
- No attitude algorithm or fusion changes.
- No BMI323 fusion, FIFO, ISR, or owner changes.
- Preserve legacy fields and frame meanings.
- Do not flash or commit.

## Independent BMI323 Sampling Extension (2026-08-18)
- [x] Design approved: independent FreeRTOS acquisition task with manager-owned software ring buffer
- [x] Add configurable BMI323 ODR (100/200/400/800 Hz), default 100 Hz
- [x] Add bounded ring buffer, overflow accounting, and capture statistics
- [x] Decouple BMI acquisition from the 100 Hz LSM303/boot-manager task
- [x] Preserve 10 ms manager snapshot/filter/attitude behavior while consuming newest BMI sample
- [x] Build CM7 and verify RAM/stack/heap impact; confirm S3/App 0x27 compatibility by source checks

## Accepted Design
- `bmi323_raw_sample_t` stores timestamp, raw accel/gyro `int16_t` axes, and a valid flag.
- `BMI_RING_BUFFER_SIZE` is 512; insertion never blocks and drops the oldest sample on full.
- BMI task period is derived from the selected ODR using the existing 1 ms FreeRTOS tick.
- The 100 Hz `imu_task` drains the ring without a loop, retaining the latest sample and marking the snapshot invalid when no new sample arrived during that manager tick.
- No ISR, hardware FIFO, SPI/I2C owner change, attitude/fusion change, or App UI change is included.

## Dual-AHRS Startup Gate and Comparison UI (2026-08-19)

### Goal

Keep Dual-AHRS inert until the complete static calibration and vibration
capture lifecycle reaches `IMU_PHASE_READY` and the latest calibration result
has been injected. Then present independent Primary and Redundant attitude
views in Developer Mode with live divergence monitoring.

### Plan

- [x] Add Dual-AHRS `WAIT_CAL` gate, calibration bias injection, and READY/TRACKING hand-off
- [x] Gate the BMI producer call path and reset the gate on any non-ready phase
- [x] Replace the composite Developer Mode card with independent comparison components
- [x] Build CM7 and SwiftPM targets; run diff/static checks and record evidence

### Boundaries

- Preserve SCBP-V3 ATTITUDE and Dual-AHRS 80-byte payload layouts.
- Do not modify sensor ownership, GPIO, transport framing, or calibration algorithms.
- Hardware, UART/BLE delivery, and physical attitude behavior remain unverified unless captured separately.
