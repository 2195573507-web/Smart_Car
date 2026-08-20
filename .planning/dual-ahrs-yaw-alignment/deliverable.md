# DualAHRS LSM303 Yaw Alignment Delivery

## Implemented

- Preserved BMI323 as schema-2 Primary and LSM303 as Redundant.
- Applied the approved `-105.0 deg` installation correction only to the solved
  redundant yaw. The primary and redundant outputs now use the primary READY
  yaw reference, avoiding independent zeroing that would cancel the fixed
  LSM303 correction.
- Rebuilt output quaternions after READY yaw referencing so the 80-byte payload
  Euler and quaternion fields represent the same output frame.
- Preserved all schema-2 offsets, message IDs, S3 relay behavior, and macOS
  field mapping.

## Source-Confirmed Lifecycle

- S3 calibration only sets and acknowledges radar PWM speed `0`.
- STM32 enters `IMU_PHASE_READY` directly after a valid static window; no
  vibration phase or `0x0204`/`0x0206` business path is present in active code.
- UART2 DMA statistics increase `rx` and `frame_events` on each non-empty DMA
  ring-buffer commit.

## Build Evidence

- CM7 clean build: passed.
- ESP32-S3 ESP-IDF 5.5.4 clean isolated build: passed, with no warning/error
  matches in its complete build log.

## Device Evidence Still Required

- Flash the matching CM7 and S3 images, reset both targets, and capture the
  correlated UART/BLE logs. Confirm `WAIT_SYNC -> STATIC_CAL_* -> IMU_READY`,
  consecutive increasing `[UART2_DMA_STACK] rx/frame_events`, PWM remains zero,
  and stable `[DUAL_AHRS] diff_deg` roll/pitch/yaw all within `+/-2.0 deg`.
