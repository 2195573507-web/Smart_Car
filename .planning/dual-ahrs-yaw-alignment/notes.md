# Findings: DualAHRS LSM303 Yaw Alignment

- `dual_ahrs.c` currently applies leveling matrices after bias removal and before
  both estimators. Primary uses BMI323 accel/gyro and redundant uses LSM303
  accel/magnetometer.
- Primary and redundant estimators share the LSM303 magnetometer input. A
  rotation applied to that common input would alter Primary as well and leave
  their yaw difference unchanged. The correction is therefore a fixed
  `-105.0 deg` offset applied only to the solved redundant yaw. Its READY
  reference is the primary yaw reference, so independent zeroing cannot cancel
  the installation correction.
- Existing output yaw zeroing captures one reference per estimator at READY. It
  must also update the output quaternion; otherwise Euler yaw and quaternion
  encode different reference frames in the schema-2 payload.
- `imu_boot_manager.c` transitions from a successful static-window result
  directly to `IMU_PHASE_READY`/`IMU_READY`. Active source has no `0x0204` or
  `0x0206` business path. Device logs remain required to prove the handshake
  exits `WAIT_SYNC` and that UART2 counters increase.

## Verification

- CM7 clean build passed from `STM32H757/CM7` with
  `cmake --preset Debug`, `cmake --build build/Debug --target clean`, and
  `cmake --build build/Debug --target Smart_Car_H757_CM7 -j2`.
- ESP-IDF 5.5.4 clean isolated build passed in `ESPS3/build-s3-yaw-clean`.
  Its 1112-step log has no `warning:`, `error:`, or failure match and generated
  `smartcar_s3_gateway.elf` and `.bin`.
- Source scan across active STM32/S3/Swift/Common C/C++/Swift/CMake files found
  no `SC_TYPE_*`, `0x0200`, `0x0204`, or `0x0206` occurrence.
- No flash, reset, UART monitor capture, sensor measurement, BLE test, or radar
  observation was performed. Those remain device-level acceptance requirements.
