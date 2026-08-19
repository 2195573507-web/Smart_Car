# Dual IMU Upgrade Evidence

Scope: LSM303 + BMI323 acquisition, static calibration, vibration profiles,
SCBP/App telemetry, and a macOS debug display. No attitude algorithm, BMI323
fusion, FIFO, ISR, SPI/I2C owner, or `attitude.c` change was introduced.

## Audit and Impact Inventory

| Item | Current owner and call path |
| --- | --- |
| `imu_raw_data_t` | Defined in `STM32H757/Middleware/Calibration/imu_calibration.h`; produced by `imu_manager.c`, consumed by boot manager, calibration, filter, runtime telemetry, and snapshot getter. Legacy `ax/ay/az/mx/my/mz/timestamp/online` remain. |
| `imu_calibrated_data_t` | Source-compatible typedef of `imu_raw_data_t` in the same header; produced by `imu_calibration_apply()`, consumed by `imu_vibration_update()` and `imu_filter_update()`. |
| `imu_filtered_data_t` | Defined in `STM32H757/Middleware/Filter/imu_filter.h`; produced by `imu_filter_update()`, read by `attitude.c` and optional runtime diagnostics. Its shape and attitude consumer are unchanged. |
| `imu_boot_manager_update()` | `imu_task` -> `imu_task_step` -> `imu_update` -> `imu_publish_unified_snapshot` -> boot manager update. `imu_boot_manager_step()` remains the FSM timer/transport step. |
| `imu_calibration_update()` | Only called by `imu_boot_manager_update()` in `STATIC_CAL_SAMPLE`; it updates LSM accel, BMI accel, and BMI gyro streaming accumulators. |
| `imu_vibration_update()` | Only called by `imu_boot_manager_update()` in `VIBRATION_SAMPLE` after `imu_calibration_apply()`. |
| S3 IMU send | `STM32H757/Application/RTOS/imu_runtime.c` sends legacy status plus two source-tagged `SC_TYPE_IMU_TELEMETRY` frames; boot manager sends calibration and vibration result frames. |
| S3 relay | `ESPS3/components/smartcar_service/command_bridge.c::relay_telemetry()` validates new lengths and converts STM SCBP frames to App BLE frames. |
| App receive | `BLEManager` feeds `SmartCarProtocol.Parser`, constructs `DecodedMessage`, and passes it to `TelemetryStore`. |
| App models/UI | `VehicleState.swift` owns `IMUDataModel`, separate `LSM303Data`/`BMI323Data`, calibration results, and vibration profiles; `DeveloperModeView.swift` displays sensor data, biases, and RMS only. |

## Firmware Changes

- `STM32H757/Middleware/Calibration/imu_calibration.h`: extended the raw
  snapshot with explicit LSM/BMI values, per-sensor timestamps, and four valid
  flags; added dual calibration result types.
- `STM32H757/Middleware/Sensor/imu_manager.c/.h`: manager-owned snapshot and
  512-entry BMI ring. LSM303 remains sampled in the existing 10 ms task;
  `bmi323_task` samples BMI323 independently at 100/200/400/800 Hz (default
  100 Hz), and each manager tick publishes only the newest queued BMI sample.
  Existing LSM values are mirrored into legacy fields.
- `STM32H757/Middleware/Calibration/imu_calibration.c`: streaming `sum`,
  `sum_square`, and `sample_count` accumulators for LSM accel, BMI accel, and
  BMI gyro. BMI gyro bias is the mean output; no 5000-point sample array.
- `STM32H757/Middleware/Calibration/imu_boot_manager.c/.h`: `DUAL_IMU_BOOT`
  is the only transition authority. It closes a common phase window, then
  requires both source result flags before advancing. Legacy WAIT_SYNC, ACK,
  and radar PWM handling remain compatibility substeps rather than a second
  lifecycle FSM.
- `STM32H757/Middleware/Calibration/imu_vibration.c/.h`: separate
  `lsm_vibration_profile_t` and `bmi_vibration_profile_t`; each stores PWM,
  sample count, timestamp, accel RMS, and BMI gyro RMS where applicable.
- `STM32H757/Application/RTOS/imu_runtime.c`: source-tagged LSM/BMI telemetry
  frames at the existing bounded telemetry cadence; legacy status remains.
- `STM32H757/Middleware/Communication/SmartCar_Frame/sc_frame.c/.h`: added
  explicit sensor enum and message types `0x25`, `0x26`, and `0x27`.
- `STM32H757/Middleware/Communication/Services/s3_service.c/.h`: transport
  entry point for the new frames; no attitude producer change.

## Data Flow

```text
LSM303 (I2C) ----+                     +--> legacy LSM fields --> calibration/filter --> existing attitude
                 +--> imu_manager --> unified imu_raw_data_t --> boot FSM
BMI323 (SPI) --> bmi323_task --> manager-owned ring --> imu_manager
                                              +--> dual calibration/vibration
                                             |
                                             +--> SCBP 0x25/0x26/0x27/0x28 --> S3 relay --> BLE --> App models/debug page
```

## Startup and Calibration

1. `imu_runtime_start()` creates `imu_task`; `imu_init()` prepares the shared
   lifecycle without initializing either sensor by itself.
2. `INIT` creates LSM303/I2C4 and BMI323/SPI1 worker tasks. Both block on a
   task notification until both task handles exist; the scheduler is suspended
   while both notifications are posted, giving the buses one common start gate.
   The FSM advances only after both workers report success.
3. Every 10 ms the manager reads LSM accel/mag, obtains at most one newest
   BMI sample from its ring, records per-sensor timestamps/valid flags,
   publishes one snapshot, and advances the boot manager. The BMI
   acquisition task itself runs independently at the selected ODR.
4. `SELF_TEST` uses a shared one-second observation window. After the existing
   zero-PWM synchronization and settling interval, `STATIC_CALIBRATION` opens
   one 10-second window for LSM accel, BMI accel, and BMI gyro. It completes
   only when that window closes and both source result flags are valid; counts
   remain telemetry diagnostics, never a transition threshold.
5. The FSM emits legacy LSM bias plus two explicit dual calibration result
   frames, then uses the existing CAL_EVENT/ACK transition.

## Vibration Flow

For PWM levels 20/40/60/80/100, the existing settle/ACK sequence starts one
shared 30-second window. LSM samples are captured at the LSM acquisition point;
BMI accel/gyro samples are captured at its independent ODR. At the window end,
the manager stores and sends one LSM profile and one BMI profile only if both
sources observed valid data. Profile counts describe the capture; they do not
end the window or trigger the transition. The existing radar event ACK then
starts the next level.

## Protocol Layout

| Frame | Payload |
| --- | --- |
| `0x25 IMU_CAL_RESULT` | LSM: `sensor, flags, accel xyz` (14 bytes); BMI: `sensor, flags, accel xyz, gyro xyz` (26 bytes). |
| `0x26 IMU_VIBRATION_PROFILE` | LSM: `sensor, pwm, count, timestamp, accel RMS xyz, total` (26 bytes); BMI adds gyro RMS xyz/total (42 bytes). |
| `0x27 IMU_TELEMETRY` | `sensor, valid flags, timestamp, accel xyz, mag xyz` for LSM or gyro xyz for BMI (30 bytes). |
| `0x28 DUAL_IMU_STATUS` | `phase, lsm progress, bmi progress, overall progress, error, flags, vibration index, PWM, phase start/end` (16 bytes). |

Both firmware copies and App use `IMU_SENSOR_LSM303 = 0x01` and
`IMU_SENSOR_BMI323 = 0x02`; no sensor is inferred from frame order.

## App Changes

`DecodedMessage` decodes the source telemetry and lifecycle frame types.
`TelemetryStore` keeps
LSM303 and BMI323 data separately, including independent accel/mag or accel/gyro
valid flags, stores dual calibration results and profiles, and clears them on
disconnect. The Developer page shows the `DUAL_IMU_BOOT` phase, LSM/BMI/overall
progress, plus accel/mag for LSM303 and accel/gyro for BMI323. No BMI attitude
is added to this debug surface.

## Verification Evidence

- `cmake --build STM32H757/CM7/build/Debug --target Smart_Car_H757_CM7 -j2`:
  passed after compiling the explicit INIT gate. The integrated configuration is
  `SMARTCAR_BMI323_DEBUG_ONLY=OFF`; FLASH is 114260 B / 1 MB and RAM is
  52120 B / 128 KB.
- `source /Users/zhiqin/.espressif/v5.5.4/esp-idf/export.sh && idf.py -B build-dual-imu build` in `ESPS3`: passed with
  `SMARTCAR_BMI323_DEBUG_ONLY=OFF`; S3 app binary is 0xb22c0 bytes with 90% of
  the smallest app partition free.
- `swift build` in `IOS_APP/SmartCar_Control_MAC`: passed. The staged app was
  already running and its binary hash differs from the current build, so it was
  not terminated or replaced and live UI rendering remains unverified.
- `git diff --check`: passed.
- No flash, monitor, BLE capture, UART capture, sensor waveform, or vehicle
  runtime test was performed. Build evidence is not hardware acceptance.

## Risks and Boundaries

- Static calibration now intentionally requires valid BMI accel and gyro data;
  a missing BMI323 leaves the FSM in its existing calibration/error path even
  though the LSM303 10 ms task remains non-ISR and bounded.
- The 512-entry ring supplies 0.64 seconds of backlog at 800 Hz. Overflow,
  contention-drop, timestamp-delta, and maximum-latency values must be measured at 100, 200,
  400, and 800 Hz on hardware before increasing the default ODR.
- Telemetry rate now produces two new 30-byte frames per IMU telemetry period;
  UART/BLE bandwidth and notification drops require bench measurement.
- Timestamp values use the existing MCU millisecond tick and are per-sensor;
  they are not synchronized hardware sample clocks.
- Existing Swift concurrency warnings are outside this dual-IMU protocol work.
- Device behavior and App receipt remain unverified until hardware/BLE tests are
  run.

No git commit was created and no device was programmed.
