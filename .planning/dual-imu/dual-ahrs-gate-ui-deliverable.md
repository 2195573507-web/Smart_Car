# Dual-AHRS Gate and Developer Mode Comparison

Status: IMPLEMENTED, source/build/staged-bundle verified; hardware pending.

## Firmware Changes

| Location | Change | Reason and impact |
| --- | --- | --- |
| `STM32H757/Middleware/Attitude/DualAHRS/dual_ahrs.h` | Added `dual_ahrs_bias_t`, `WAIT_CAL`, `READY`, `TRACKING` states, and `dual_ahrs_set_bias()` | Establishes a public hand-off contract without changing the 80-byte payload layout. |
| `STM32H757/Middleware/Attitude/DualAHRS/dual_ahrs.c` | Added runtime reset, finite Bias validation, pre-calibration early return, and BMI/LSM accel plus gyro Bias subtraction | Prevents biquad/history/gyro integration before calibration and starts from a clean state after injection. |
| `STM32H757/Middleware/Sensor/imu_manager.c` | Gates the BMI producer on `imu_boot_manager_is_ready()` and maps `imu_calibration_result_t` into Dual-AHRS Bias once per READY epoch | Ensures INIT, static calibration, and vibration capture cannot advance the estimator. Reset remains in the BMI task to avoid cross-task state races. |

SCBP-V3 message IDs, field order, and the existing 80-byte Dual-AHRS payload are unchanged.

## App Changes

| Location | Change |
| --- | --- |
| `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/UI/DualAttitudeComparisonView.swift` | Added independent Primary and Redundant pose cards, independent 3D transforms, Euler values, Primary quaternion q0-q3, source/status labels, and a separate divergence panel. |
| `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/UI/DeveloperModeView.swift` | Replaced the former composite `DualAttitudeCard` with `DualAttitudeComparisonView`. |

`TelemetryStore.dualAttitude` remains the single source of data. The view reads
`primary` and `redundant` separately and uses `deltaRad` only for the monitoring
panel. `ViewThatFits` keeps the approved side-by-side layout on wide windows
and stacks the cards on narrow windows.

## Verification

- CM7 CMake target passed without new warnings/errors.
- SwiftPM `swift build` passed without new warnings/errors.
- `script/build_and_run.sh --verify` passed; staged app binary and SwiftPM
  binary SHA-256 matched.
- `git diff --check` passed.
- Hardware startup timing, BMI/LSM physical data, UART/BLE transport, and live
  posture behavior remain unverified because no flash or device capture was run.

## App Operation Path

Launch SmartCar Control MAC, select the `Developer` segment in the top mode
picker, and leave the page on `Overview`. The `DUAL_ATTITUDE` panel appears in
the right column below `DUAL_IMU_BOOT`; on a wide window it shows Primary on the
left and Redundant on the right, with the `ATTITUDE DIVERGENCE` panel below.
