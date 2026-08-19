# STM32 VERIFY_TIMEOUT Root-Cause Audit

## Scope and evidence boundary

This is a read-only audit of the current checkout. I inspected:

- `STM32H757/Middleware/Calibration/imu_boot_manager.c/.h`
- `STM32H757/Middleware/Calibration/imu_calibration.c/.h`
- `STM32H757/Middleware/Calibration/imu_vibration.c/.h`
- the active acquisition/call sites in `STM32H757/Middleware/Sensor/imu_manager.c`
- the STM ACK dispatch in `STM32H757/Middleware/Communication/Services/s3_service.c`

No implementation, protocol, timeout, sampling, or build configuration was changed by this audit. Line references below are to the inspected working tree and should be rechecked after any concurrent edit.

## Executive conclusion

1. The requested names `static_lsm`, `static_bmi`, `vibration_lsm`, and `vibration_bmi` do not exist as variables in the current STM32 source. They are logical aliases for four getter results:

   - `static_lsm` -> `imu_calibration_is_lsm_complete()`
   - `static_bmi` -> `imu_calibration_is_bmi_complete()`
   - `vibration_lsm` -> `imu_vibration_is_lsm_complete()`
   - `vibration_bmi` -> `imu_vibration_is_bmi_complete()`

2. `verify_lsm_done` and `verify_bmi_done` are stack locals in `imu_boot_manager_step()` (lines 721-722). They are recomputed only while `IMU_PHASE_VERIFY` is active (lines 805-813) and then copied to `s_boot.lsm_phase_complete`/`s_boot.bmi_phase_complete` (lines 868-871).

3. In the current state logic, a genuinely accepted id=1 ACK followed by a genuinely accepted final id=2 ACK should leave all four underlying completion getters true. The static result is retained after `imu_calibration_finish_window()`; the final vibration dataset is retained after `imu_vibration_poll()`/`finalize_window_locked()`, and no later vibration window is started on the final-index id=2 ACK. Therefore a subsequent five-second `VERIFY_TIMEOUT` is not explained by the normal calibration data flow alone.

4. `VERIFY_TIMEOUT` is taken at line 877 only when the VERIFY deadline is reached before both local completion flags and the final ACK condition are satisfied. If the four local completion getters are true but id=3 is not accepted, the code normally follows the separate event retry path and reaches `IMU_ERROR_CAL_EVENT_TIMEOUT` at lines 886-895, not `IMU_ERROR_VERIFY_TIMEOUT`. This distinction is important: an S3-side “ACK sent/accepted” log is not proof that STM accepted the correlated ACK.

5. The current source has no VERIFY-entry or VERIFY-timeout diagnostic carrying the requested flag values. A bounded diagnostic addition in `imu_boot_manager.c` is the minimum safe next step; it must not alter state transitions, protocol IDs, payloads, sampling cadence, or timeout constants.

## Completion-source audit

| Logical value | Current source and update point | Exact condition |
| --- | --- | --- |
| `verify_lsm_done` | `imu_boot_manager_step()`, lines 721, 805-813, 868-871 | `imu_calibration_is_lsm_complete() && imu_vibration_is_lsm_complete()` |
| `verify_bmi_done` | `imu_boot_manager_step()`, lines 722, 805-813, 868-871 | `imu_calibration_is_bmi_complete() && imu_vibration_is_bmi_complete()` |
| `static_lsm` (logical alias) | `imu_calibration_is_lsm_complete()`, `imu_calibration.c:457-465` | `s_calibration.complete != 0 && quality.lsm_accel.quality_ok != 0` |
| `static_bmi` (logical alias) | `imu_calibration_is_bmi_complete()`, `imu_calibration.c:467-475` | `s_calibration.complete != 0 && quality.bmi_accel.quality_ok != 0 && quality.bmi_gyro.quality_ok != 0` |
| Static result producer | `imu_calibration_finish_window()`, `imu_calibration.c:289-316` | `window_active` expires; all three quality gates pass; then `s_calibration.complete = 1` |
| Static quality inputs | `imu_calibration.c:173-224` | 90% floor (`quality_ok`) for LSM accel, BMI accel, and BMI gyro |
| `vibration_lsm` (logical alias) | `imu_vibration_is_lsm_complete()`, `imu_vibration.c:611-618` | `s_vibration.lsm_dataset.complete` |
| `vibration_bmi` (logical alias) | `imu_vibration_is_bmi_complete()`, `imu_vibration.c:621-628` | `s_vibration.bmi_dataset.complete` |
| Vibration completion producer | `finalize_window_locked()`, `imu_vibration.c:213-229` | Time closes the shared window; each dataset gets `quality_ok` and `complete` |
| LSM capture source | `imu_manager.c:777-784` | `imu_vibration_capture_lsm()` at LSM acquisition cadence, valid/invalid sample recorded |
| BMI capture source | `imu_manager.c:883-920` | Independent BMI task calls calibration update and `imu_vibration_capture_bmi()` at the active BMI cadence |

## State and event timeline

1. `enter_verify_phase_locked()` (`imu_boot_manager.c:658-663`) is called only from the final-index id=2 ACK branch (`imu_boot_manager.c:1143-1152`). `enter_phase_locked()` resets the phase-completion flags to zero (`imu_boot_manager.c:366-385`), then VERIFY gets a five-second deadline.

2. The static window closes in `imu_boot_manager_step()` (`imu_boot_manager.c:911-930`). The id=1 wait is armed only if both static completion results are true (`begin_event_wait_locked(SC_CAL_EVENT_STATIC_CAL_DONE)` at lines 925-928); the event is transmitted at line 1009.

3. Each vibration window closes at lines 966-1000. The id=2 wait is armed only if both current-window dataset completion results are true (`imu_boot_manager.c:974-995`); the event is transmitted at line 1014.

4. Upon final id=2 ACK, `imu_boot_manager_on_cal_event_ack()` clears `event_waiting` and enters VERIFY (`imu_boot_manager.c:1130-1152`). It does not clear either calibration module's retained result.

5. During VERIFY, local checks happen outside the boot mutex (`imu_boot_manager.c:802-813`). The manager then publishes the two flags and, only when both are set and `event_waiting == 0`, arms/transmits id=3 (`imu_boot_manager.c:868-884`, line 1017).

6. An id=3 ACK only sets `verify_acknowledged` (`imu_boot_manager.c:1154-1156`). READY requires that ACK plus both flags (`imu_boot_manager.c:873-876`, then `imu_boot_manager.c:1019-1036`).

## Why the reported symptom is unresolved by source alone

The following are **confirmed source facts**:

- id=1 TX is reachable only after both static completion flags pass (lines 925-929 and 1005-1009).
- id=2 TX is reachable only after both final-window vibration completion flags pass (lines 983-995 and 1011-1014).
- Final-index id=2 ACK enters VERIFY without starting another vibration window (lines 1146-1152).
- The completion getters are not reset by `enter_verify_phase_locked()`; only `s_boot.lsm_phase_complete` and `s_boot.bmi_phase_complete` are reset by the generic phase transition.

Consequently, if “id=1 ACK success” and “id=2 ACK success” mean that STM accepted the correlated ACKs through `s3_service_on_frame()` (`s3_service.c:243-261`) and `imu_boot_manager_on_cal_event_ack()` (`imu_boot_manager.c:1130-1156`), then the expected next branch is id=3 TX, followed by READY after id=3 ACK. A five-second VERIFY timeout would require an additional condition not established by this static source review, such as:

- the observed ACK was only an S3-side transmit/accept log, while STM rejected or never correlated it;
- a stale binary or mixed STM/S3 build is running;
- one or more completion getters returned zero at VERIFY due to runtime reset/concurrency/memory corruption;
- `imu_boot_manager_step()` was not scheduled during the VERIFY window (the normal FreeRTOS caller is `imu_manager.c:1362-1393`, once per 10 ms task loop);
- a reset/recovery path restarted one calibration module or replaced the manager state between events.

These are hypotheses, not runtime findings. Current source contains no log that can distinguish them.

## Minimum diagnostic recommendation (no state change)

Add bounded, non-blocking logs in `imu_boot_manager.c` only:

- On VERIFY entry, after the entry is committed, report:

  `IMU_VERIFY_ENTER static_lsm=<0|1> static_bmi=<0|1> vibration_lsm=<0|1> vibration_bmi=<0|1>`

- On the timeout branch, report:

  `IMU_VERIFY_TIMEOUT verify_lsm_done=<0|1> verify_bmi_done=<0|1> pending_event_id=<n> event_waiting=<0|1>`

For a useful correlation, also include a monotonic timestamp and log STM-side ACK reception (event id, result, pending id, and phase) at the existing `imu_boot_manager_on_cal_event_ack()` boundary. Keep the records bounded by `SMARTCAR_LOG_MAX_PAYLOAD`, emit once per entry/timeout, and do not call a blocking transport while holding `s_boot`'s mutex.

The entry log should capture the four getter values, while the timeout log should use the exact locals/fields that gate the branch. If the four entry values are all `1` and no id=3 TX appears, inspect `event_waiting`/transport correlation. If any entry value is `0`, inspect that module's quality/count diagnostics and acquisition scheduling. If entry values are all `1`, id=3 TX and STM ACK RX are present, but timeout still occurs, treat it as a build/runtime identity or memory-corruption issue rather than changing the state machine.

## Requested change record

| Item | Audit result |
| --- | --- |
| Modified source files | None |
| Suggested implementation location | `STM32H757/Middleware/Calibration/imu_boot_manager.c`, VERIFY entry/timeout diagnostics only |
| Functions/symbols | `enter_verify_phase_locked()`, `imu_boot_manager_step()`, optionally `imu_boot_manager_on_cal_event_ack()` for ACK RX correlation |
| Reason | Expose the four completion sources and distinguish local VERIFY gating from id=3 transport correlation |
| Potential impact | A few bounded log calls; possible UART/log timing perturbation if emitted excessively; no protocol/state/timeout/sampling change is required |
| Validation | CM7 configure/build; then device capture of id=1/id=2/id=3 TX/RX, four completion flags, `pending_event_id`, `event_waiting`, phase and deadline; hardware/UART/BLE acceptance remains unverified here |

## Evidence limitations

- This report is static source evidence only; no STM32 flash, UART capture, S3 runtime log, sensor response, or hardware integration test was available.
- The working tree is already dirty in the inspected modules. The report does not attribute those pre-existing edits to this audit.
- The current STM source does not emit the requested VERIFY diagnostics, so the exact runtime value that caused the reported timeout remains unproven until a diagnostic build is captured.
