# ESP32-S3 Radar Calibration Audit

## Scope and evidence boundary

This report is a source-only audit of the ESP32-S3 calibration receiver and
its STM32 SCBP-V3 dispatch path. The inspected files are:

- `ESPS3/components/smartcar_service/radar_calibration_manager.c`
- `ESPS3/components/smartcar_service/radar_calibration_manager.h`
- `ESPS3/components/smartcar_service/command_bridge.c`
- `ESPS3/components/smartcar_protocol/include/frame.h`
- `ESPS3/components/smartcar_protocol/frame.c`
- `ESPS3/components/radar_control/radar_control.c/.h`
- STM32 event timing sources used to calculate the cross-device timeout

No ESP32 source logic was modified by this audit. The only file created by
this agent is this report. UART delivery, STM32/S3 runtime identity, radar
motion, and hardware behavior remain unverified.

## Executive findings

1. The current SCBP-V3 event route is present and preserves the frozen
   `SCBP_MSG_ID_CAL_EVENT = 0x0401` mapping. The wire payload remains one byte:
   `0x01`, `0x02`, or `0x03`.
2. The logical states requested by the task are present under different source
   names: `RADAR_WAIT_SYNC`, `RADAR_WAIT_ACK`, `RADAR_WAIT_EVENT`,
   `RADAR_CAL_DONE`, and `RADAR_CAL_ERROR`. `WAIT_RADAR_ACK` is only a log
   label; `WAIT_CAL_EVENT` is represented by `RADAR_WAIT_EVENT`.
3. The current working tree contains an uncommitted S3 timeout change from
   `RADAR_CAL_VIBRATION_EVENT_TIMEOUT_MS = 25 s` to `35 s`. This is justified:
   STM waits 2 s before each vibration window and captures for 30 s, so an
   event can arrive about 32 s after the S3 PWM ACK. The old 25 s deadline was
   too short. The change is visible in the working tree but was not made by
   this audit.
4. `RADAR_CAL_COMPLETE_TIMEOUT_MS = 5 s` is used for event `id=3`. STM's
   `DUAL_IMU_VERIFY_WINDOW_MS` is also 5 s, and STM sends `id=3` as soon as the
   local completion getters are true. Static source evidence does not prove
   that this window must be increased, but the equal deadlines leave no
   scheduling/transport margin. A runtime capture is required before changing
   it. No `RADAR_COMPLETE_EVENT_TIMEOUT_MS` symbol currently exists.
5. The working tree also contains broader, uncommitted S3 state/recovery and
   completion changes unrelated to the timeout. They should be reviewed
   separately because the requested ESP32 scope is limited to a necessary
   timeout repair.

## Protocol and dispatch audit

| Layer | Source | Result |
| --- | --- | --- |
| STM producer | `STM32H757/Middleware/Calibration/imu_boot_manager.c:589-593` | `send_cal_event(id)` sends a one-byte payload through `SC_TYPE_CAL_EVENT`. |
| S3 SCBP dispatch | `ESPS3/components/smartcar_service/command_bridge.c:327-335` | Accepts `SCBP_MSG_ID_CAL_EVENT (0x0401)`, requires length 1, stores ACK context, and forwards to `radar_calibration_manager_on_frame(SC_TYPE_CAL_EVENT, ...)`. |
| ID definitions | `ESPS3/components/smartcar_protocol/include/frame.h:70-72,116-117,145-147` | `0x0401` is unchanged; event IDs are `0x01/0x02/0x03`; ACK adapter type is `0x19`. |
| S3 event ACK | `radar_calibration_manager.c:124-146` | Encodes the two-byte logical `{event_id, result}` using the existing V3 generic ACK envelope and sends it on STM UART. |
| PWM ACK dispatch | `command_bridge.c:337-359` | Matches the pending `RADAR_PWM_READY` transaction before forwarding the two-byte speed/result to the manager. |

There is no source evidence of an ID or payload-format drift in the live
SCBP-V3 path. The ACK context is set when the incoming event frame is
dispatched, which is required by `sc_frame_encode(SC_TYPE_CAL_EVENT_ACK, ...)`.

## State machine and event handling

| Requested state | Actual enum/log | Entry | Event/timeout behavior |
| --- | --- | --- | --- |
| `WAIT_SYNC` | `RADAR_WAIT_SYNC` | `init`/`reset_tracking`; also entered after current recovery paths | Ignores calibration events until a valid STM `BOOT_READY {state=1,result=0}` arrives. |
| `WAIT_RADAR_ACK` | `RADAR_WAIT_ACK`; log `WAIT_RADAR_ACK` | `RADAR_SET_PWM` applies the current level and sends `RADAR_PWM_READY` | 500 ms ACK deadline, up to 3 retries; a mismatched/rejected ACK enters the current recovery path. |
| `WAIT_CAL_EVENT` | `RADAR_WAIT_EVENT` | Accepted PWM ACK | Waits for the stage-specific event and applies the event deadline. |
| `CAL_DONE` | `RADAR_CAL_DONE` | Accepted `id=3` after final completion commit | Sets `s_done=true`; duplicate accepted event IDs are re-ACKed. |
| `CAL_ERROR` | `RADAR_CAL_ERROR` | Enum remains available | In the current working tree, failures call `enter_sync_wait()` and reset to `RADAR_WAIT_SYNC`; the baseline code used terminal `enter_error()` and `RADAR_CAL_ERROR`. This is a broader behavior change, not part of the timeout audit. |

### `id=1`: static calibration complete

1. After valid STM `BOOT_READY`, S3 enters `RADAR_SET_PWM` with `s_pwm=0`.
2. `radar_control_set_calibration_pwm(0)` is applied, then S3 waits for the
   matching `RADAR_PWM_ACK`.
3. A matching ACK sets `s_wait_event=RADAR_WAIT_STATIC_DONE` and
   `s_state=RADAR_WAIT_EVENT`.
4. `SC_CAL_EVENT_STATIC_CAL_DONE (0x01)` is accepted only when the state is
   `RADAR_WAIT_EVENT`, `s_pwm == RADAR_MIN_SPEED`, and the event has not
   already been ACKed. S3 sends `{id=1,result=OK}` and advances to
   `RADAR_NEXT_LEVEL`.

### `id=2`: vibration step complete

1. For each nonzero level, the matching PWM ACK sets
   `s_wait_event=RADAR_WAIT_VIBRATION_DONE` and starts the event window.
2. `SC_CAL_EVENT_VIBRATION_STEP_DONE (0x02)` is accepted only in
   `RADAR_WAIT_EVENT` after `s_event_not_before_us`.
3. A repeated early `id=2` is explicitly re-ACKed and does not advance the
   level. This handles an older event retry while STM is still in its 2 s
   settle/capture boundary.
4. For non-final levels, S3 advances to `RADAR_NEXT_LEVEL`. For the final
   level, it switches `s_wait_event` to `RADAR_WAIT_CAL_COMPLETE` and arms the
   separate `id=3` deadline.

### `id=3`: final calibration complete

1. After the final `id=2` ACK, STM enters its VERIFY phase and S3 waits in
   `RADAR_WAIT_EVENT` with `s_wait_event=RADAR_WAIT_CAL_COMPLETE`.
2. `SC_CAL_EVENT_COMPLETE (0x03)` is accepted only in that state.
3. Current working-tree code applies final PWM/control completion before
   sending the success ACK. Baseline code ACKed first; the ordering change is
   not a timeout fix and needs separate review.
4. The current S3 deadline is 5 s. STM's source deadline for VERIFY is also
   5 s. This is a matching contract, not proof of runtime success.

## Timeout calculation

### `id=2` window

The STM sources define:

- `DUAL_IMU_STATIC_SETTLE_MS = 2000` in `imu_boot_manager.c`.
- `IMU_VIBRATION_WINDOW_DURATION_MS = 30000` in `imu_vibration.h`.
- Event transmission follows the window close and result checks in
  `imu_boot_manager_step()`.

Therefore the expected event time after an accepted PWM ACK is approximately
`2,000 + 30,000 = 32,000 ms`, plus scheduler and UART latency. The baseline
S3 `25,000 ms` deadline could expire before the event. The current uncommitted
`35,000 ms` value leaves approximately 3 s for that latency and is the
necessary timeout correction identified by this audit.

### `id=3` window

The STM VERIFY deadline is `DUAL_IMU_VERIFY_WINDOW_MS = 5000`. After the final
`id=2` ACK, `imu_boot_manager_on_cal_event_ack()` enters VERIFY; on the next
`imu_boot_manager_step()` it checks the retained static/vibration completion
getters and sends `id=3` when both are true and no event is pending. S3 arms
`RADAR_CAL_COMPLETE_TIMEOUT_MS = 5000` immediately after acknowledging final
`id=2`.

This equal-deadline arrangement can fail only if the STM task, UART, or S3
service is delayed long enough to consume the available window. No source-only
evidence justifies changing the value now. If a device capture demonstrates
`id=3` TX after the S3 deadline, the bounded proposal is:

| Before | After (proposal only) | Reason |
| --- | --- | --- |
| `RADAR_CAL_COMPLETE_TIMEOUT_MS = 5000` | Introduce a named `RADAR_COMPLETE_EVENT_TIMEOUT_MS` and set it to a value greater than the measured worst-case delay (for example 7000 ms) | Add measured transport/scheduling margin while keeping the event ID, payload, state transitions, and sampling unchanged. |

Do not apply that proposal without a capture showing the actual delay; an
arbitrary timeout increase would mask an ACK correlation or scheduling fault.

## Current working-tree changes requiring review

`git diff` shows the following S3 changes relative to `HEAD`; they are
pre-existing in the shared working tree and are not attributed to this audit:

- `RADAR_CAL_VIBRATION_EVENT_TIMEOUT_MS`: 25 s -> 35 s (the justified timeout
  repair described above).
- `enter_error()`/terminal `RADAR_CAL_ERROR` paths changed to
  `enter_sync_wait()`/automatic reset and recovery.
- Additional encode/UART logging and failed-TX handling.
- BOOT_READY recovery handling and radar-control admission changes.
- Final `id=3` completion ordering changes, including a call to
  `radar_control_set_imu_cal_done(true)`.

Only the first item is in the requested narrow timeout scope. The others may
be valid separate repairs, but combining them with this audit would make it
impossible to attribute a runtime result to the timeout change alone.

## Modification record

| Item | Result |
| --- | --- |
| Modified files | None in ESP32 source; this report only. |
| Modified functions | None. |
| Reason | The task explicitly requested audit first and advised against directly changing the S3 state logic. |
| Potential impact | No firmware impact from this audit. The existing dirty S3 changes have the wider impacts listed above. |
| Verification | Static source/path checks completed. Firmware build and hardware/runtime capture were not run by this agent. |

## Verification plan

1. Build the active S3 project after the main agent has selected the authorized
   timeout diff; do not treat a green build as UART or radar acceptance.
2. Capture the STM/S3 sequence and correlate frames by sequence and event ID:
   `BOOT_READY -> PWM_READY/ACK -> CAL_EVENT id=1 -> ACK ->` five vibration
   levels of `PWM_READY/ACK -> CAL_EVENT id=2 -> ACK -> CAL_EVENT id=3 -> ACK`.
3. Confirm the final `id=2` event arrives before 35 s from its matching PWM ACK,
   and confirm `id=3` TX/RX timestamps relative to the 5 s VERIFY window.
4. Confirm every CAL_EVENT frame remains `msg_id=0x0401`, one-byte payload,
   ACK-required flag, valid sequence correlation, and CRC16-MODBUS.
5. Confirm no event is accepted in `WAIT_SYNC`, `WAIT_RADAR_ACK`, or the wrong
   stage, and that duplicate `id=2`/`id=3` behavior is idempotent.
6. Hardware acceptance still requires flashed firmware, STM UART capture,
   S3 logs, radar PWM observation, and sensor/runtime evidence.

## Open risks

- The 35 s value is source-level timing evidence only; it has not been proven
  on a board with real UART scheduling and radar load.
- The S3 and STM VERIFY deadlines are equal, leaving no explicit margin for
  task starvation or transport delay.
- The current working-tree final-completion ordering may leave radar control at
  a different speed than `s_pwm=100`; inspect `radar_control_set_imu_cal_done`
  before accepting the broader dirty diff.
- The service uses a single pending ACK slot for ACK-required outbound frames;
  an unrelated interleaved transaction could overwrite CAL_EVENT correlation.
- No runtime evidence is available to distinguish a missing event, a rejected
  ACK, a stale binary, or a UART wiring problem.
