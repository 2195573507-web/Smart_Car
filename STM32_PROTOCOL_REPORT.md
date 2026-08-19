# STM32 Protocol Audit Report

## Scope and evidence boundary

This is a read-only audit of the STM32 CAL_EVENT path requested for the
VERIFY_TIMEOUT investigation. The live checkout contains unrelated uncommitted
changes; no source file was modified for this report. Source inspection proves
the static call/encoding path only. UART delivery, S3 ACK delivery, and device
behavior remain unverified until an authorized capture is made.

## 1. Current call path

| Layer | File and function | Current behavior |
| --- | --- | --- |
| Event producer | `STM32H757/Middleware/Calibration/imu_boot_manager.c:589-593`, `send_cal_event()` | Sends a one-byte payload containing `event_id` through `send_frame(SC_TYPE_CAL_EVENT, ...)`. The three IDs are static done (`0x01`), vibration step done (`0x02`), and complete (`0x03`). |
| Callback adapter | `STM32H757/Middleware/Calibration/imu_boot_manager.c:404-416`, `send_frame()` | Reads the configured transport callback, calls it, and returns `1` whenever the callback is non-NULL. Because the callback type returns `void`, this return value does not represent encode or UART success. |
| STM/S3 service | `STM32H757/Middleware/Communication/Services/s3_service.c:136-158`, `s3_service_send_boot_frame()` | Calls `sc_frame_encode()`, then `uart_link_send()`. Encode failures and non-`HAL_OK` UART results are currently ignored for all types except a pre-existing radar-ACK log. |
| V3 compatibility encoder | `STM32H757/Middleware/Communication/SmartCar_Frame/sc_frame.c:286-350`, `sc_frame_encode()` | Maps the legacy type to a V3 message, allocates a sequence, encodes the frame, and records a single pending ACK transaction for `SC_TYPE_CAL_EVENT`. |
| V3 encoder/CRC | `STM32H757/Middleware/Communication/SmartCar_Frame/sc_frame.c:201-237`, `scbp_frame_encode()` | Validates version, priority, flags, payload length, and capacity; writes the V3 frame and CRC16-MODBUS. |
| UART transport | `STM32H757/Middleware/Communication/UART_Link/uart_link.c:129-167`, `uart_link_send()` | Rejects an unready link/null/zero-length input, serializes through a mutex, calls `HAL_UART_Transmit()`, and returns `HAL_OK`, `HAL_TIMEOUT`, or another HAL error while incrementing counters. |
| ACK receive | `STM32H757/Middleware/Communication/Services/s3_service.c:243-261`, `s3_service_on_frame()` | Validates a five-byte V3 ACK and calls `imu_boot_manager_on_cal_event_ack()` only after strict pending message-ID, sequence, source, destination, and result checks. |

`STM32H757/Application/RTOS/imu_runtime.c` does not contain `send_cal_event`,
`SC_TYPE_CAL_EVENT`, or `SCBP_MSG_ID_CAL_EVENT`. Its current role is periodic
IMU/attitude telemetry (`imu_send_telemetry()`, lines 65-145). The boot manager
is stepped by the sensor task (`STM32H757/Middleware/Sensor/imu_manager.c`),
not by this runtime telemetry module.

## 2. Protocol mapping confirmed

The current source preserves the frozen SCBP contract:

- `SC_TYPE_CAL_EVENT = 0x18` is an adapter input and maps to
  `SCBP_MSG_ID_CAL_EVENT = 0x0401` (`sc_frame.h:71,118` and
  `sc_frame.c:131`). No alternate ID is selected.
- The generated V3 frame uses local STM32 source `0x01`, default S3 destination
  `0x02`, realtime priority, and `SCBP_FLAG_ACK_REQUIRED`
  (`sc_frame.h:17-29`, `sc_frame.c:58-94`).
- The wire payload remains exactly one byte: the event ID. No result byte is
  added to the protocol payload.
- The sequence is generated at `sc_frame.c:47-55` and is written at V3 header
  offset 8. On successful encoding, the pending transaction stores message ID,
  sequence, destination, legacy type, and event ID (`sc_frame.c:339-348`).
- The expected ACK is generic V3 `MSG_ID=0x0005` with the five-byte ACK payload.
  `scbp_pending_tx_match_ack()` (`sc_frame.c:172-199`) requires matching
  message ID/sequence and endpoint direction before clearing the pending slot.

There is no static source evidence that the current CAL_EVENT ID or payload is
wrong. The communication gap is observability and status propagation.

## 3. Findings and risks

### Finding A: encode and UART failures are silent

`s3_service_send_boot_frame()` only enters its body when `sc_frame_encode()`
returns zero and discards both the encoder error code and the return value from
`uart_link_send()`. A CAL_EVENT can therefore be queued in the boot manager's
pending-event state without a visible `CAL_EVENT` transport diagnosis.

### Finding B: `send_cal_event()` success is not transport success

`send_frame()` returns `1` after invoking a `void` callback. Consequently,
`send_cal_event()` cannot distinguish an absent callback, encode failure, UART
not-ready, UART mutex timeout, or HAL transmit failure. The boot manager can
retry based on its ACK deadline, but the current log stream cannot identify
which layer failed.

### Finding C: one pending ACK slot is shared

`scbp_pending_tx_t s_pending_tx` stores only one outgoing ACK-required
transaction. CAL_EVENT retries refresh that slot with a new sequence, which is
correct for a serialized event wait. Any unrelated ACK-required transmission
interleaved before the CAL_EVENT ACK would overwrite the correlation state and
cause the valid ACK to be rejected. No such interleaving is proven by this
audit, so this is a robustness risk rather than a confirmed VERIFY root cause.

### Finding D: ACK rejection is intentionally terminal for the transaction

ACKs with the wrong source, destination, message ID, or sequence leave the
pending transaction unchanged and produce the generic
`SCBP_ACK rejected; pending transaction unchanged` warning. This protects
protocol compatibility, but without a CAL_EVENT-specific TX/ACK trace it is
not possible to tell an ACK loss from an ACK correlation failure in a device
log.

## 4. Required diagnostics (minimal, wire-compatible)

The diagnostics should be added around the existing `s3_service_send_boot_frame`
call. They must use the existing logger and must not change the V3 frame, the
`0x0401` ID, or the one-byte event payload.

| Diagnostic | Emit condition | Required fields |
| --- | --- | --- |
| `CAL_EVENT_TX` | The CAL_EVENT frame was encoded and the UART send result is available | `id=<payload[0]>`, `seq=<encoded frame[8]>`, `result=<HAL send status>`; use the encoded frame's sequence rather than allocating a second sequence. |
| `CAL_EVENT_ENCODE_FAIL` | `sc_frame_encode()` returns non-zero | `id=<payload[0] or 0>`, `result=<encoder return code>`; do not call UART. |
| `CAL_EVENT_UART_FAIL` | `uart_link_send()` returns anything other than `HAL_OK` | `id=<payload[0]>`, `seq=<encoded frame[8]>`, `result=<HAL status>`; optional link-ready and UART counter fields can identify `HAL_ERROR` versus timeout. |

`result` in these records is a local diagnostic status, not a new payload
field. The CAL_EVENT wire payload remains `{ event_id }`. A `CAL_EVENT_TX` line
should be emitted after the UART call if it represents a completed HAL send;
the failure path should emit `CAL_EVENT_UART_FAIL` instead. If an attempted-TX
trace is desired, emit it before the call with an explicit local status
(`encoded`) and retain `CAL_EVENT_UART_FAIL` for the final HAL result.

The current `s3_service_send_boot_frame()` callback is `void`; changing that
public callback to return a HAL status would broaden the interface and is not
needed for these diagnostics. Logging the status at the service boundary keeps
the existing STM32/ESP32/App interfaces intact.

## 5. Requested modification record

This agent made no source modification.

If the main agent applies the minimum diagnostic change, the intended record
is:

| Item | Recommendation |
| --- | --- |
| Modification file | `STM32H757/Middleware/Communication/Services/s3_service.c` (the existing `s3_service_send_boot_frame()` function) |
| Modification reason | Expose encoder and UART outcomes for CAL_EVENT without changing protocol data or state transitions. |
| Modification content | Add bounded logs for `CAL_EVENT_TX`, `CAL_EVENT_ENCODE_FAIL`, and `CAL_EVENT_UART_FAIL`; derive event ID and sequence from the existing payload/encoded frame. |
| Potential impact | Small CPU/stack/log bandwidth increase on the boot event path; no sampling-rate, state-machine, ID, payload, CRC, or UART configuration change. Ensure log lines stay within the existing logger bound. |
| Verification | Static `rg`/diff checks for unchanged `0x0401` and one-byte payload; CM7 configure/build; then authorized UART capture confirming `AA 55 ... MSG_ID=01 04 ... FLAGS=01 ... PAYLOAD=01/02/03 ... CRC`, followed by matching five-byte ACK. Hardware behavior is otherwise unverified. |

## 6. Verification checklist

### Static/source checks

- [x] `send_cal_event()` resolves to the `SC_TYPE_CAL_EVENT` adapter path.
- [x] `SC_TYPE_CAL_EVENT` maps to `SCBP_MSG_ID_CAL_EVENT (0x0401)`.
- [x] V3 flags, source/destination, sequence, and CRC are produced by the
  existing encoder.
- [x] ACK correlation checks event message ID and sequence before invoking the
  boot manager callback.
- [x] `imu_runtime.c` has no duplicate CAL_EVENT sender.
- [x] `git diff --check` is clean for the inspected communication/runtime
  files. No build or hardware test was run in this read-only audit.

### Runtime/hardware checks still required

1. Observe `CAL_EVENT_TX id=1`, `id=2`, and `id=3` with distinct sequences and
   `result=HAL_OK`.
2. Capture each STM32-to-S3 V3 frame and verify ID `0x0401`, one-byte payload,
   ACK-required flag, and CRC16-MODBUS.
3. Capture the S3 ACK and verify ACK message ID `0x0401`, the same event frame
   sequence, source `0x02`, destination `0x01`, and result/error bytes.
4. Under an intentionally disconnected/not-ready UART (only in an authorized
   test), verify `CAL_EVENT_UART_FAIL` identifies the HAL status and that no
   protocol payload is altered.
5. Correlate the trace with the calibration state log. A missing TX line,
   UART failure, missing ACK, rejected ACK, and local VERIFY flag failure are
   distinct outcomes and must not be collapsed into one root cause.

