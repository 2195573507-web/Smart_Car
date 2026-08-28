# Findings: App Speed Command Dispatch

## Confirmed

- App-BLE wheel and chassis commands carry 16-byte payloads and therefore
  encode to 24-byte frames.
- Both app BLE managers currently call `writeValue` with the whole frame and
  do not query `maximumWriteValueLength(for: .withResponse)`.
- The S3 GATT service starts with ATT MTU 23 and its App parser accepts
  fragmented byte streams, so app-side write chunking is the missing transport
  adaptation.
- The current macOS percentage slider calls `updateSpeed()` -> `sendSpeed()`
  -> App `CONTROL/SPEED_CONTROL`; the active S3 bridge forwards only the
  current wheel/chassis command types and rejects the old path.

## Design

- Queue items carry a frame id and motion type. Each full frame is split into
  chunks, but coalescing removes only pending frames of the same motion type.
- All chunks belonging to the current in-flight frame are retained, so a new
  slider value cannot truncate a parser frame halfway through transmission.
- A write error drops the remaining chunks of the failed frame before the
  queue advances.

## Evidence Boundary

Source and host Swift builds can prove frame construction and queue behavior;
they cannot prove negotiated MTU, BLE delivery, S3 UART forwarding, ACKs, or
physical motor response.

## Follow-up Repair

- Both ViewModels now default and reset chassis straight-line speed to `0.0`
  and reset it to zero when entering chassis-diff mode.
- Chassis and wheel target updates use one 50 ms repeating timer per active
  mode, sending the latest target instead of resetting a one-shot debounce
  timer during a drag.
- `stop()`, `emergencyStop()`, and the macOS red stop button now use the full
  zero-wheel path. The BLE queue drops pending motion frames before enqueueing
  zero targets.
- The S3 bridge gates ACK-required motion transactions to one in flight and
  retains newest target and scale replacements independently. A zero-wheel
  command clears both pending replacements and uses a realtime stream frame,
  so it does not wait for the four-slot ACK pool.
