# App-BLE V2 Findings

## Confirmed Current Facts

- App BLE V1 is `AA | 01 | TYPE | LEN_LE | PAYLOAD | CRC16-MODBUS | 55`
  with a maximum payload of 128 bytes.
- FFE1 is the app write characteristic, FFE2 is telemetry notification, and
  FFE3 remains the separate log notification path.
- Existing nonzero motion commands already reach STM32 as SCBP-CAN
  ACK-required transactions. The all-zero wheel tuple is an immediate
  non-transactional safety stop.
- S3 already coalesces pending target/scale motions and serializes a BLE
  disconnect into a zero-wheel SCBP frame.
- The two Swift apps currently carry equivalent protocol, BLE, telemetry, and
  scheduling source files, which creates future behavior-drift risk.

## Design Decisions

- V2 uses outer App-BLE version `0x02` but retains the V1 outer framing and
  the FFE0/FFE1/FFE2/FFE3 GATT contract.
- V2 command payloads are wrapped with a session id, monotonic command
  sequence, a receiver-relative validity duration, and the preserved V1
  command type/payload.
- The S3 assigns the session id and maps completion to the existing STM SCBP
  ACK/ERROR path. STM32 needs no new App-BLE parser or authority change.
- App startup attempts V2 within a bounded interval, then enters explicit V1
  fallback for older S3 firmware. A connected-but-unnegotiated V2 link never
  emits nonzero V2 motion.

## Implementation Evidence (2026-08-23)

- Shared `SmartCarAppCore` now owns V1/V2 framing, bounded stream parsing,
  session state, fixed-capacity outbound scheduling, and a bounded ring buffer.
- Both BLE managers preserve one `.withResponse` ATT fragment in flight while
  the shared scheduler coalesces unsent motion and prioritizes zero/stop.
- Session expiry is exposed as an explicit `.expired` state; the Apps clear
  unsent motion and queue a best-effort V1 zero-wheel stop.
- S3 V2 commands validate session, sequence, validity window, and preserve the
  original V1 payload. The latest completed ACK is cached for idempotent retry;
  stale older sequences are rejected.
- S3 motion commands carry the receiver-relative deadline through the pending
  queue and return `EXPIRED` if downstream admission occurs too late.
- `DOCS/protocol/app-ble-protocol-v2.md` is now the canonical GATT, frame,
  payload, mapping, safety-boundary, and test-vector document.
