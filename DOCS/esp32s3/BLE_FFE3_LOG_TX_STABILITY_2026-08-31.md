# ESP32-S3 BLE FFE3 Log TX Stability

Status: `SOURCE/BUILD CONFIRMED`, `DEVICE/10-MINUTE SESSION UNVERIFIED`

Date: 2026-08-31

## Scope

This change isolates SmartCarLog traffic on FFE3 behind one low-priority TX
worker. It does not change SmartCarLog bytes, FFE1 command semantics, FFE2
notification bytes or priority, SRP, radar UART/S3RD, Wi-Fi/TCP, vehicle
control, or safety behavior. No device was flashed during implementation.

## Confirmed Baseline

- A SmartCarLog frame is at most 108 bytes: 12 bytes of envelope/CRC and up to
  96 bytes of payload.
- With the default ATT MTU of 23, one maximum frame needs six notifications.
- The previous FFE3 CCC callback could synchronously flush up to 48 pending
  records.
- `log_bridge_handle()` and multiple S3 tasks could reach the FFE3 GATT send
  path concurrently.
- The prior pending array held 48 text records but had no congestion gate or
  single TX owner.

## Implemented Design

```text
log_bridge / S3 tasks / radar diagnostics / CCC markers
                         |
                         | validate + copy complete SmartCarLog frame
                         v
             +---------------------------+
             | critical FIFO: 8 frames   | WARN / ERROR
             | normal FIFO: 40 frames    | DEBUG / INFO
             +---------------------------+
                         |
                         | fixed storage, drop oldest per full queue
                         v
              low-priority FFE3 TX worker
                         |
                         | one complete frame at a time
                         | ATT chunks separated by 1 RTOS tick
                         v
              esp_ble_gatts_send_indicate(FFE3)
```

Only the worker calls `esp_ble_gatts_send_indicate()` for the FFE3 value
handle. Producers never fragment or wait for BLE. `ESP_OK` from
`s3_ble_log_notify_send()` means the complete frame was accepted into fixed
storage; it is not delivery acknowledgement.

The worker selects critical frames first. After four consecutive critical
frames it sends one waiting normal frame, preventing permanent INFO/DEBUG
starvation while preserving WARN/ERROR preference. FIFO order is retained
inside each queue. A full queue drops its own oldest frame, accepts the new
frame, and increments the matching drop counter.

## Connection and Congestion State

- `ESP_GATTS_CONGEST_EVT(congested=true)` pauses chunk preparation without
  polling or discarding the active frame.
- `congested=false`, connect, and FFE3 CCC enable notify the worker.
- FFE3 CCC handling only creates bounded BOOT/BLE_CONNECTED/previous-disconnect
  records, updates state, and wakes the worker. It does not flush in a loop.
- Disconnect aborts the active frame and increments `partial_drop` when any
  part may already be in flight. Complete frames still waiting in either FIFO
  remain queued.
- The latest disconnect reason and a saturating cumulative count are retained.
  The next FFE3 enable queues `BLE_PREV_DISC reason=0xNN count=N` as WARN.

## Read-Only Statistics

`s3_ble_get_log_notify_stats()` returns saturating counters:

`queued`, `sent_frames`, `sent_chunks`, `drop_normal`, `drop_critical`,
`send_fail`, `congest_events`, `partial_drop`, `current_depth`, and
`high_watermark`.

`current_depth` counts complete frames that have not started; it excludes the
active frame. `s3_ble_get_disconnect_info()` returns `valid`, the latest
disconnect `reason`, and cumulative `count`. The compatibility getter
`s3_ble_get_notify_fail_count()` still counts FFE2 plus FFE3 lower-level GATT
submission failures; queue drops and readiness failures are not added to it.

## RAM, Stack, and Timing

| Item | Impact | Boundary |
| --- | ---: | --- |
| Fixed queue/state object | 5456 bytes | 40 normal + 8 critical + one active frame + counters |
| Previous pending frame array | about 5184 bytes | 48 records at about 108 bytes each |
| Net fixed-state growth | about 272 bytes, plus small handles/flags | Same 48-frame storage order of magnitude |
| Worker stack | 3072 bytes from FreeRTOS heap, plus TCB | Created once by `s3_ble_init()` |
| Worker local chunk copy | about 120 bytes | Keeps GATT input stable outside the queue lock |
| Producer critical section | at most one 108-byte copy | No queue wait; encoding and CRC run before the lock |

The worker priority is `tskIDLE_PRIORITY + 1`, below the current radar uplink,
smartcar service, radar UART, and STM UART RX tasks. FFE2 stays on its existing
path. With `CONFIG_FREERTOS_HZ=100`, the one-tick chunk delay is 10 ms. A
sustained source faster than BLE can drain is expected to produce counted
oldest-frame drops instead of controller bursts or control-task blocking.

## Host and Build Verification

| Check | Result | Evidence level |
| --- | --- | --- |
| Pure-C FIFO/priority/drop/fairness/fragment/disconnect/congestion/counter tests | PASS | Host |
| Same tests with `-Wall -Wextra -Werror` | PASS | Host |
| Same tests with ASAN + UBSAN | PASS (`detect_leaks=0`; Apple ASAN has no leak mode) | Host |
| Existing S3 radar host tests | PASS after change | Host |
| `__idf_s3_ble` component build | PASS; CMake emitted a non-fatal esp_rom gdbinit warning | Build |
| ESP-IDF 5.5.4 full `IDF_BUILD_JOBS=2 idf.py build` | PASS | Build |
| `idf.py size` | PASS; DIRAM 178687/341760 bytes (52.28%), `.bss` 55592 bytes | Build |
| Target-file `git diff --check` | PASS | Static |

These checks do not prove BLE controller behavior, phone reception, connection
stability, or vehicle acceptance.

## Build Artifact

- BIN: `ESPS3/build/smartcar_s3_gateway.bin`
- BIN size: 1234464 bytes (`0x12d620`); smallest app partition has 83% free.
- BIN SHA-256: `d9df9137fac376c3bbd7b6079f3267ddbe4fafc1f2b80889629cc6d425796254`
- ELF: `ESPS3/build/smartcar_s3_gateway.elf`
- ELF SHA-256: `95988c85fa0aecc646f52ec8d12d441b3fec9ff05707c8161784e2d28d6a3ed7`

## Required Post-Flash Evidence

Use the generated S3 BIN only after the operator authorizes flashing. Capture
one timestamped serial/App session containing all of the following:

1. Connection start, negotiated MTU, FFE2 CCC, and FFE3 CCC enable.
2. `BLE_PREV_DISC reason=0xNN count=N` after any reconnect, plus the raw S3
   `ESP_GATTS_DISCONNECT_EVT` reason from the same session.
3. Start/end snapshots from `s3_ble_get_log_notify_stats()` and
   `s3_ble_get_disconnect_info()`, including queue depth, high-watermark,
   normal/critical drops, congestion events, send failures, and partial drops.
4. At least 10 continuous minutes with FFE3 enabled and representative STM/S3
   log load. Record whether any reconnect occurred and preserve phone-side FFE3
   frame/CRC/reassembly errors.
5. During the same window, preserve evidence that FFE2 ACK/status delivery,
   BLE disconnect stop handling, STM SRP service, radar sequence progression,
   and Wi-Fi/TCP tasks remain responsive. This is regression evidence, not
   authorization for vehicle motion.

Acceptance requires no unexpected disconnect during the 10-minute window, no
interleaved/corrupt FFE3 frame, bounded queue/drop behavior under the applied
load, and no regression in the independent FFE2 or safety paths.
