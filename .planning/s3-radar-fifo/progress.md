# Progress

## 2026-08-29

- Reviewed the current S3 radar parser, UART frame slot, uplink task, protocol
  codec, tests, and project status.
- Confirmed the latest-only loss boundary is the single frame slot in
  `radar_uart.c`, not S3RD encoding.
- Confirmed the requested change must not alter UART1/GPIO44, GPIO4 PWM,
  UART2/SCBP-CAN, BLE command semantics, or S3RD fields.
- Added `radar_frame_fifo.[ch]` with eight fixed complete-frame slots,
  oldest-drop accounting, order/metadata preservation, and non-consuming short
  buffer behavior.
- Replaced the UART latest-frame accessor with a mutex-protected FIFO pop API;
  the uplink now sends each queued frame with its original parser timestamp.
- Added FIFO depth/high-water/oldest-drop/lock-drop diagnostics and retained the
  disabled raw UART BLE logging switch.
- Added host tests for FIFO order, metadata, full-queue oldest-drop behavior,
  empty reads, and short-buffer non-consumption.
- Verified host tests, ASAN/UBSAN FIFO tests, `git diff --check`, ESP-IDF 5.5.4
  uplink-enabled build, and `idf.py size`.
- Added a pending TCP packet retry so an interrupted send is retried after
  reconnect before dequeuing another frame.
- Reduced the low-priority uplink polling interval from 20 ms to 5 ms so the
  sender does not artificially cap normal X3PRO packet bursts at 50 frames/s.
- Updated the X3PRO receive/uplink documentation and ROS2 radar progress notes;
  explicitly retained the source/build versus flashed-device/live-link boundary.
- Follow-up after a live BLE capture showed `q=5..8` and increasing oldest-drop
  counts. Changed the sender to connect immediately, delay only after failed
  connections, and drain a bounded four-packet burst. Added one-second
  `RADAR_UPLINK_STATS` counters for complete sends/bytes, send timeout/failure,
  connect failures/successes, last sent sequence, and maximum dequeue age.
- Re-ran the radar host tests, ESP-IDF 5.5.4 build, image-size check, and
  `git diff --check`. No flash, Wi-Fi/TCP capture, or ROS2 validation was run.

## 2026-08-29: S3 PSRAM and non-blocking TX optimization

- Replaced the fixed eight-slot inline FIFO storage with a 256-entry FIFO whose
  complete-frame entries are allocated with `heap_caps_calloc()` using
  `MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT`. Allocation failure is reported and
  does not silently fall back to a smaller queue.
- Changed the parser/uplink handoff to FreeRTOS task notifications and a zero-
  wait FIFO mutex. UART DMA/read buffers and task stacks remain in internal RAM.
- Kept the TCP socket non-blocking after connect. Added a host-testable TX
  state machine that preserves packet offset across partial writes and
  `EAGAIN`, with a bounded 16-call send budget per attempt.
- After initial TCP connect or reconnect, the sender discards queued non-zero
  frames until the next YDLIDAR zero packet, preventing a partial revolution
  from being joined to the next one. Diagnostics now report send waits,
  partial writes, retries, resync drops, and encode failures.
- Re-ran host tests, ASAN/UBSAN, ESP-IDF 5.5.4 `idf.py build`, `idf.py size`,
  and `git diff --check`; no flash or live Wi-Fi/TCP/ROS2 validation was run.
