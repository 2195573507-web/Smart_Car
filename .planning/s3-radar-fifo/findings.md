# Findings

## Initial implementation inspected on 2026-08-29

- `ESPS3/main/radar/radar_uart.c` stored one validated frame in
  `s_latest_frame[]` under `s_latest_frame_mutex`.
- The parser callback used a zero-time mutex take. A busy mutex incremented
  `s_latest_frame_drop_count`; a new frame otherwise overwrote the previous
  frame and incremented `s_latest_frame_sequence`.
- `radar_uplink.c` polled every 20 ms with `radar_uart_get_latest_frame()` and
  suppressed a sequence already sent. This was the latest-only loss point.
- `radar_uplink_protocol.c` encodes the complete validated AA55 frame as the
  S3RD payload; no protocol change is required for a FIFO.

## Design decision

Use a fixed-size array of complete frame slots guarded by the existing mutex.
The producer increments a monotonic sequence and records the parser timestamp.
If full, it advances the tail and increments an explicit oldest-drop counter,
then writes the new frame. The consumer copies and removes one oldest slot so
TCP send latency cannot hold the UART task.

The queue depth remains small enough for internal RAM while absorbing short TCP
scheduling stalls. The implementation exposes the depth and counters in the
existing one-second radar diagnostics.

## Evidence boundary

This change can prove source/build/host behavior only. Real S3 Wi-Fi throughput,
TCP backpressure, and Windows whole-revolution assembly still require a flashed
device and live capture.

## 2026-08-29 follow-up: live FIFO saturation

- A BLE capture proved that the eight-frame FIFO can remain at depth 5-8 and
  that its oldest-drop counter grows before frames reach TCP or ROS.
- `radar_uplink_task()` previously waited the initial 500 ms retry interval
  before every TCP connection attempt, including immediately after a send
  failure. At the observed 115-140 validated frames/s, that delay alone can
  overflow the bounded FIFO.
- The sender now attempts a connection immediately, backs off only after a
  failed connection, drains at most four complete packets per scheduling pass,
  and emits throttled S3/BLE transmit diagnostics. The UART parser priority,
  FIFO depth, and S3RD fields remain unchanged.

## 2026-08-29 final optimization

- The FIFO is now 256 entries in PSRAM, approximately 200 KiB for the maximum
  complete-frame slot size. The queue remains bounded and still drops only the
  oldest entry when full.
- Frame arrival wakes the uplink task through a task notification. The UART
  parser never performs network I/O and takes the FIFO mutex without waiting,
  so a slow sender cannot hold the receive task for a scheduler tick.
- TCP remains non-blocking after connect. A bounded TX state machine preserves
  the packet offset across partial writes and `EAGAIN`; permanent socket errors
  reset the pending packet and trigger zero-packet resynchronization.
- Source and build evidence do not prove PSRAM availability at runtime,
  sustained Wi-Fi goodput, Windows receive behavior, or complete ROS2 scans;
  those still require a flashed device and correlated live capture.
