# S3 Radar Valid-Frame FIFO Uplink

## Goal

Replace the S3 radar uplink's single latest-frame slot with a bounded FIFO so
every checksum-valid YDLIDAR frame is offered to the TCP sender in order. Keep
the experimental S3RD framing and all UART1/GPIO44, BLE, UART2/SCBP-CAN, and
GPIO4 ownership unchanged.

## Scope

- Modify only the S3 radar UART/uplink implementation and focused host tests/docs.
- No Windows checkout edits, no STM32 edits, no protocol-field changes, no flash.
- Store complete frames in a bounded FIFO allocated from PSRAM; drop the oldest
  radar frame only when full, and never block the UART parser task on network I/O.

## Phases

- [completed] Inspect current latest-only ownership and define FIFO API/resource budget.
- [completed] Implement FIFO producer/consumer and diagnostics.
- [completed] Add host-testable FIFO behavior coverage.
- [completed] Run host tests, ESP-IDF build/size, and diff checks.
- [completed] Record evidence and remaining hardware/live-link gaps.

## Invariants

- YDLIDAR validation remains the admission gate.
- `device_id=1`, `stream_id=1`, `flags=0`, 26-byte little-endian S3RD header,
  and CRC16-Modbus remain unchanged.
- UART receive/parser task remains higher priority than uplink task.
- BLE raw UART logging remains disabled.

## Validation

- Host tests cover FIFO order, external storage, full-queue oldest-drop
  accounting, empty reads, sequence/timestamp preservation, and resumable
  partial TCP writes.
- ESP-IDF 5.5.4 build and `idf.py size` pass for default and uplink-enabled
  configurations where available.
- No claim of flashed-device, Wi-Fi, TCP, or ROS2 acceptance from build tests.
