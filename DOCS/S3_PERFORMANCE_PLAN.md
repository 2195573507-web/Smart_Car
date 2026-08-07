# ESP32-S3 Performance and Resource Plan

## Scope

Planning targets for ESP-IDF gateway work only. Values below are budget gates
to measure and tune, not observed performance. No firmware or benchmark ran.

## Task and Core Budget

Use core affinity only where the final ESP-IDF target and Wi-Fi/BLE stack permit
it. Keep control admission, STM32 bridge, and safety timeout work bounded and
independent from radar and bulk telemetry. Candidate roles are:

| Work | Candidate priority/ownership | Rule |
| --- | --- | --- |
| Safety/session supervisor | high, short task | no blocking network or heap-heavy work |
| STM32 UART RX/TX bridge | high, bounded task/ISR handoff | queue frames, CRC outside ISR |
| BLE/Wi-Fi adapters | medium | backpressure and bounded buffers |
| Status publisher | medium | coalesce stale telemetry |
| Radar adapter | lower, isolated | never starve control or safety |
| Diagnostics/ROS bridge | lowest | rate limit and suspend under pressure |

The final plan must record actual task stack high-water marks, queue depth,
watchdog margins, and CPU load on both cores. Single-core fallback must remain
safe; no design conclusion may depend on a core that is unavailable in the
selected ESP-IDF configuration.

## Memory and Throughput Gates

- Reserve internal RAM for interrupts, control queues, and safety state.
- Use PSRAM for explicitly audited bulk radar/history buffers only after
  lifetime, cache, and DMA requirements are proven.
- Define maximum frame, queue, and concurrent-session counts before coding.
- Measure BLE notification/Write throughput and Wi-Fi goodput under telemetry
  load; control latency and loss must be reported separately.
- Measure UART frame rate, jitter, CRC rejection, and queue occupancy at the
  selected baud and frame size. Do not infer these from nominal baud rate.

## Acceptance Matrix

| Metric | Planning gate | Required evidence |
| --- | --- | --- |
| Control command latency | bounded p95/p99 under normal load | timestamped App/S3/STM32 trace |
| Stop latency | independent priority path | fault-injection trace |
| Heap headroom | no allocation failure during sustained load | allocator telemetry |
| Task starvation | no missed safety/bridge deadlines | runtime trace and watchdog report |
| BLE/Wi-Fi coexistence | control remains within gate with telemetry | repeatable transport test |
| Radar backpressure | bulk input cannot grow without bound | queue/PSRAM curve |

All numbers are to be frozen after hardware and target configuration are
known. A successful build, static task table, or nominal protocol rate is not
runtime performance evidence.
