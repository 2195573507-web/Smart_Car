# CM7 Scheduler Probe Design

## Goal

Separate scheduler/task execution from IMU, attitude, chassis, and SRP sync
admission failures without changing production behavior.

## Design

`main.c` will expose a CMake-controlled `SMARTCAR_SCHEDULER_PROBE` option,
defaulting to `OFF`. When enabled, it creates `scheduler_probe` at priority
`configMAX_PRIORITIES - 1`. The task uses static buffers, `vTaskDelay(50 ms)`,
the canonical `srp_encode_frame()` codec, and `uart_link_send()` to transmit a
valid SRP `0x30` log envelope. It never sends a motor/control message and does
not alter the normal sync gate.

Before scheduler start, `main.c` records a stage marker and a compact task
admission/heap snapshot. The first probe iteration records `IPSR`, scheduler
state, `BASEPRI`, heap, and send counters; subsequent iterations only transmit
the fixed diagnostic frame and periodically emit a bounded health line.

`s3_service_handle_sync_req()` records the execution context of the response
path (`IPSR` and scheduler state). This confirms whether the current image
handles sync in task context and avoids inferring ISR behavior from waveform
timing alone.

## Safety and Compatibility

The option is off in production builds. The probe bypasses only the telemetry/
log admission gate in the explicitly named diagnostic image; it cannot carry a
motion command and leaves `motor_board_force_stop()` and attitude readiness
unchanged. Existing SRP v4 bytes, UART2 pins, and S3 BLE conversion remain
untouched.

## Verification

Build the default CM7 image and an isolated `SMARTCAR_SCHEDULER_PROBE=ON` image,
run `git diff --check`, and build ESPS3 from an isolated ESP-IDF directory. On
hardware, flash matching probe images and verify a first response followed by
regular D2 bursts at 50 ms, increasing S3 RX counters, and FFE3 log delivery.
