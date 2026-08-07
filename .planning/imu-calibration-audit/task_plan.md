# IMU/Radar Calibration Audit Plan

## Goal
Complete twelve independent read-only expert audits, synthesize the root cause,
then implement only the lowest-risk STM32H757 + ESP32-S3 startup calibration
fixes authorized by the user. No flash, hardware connection, interface, UUID,
AA55 framing, or CRC changes.

## Phases
- [x] Record user waiver of unavailable specialist Agent workflow
- [x] Reconcile state, sampling, RMS, protocol, S3, UART, RTOS, logging, attitude, architecture, and regression evidence
- [x] Select the user-authorized minimum-risk implementation design
- [x] Implement scoped fixes with exact-once progression and heartbeat changes
- [x] Run clean STM32 CM7 build and ESP32-S3 build only
- [x] Final report with evidence boundaries and no-flash confirmation

## Status
Read-only audit phase restarted on 2026-08-07. A previous partial attempt did
not collect all twelve independent handoffs, so all requested roles are being
run again. Firmware source remains unchanged until Agent 12 completes.

On 2026-08-07 the user explicitly waived the multi-Agent requirement after the
collaboration dispatcher could not be used. The primary agent is authorized to
complete the source audit, select the minimum-risk repair, implement it, and
run build-only verification without further Agent reports.

Implementation and build-only verification are complete. The final source
review confirmed a 112-second theoretical minimum calibration window plus
protocol overhead, no periodic PING sender, protocol compatibility, and no
hardware-facing changes. No flash, monitor, serial, BLE, or hardware operation
was performed.

## Errors

| Error | Attempts | Resolution |
| --- | ---: | --- |
| Agent dispatch called without required role payload | 2 | No agent was created; retry with the collaboration namespace and complete message. |
| Combined read-only inspection script had a JavaScript quoting error | 1 | No shell command ran; split the checks into independent calls. |

## Agent Status

| Agent | Scope | Status |
| --- | --- | --- |
| 1-12 | Requested specialist roles | Waived by user after dispatcher failure |
