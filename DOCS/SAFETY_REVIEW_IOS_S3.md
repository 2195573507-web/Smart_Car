# Safety Review: IOS_APP and ESP32-S3 Architecture

## Review Scope

Independent architecture review of the planned iOS and S3 boundary. This is a
static review; no code, device, wireless, UART, or vehicle test was run.

## Findings and Required Controls

| ID | Risk | Required control | Evidence needed later |
| --- | --- | --- | --- |
| S-01 | Phone/BLE/Wi-Fi loss leaves stale motion intent | S3 lease timeout and STM32 local command timeout, both fail safe | timed disconnect bench test |
| S-02 | Two command sources race | S3 admits one authority/session and rejects conflicting source IDs | arbitration test with App and ROS2 simulators |
| S-03 | Stop ACK mistaken for physical stop | expose accepted/rejected/unknown separately; STM32 reports actual safe state | bus capture and controlled stop test |
| S-04 | Reconnect resumes motion silently | reconnect requires new admission, fresh status, and explicit operator action | reconnect matrix |
| S-05 | Malformed/replayed packet | length, version, CRC, sequence, freshness, and identity checks at both boundaries | fuzz/property and replay tests |
| S-06 | Emergency stop is ordinary queued traffic | priority path, bounded queueing, and explicit latch/clear policy | latency and fault-injection test |
| S-07 | Backgrounded iOS scene keeps controls enabled | cancel input, request stop, invalidate local readiness on lifecycle change | iOS UI/lifecycle tests |
| S-08 | Future autonomy bypasses manual safety | autonomy is an authority mode, never direct PWM; manual/E-stop policy is explicit | ROS2/S3 arbitration test |
| S-09 | UART physical route is assumed valid | resolve the existing `PD3`/`PD4` mismatch before implementation | electrical review and loopback |

## Safety State Model

Minimum externally visible states are `DISCONNECTED`, `NEGOTIATING`, `READY`,
`STALE`, `STOPPING`, `ESTOP_LATCHED`, and `FAULT`. `READY` requires a current
session, compatible protocol, admitted authority, STM32 readiness, and fresh
status. Any ambiguous or stale condition suppresses motion and remains visible.

## Review Conclusion

The architecture is conditionally acceptable as a planning baseline if the
controls above become explicit protocol and implementation requirements. It is
not a safety acceptance result. The UART route, timeout values, electrical stop
path, and emergency-stop physical behavior remain open.

## Audit Provenance

The requested delegated safety-review call was attempted, but the agent
service returned `502 Bad Gateway` before execution. This document is therefore
the task-owner fallback under the same read-only architecture boundary and must
receive a fresh independent review before functional implementation.
