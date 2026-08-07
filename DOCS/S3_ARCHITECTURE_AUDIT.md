# ESP32-S3 Architecture Audit

## Review Scope

This is an independent, static review of the architecture-only `ESPS3/`
scaffold created for the current initialization stage. The review checks the
S3 responsibility boundary, the STM32H757 interface boundary, the BLE seam,
and the reserved radar/ROS2 extension points against:

- `DOCS/SMART_CAR_SYSTEM_ARCHITECTURE.md`
- `DOCS/SMART_CAR_PROTOCOL.md`
- `DOCS/ESP32/S3_ARCHITECTURE.md`
- `DOCS/S3_PERFORMANCE_PLAN.md`
- `DOCS/DEVELOPMENT_INDEX.md`
- the files currently present under `ESPS3/`

No source, STM32H757, or iOS file was changed by this audit. This document is
static evidence only; it does not prove BLE, UART, ESP-IDF runtime behavior,
vehicle safety, radar compatibility, or ROS2 interoperability.

## Summary

The scaffold is conditionally acceptable as an architecture foundation. Its
module names and placeholders preserve the requested high-level S3 role and
do not move motor authority out of STM32H757. BLE, UART, protocol, heartbeat,
state, and safety behavior remain explicitly unimplemented, which is correct
for the current phase.

Two follow-up items must be closed before functional work:

1. The STM32-S3 UART physical route is still unresolved. The existing
   `PD3`/`PD4` record is not a valid STM32H757 UART TX/RX pair; the scaffold
   correctly avoids selecting pins or claiming a working bridge.
2. Radar is reserved by a dedicated directory, but there is no dedicated
   `ros_bridge` or `sensor_bridge` interface in the scaffold. The ROS2 path is
   currently documented only and should receive an explicit adapter seam before
   L2/L3 implementation.

## Findings

| ID | Area | Result | Evidence and assessment |
| --- | --- | --- | --- |
| S3-A-01 | S3 responsibility | **Pass** | `ESPS3/README.md` identifies S3 as the iOS BLE/Wi-Fi endpoint, validated UART gateway, and future radar/ROS2 ingress point. The README explicitly excludes motor control, radar parsing, ROS2, SLAM, and navigation. The `app`, `communication`, `bsp`, and `system` placeholders match the requested ownership layers. |
| S3-A-02 | STM32 boundary | **Pass with prerequisite** | `bsp_uart` reserves the hardware adapter without choosing pins, framing, DMA, or electrical settings. `app/safety` states that final motion interlocks remain with STM32H757. This agrees with the system architecture and protocol documents. The documented `PD3`/`PD4` mismatch remains a blocking hardware decision; no S3 code may treat the UART as operational until a valid route is approved. |
| S3-A-03 | BLE design | **Conditionally sound** | `bsp_ble` is isolated as a board-level adapter and its README explicitly defers GATT/session implementation, pairing, authentication, MTU handling, and notifications. This keeps transport separate from command/session logic. Before implementation, the BLE contract must define identity/pairing, one active control lease, fragmentation limits, freshness/sequence checks, disconnect behavior, and a control path independent of bulk diagnostics. |
| S3-A-04 | Protocol and safety seam | **Pass as placeholder** | `communication/protocol`, `packet`, `crc`, and `heartbeat` are separate from `app/command`, `app/state`, and `app/safety`. All C functions return `ESP_ERR_NOT_SUPPORTED`; no partial protocol behavior is being mistaken for a validated implementation. The eventual implementation must use the unified envelope and preserve S3 admission plus STM32 independent validation. |
| S3-A-05 | Radar extension | **Reserved** | `ESPS3/drivers/radar/README.md` reserves a sensor-ingress boundary and explicitly defers driver code, frame parsing, buffering, and ROS2 conversion. This prevents radar work from being coupled to phone-session or STM32 bridge code. L2 still needs an interface for timestamps, source health, bounded buffering/backpressure, and one-owner-per-physical-UART. |
| S3-A-06 | ROS2 extension | **Partial / follow-up required** | The S3 and system architecture documents reserve `S3 -> Wi-Fi -> ROS2_WIN`, and the top-level README mentions a future ROS2 gateway. However, the scaffold has no `ros_bridge`/`sensor_bridge` directory or header-level adapter contract. Add a reserved interface (without implementing ROS2) before L3 work so ROS2 logic cannot bypass S3 command admission or reach STM32 directly. |
| S3-A-07 | Architecture-document alignment | **Follow-up required** | The current scaffold follows the task-requested grouping (`components/bsp`, `communication`, `app`, `system`), while `DOCS/ESP32/S3_ARCHITECTURE.md` describes a future grouping (`gateway_core`, `control_protocol`, `ble_transport`, `wifi_transport`, `stm32_link`, `sensor_bridge`, `ros_bridge`, `diagnostics`). This is not a runtime defect at the scaffold stage, but the authoritative mapping should be updated before implementation to prevent two competing component taxonomies. |

## Boundary Checks

### S3 versus STM32H757

The direction is correct: S3 owns phone-facing sessions, admission, protocol
framing, and gateway state; STM32H757 retains low-level actuator execution,
encoder/IMU handling, local interlocks, and final motion authority. The current
stubs do not expose PWM, encoder, motor, or STM32 control algorithms. UART is a
reserved interface only and is not configured around the known invalid pin
record.

### BLE and control sessions

The BLE adapter is correctly kept below the application/session layer. The
scaffold does not yet define a GATT profile or security policy, so no claim of
connectivity, authenticated control, MTU behavior, or stop-on-disconnect can be
made. Those requirements are consistent with the protocol security gates and
must be specified before adding transport behavior.

### Future radar and ROS2

Radar has a physical source-tree reservation and an explicit non-implementation
boundary. ROS2 has a documented ownership route but not yet a source-level
adapter seam. Both future paths must remain data/intent adapters: ROS2 may
submit bounded, authorized autonomy intent, while neither radar nor ROS2 may
write motor-level commands or bypass the S3 safety/session gate.

## Required Next Gates

1. Approve and document a hardware-valid STM32H757-to-S3 transport route and
   electrical settings; keep the current UART placeholder inert until then.
2. Freeze the protocol version, limits, CRC test vectors, sequence/freshness,
   heartbeat lease, ACK/NACK, and emergency-stop semantics before coding.
3. Define BLE pairing/authentication, GATT characteristics, MTU fragmentation,
   reconnect, and single-controller admission behavior.
4. Add reserved `sensor_bridge` and `ros_bridge` interface contracts with
   bounded queues, timestamps, health/fault state, and explicit ownership.
5. Reconcile the planned component taxonomy with the implemented scaffold and
   then run ESP-IDF build/static checks. A successful build remains build
   evidence only, not hardware or vehicle acceptance.

## Audit Conclusion

**Architecture status: conditionally acceptable for the requested scaffold
stage.** The current files preserve S3 as the high-level gateway and preserve
STM32H757 as final motion authority. Functional implementation is not yet
authorized by this audit; the UART hardware prerequisite, BLE security/session
contract, and explicit ROS2 adapter seam remain required before the next phase.

