# Deprecated: A5/5A Smart_Car Unified Communication Protocol Plan

> **Deprecated.** This document describes an unimplemented A5/5A protocol
> proposal and must not be used for new SmartCar communication work. The
> implemented AA/55 protocol is specified exclusively in
> [SMARTCAR_PROTOCOL_V1.md](../../SMARTCAR_PROTOCOL_V1.md).

# Smart_Car Unified Communication Protocol Plan

## Status and Scope

This is a versioned protocol design target for L1 remote control and L3 ROS2,
SLAM, and navigation expansion. It is not an implementation or proof of BLE,
Wi-Fi, UART, vehicle, or ROS2 operation. The physical STM32H757 route remains
blocked until an authorized hardware-valid TX/RX pair replaces the documented
`PD3`/`PD4` mismatch.

## Layering

`APP <-> S3` uses the same logical envelope over a BLE characteristic or a
Wi-Fi stream/datagram adapter. `S3 <-> STM32` uses the envelope over the
approved low-level transport. Adapters must not reinterpret command meaning.

## Frame Format

All integer fields are little-endian. A frame is:

```text
SOF(2) | VERSION(1) | FLAGS(1) | HEADER_LEN(1) | TYPE(1) |
SOURCE(1) | AUTHORITY(1) | SESSION_ID(4) | SEQ(4) | ACK(4) |
TIMESTAMP_MS(4) | PAYLOAD_LEN(2) | PAYLOAD(N) | CRC16(2)
```

`SOF=0xA5 0x5A`; `HEADER_LEN` permits additive header fields. `VERSION` is
major/minor-compatible: an unknown major is rejected and an unknown optional
minor field is ignored only when its length is known. Payload length is capped
per transport capability and negotiated before control. CRC covers VERSION
through PAYLOAD and uses CRC-16/CCITT-FALSE, polynomial `0x1021`, initial
`0xFFFF`, no reflection, no final XOR. SOF and CRC are not included in the
CRC input.

## Message Types and Commands

| Type/CMD | Meaning | Authority/response |
| --- | --- | --- |
| `HELLO` | version, capabilities, identity | session negotiation |
| `SESSION_OPEN/CLOSE` | admit or end one source session | S3 is arbiter |
| `CMD_STOP` | normal zero-motion request | acknowledged, idempotent |
| `CMD_ESTOP` | emergency stop request | highest priority, latched per policy |
| `CMD_MANUAL_PAD` | bounded direction intent | L1 manual |
| `CMD_MANUAL_JOYSTICK` | bounded normalized linear/turn intent | L1 manual |
| `CMD_AUTONOMY_INTENT` | future bounded navigation intent | L3 only, never direct PWM |
| `HEARTBEAT` | liveness and lease renewal | expires command authority |
| `STATUS` | drive, gateway, fault, and telemetry state | source stamped |
| `ACK/NACK` | result correlated to `SEQ` | accepted/rejected/expired |
| `CAPABILITIES` | optional radar, telemetry, and L3 features | negotiated |

Every command carries `SESSION_ID`, `SEQ`, `SOURCE`, `AUTHORITY`, and a bounded
valid-until timestamp. Duplicate `SEQ` is idempotently answered; an older
sequence or expired lease is rejected. Motion values are normalized and range
checked by S3 and STM32 independently.

## Heartbeat and Timeout Plan

The active source sends heartbeat and command freshness within negotiated
intervals. S3 expires the source lease when heartbeats or valid commands stop,
emits a stop toward STM32, and reports `STALE_SOURCE`. STM32 independently
expires the last admitted motion command and enters its local safe output state.
Exact millisecond thresholds are commissioning parameters, not asserted values
in this planning document. `CMD_ESTOP` is not cleared by reconnect alone.

## Status and Fault Semantics

`STATUS` includes protocol/session identity, source authority, drive enable,
motion command freshness, link health, STM32 readiness, fault bitset, sequence
of last accepted command, and monotonic timestamp. Consumers must distinguish
`accepted`, `rejected`, `timed_out`, `stale`, `faulted`, and `unknown`; an ACK is
not evidence that a motor moved. Fault transitions are monotonic until the
owner-defined clear sequence and are never hidden by transport reconnect.

## L3 Extension

Reserve capability IDs and message types for map metadata, localization
quality, task lifecycle, route progress, scan/telemetry chunks, and bounded
autonomy intent. ROS2 messages are adapted at the gateway boundary; ROS2
topics, SLAM, and Nav2 are not placed on the S3-to-STM32 wire. L1 command IDs
remain stable and cannot be reinterpreted by L3.

## Security and Validation Gates

Pairing/identity, replay protection, encryption, Wi-Fi provisioning, BLE MTU
fragmentation, and key storage require a separate approved security design.
Before implementation: freeze field limits and error codes; resolve the
physical UART route; review safety ownership; then run parser/property tests,
transport simulation, bench tests, and controlled vehicle tests separately.

## Evidence Boundary

This document proves only a proposed contract. It does not prove CRC code,
packet interoperability, BLE/Wi-Fi latency, UART traffic, emergency-stop
timing, motor response, ROS2 integration, SLAM, navigation, or hardware safety.
