# ROS Motion Control Protocol v1

Status: **FROZEN for the ESP32-S3 implementation (2026-09-03)**

This protocol is a dedicated S3-to-Windows ROS control session. It is not
S3RD, does not use TCP port 8765, `radar_uplink`, the radar FIFO, or radar
bandwidth. The S3 is the TCP client and actively connects to the configured
Windows host on TCP port **8766**.

## Scope and authority

The protocol carries a bounded body-speed intent and selected STM telemetry.
The ESP32-S3 validates the session and lease, then hands accepted commands to
the existing `smartcar_service` owner. Only that owner may encode/send SRP.
STM32H757 remains the final motion authority and continues to validate SRP,
apply local interlocks, and drive the MotorBoard.

The ROS command is two-axis chassis speed:

| Input | Unit | SRP 0x06 mapping |
| --- | --- | --- |
| `linear_v` | m/s | payload bytes 0..3, `linear_v * 1000` mm/s |
| `angular_w` | rad/s | payload bytes 4..7, unchanged rad/s |
| reserved | none | bytes 8..15 must be zero |

The default S3 limits are `abs(linear_v) <= 0.10 m/s` and
`abs(angular_w) <= 0.30 rad/s`. They are parameterized by Kconfig but the
defaults are safety limits, not a vehicle acceptance claim.

## Wire frame

All integer fields are little-endian. A complete frame is at most **256 bytes**.
The fixed header is 24 bytes; the maximum payload is 214 bytes.

```text
magic u16 | version u8 | type u8 | flags u16 | payload_len u16 |
session_id u32 | sequence u32 | lease_id u32 | ttl_ms u16 | reserved u16 |
payload[payload_len] | auth_tag[16] | crc16 u16
```

- `magic = 0x4D52` (wire bytes `52 4D`, ASCII `RM`).
- `version = 1`.
- `flags` currently accepts only `AUTH_PRESENT = 0x0001`.
- `payload_len` must be `0..214`; the declared total must be exactly received.
- `reserved` and all unknown flag bits must be zero.
- `auth_tag` is the first 16 bytes of HMAC-SHA256.
- CRC is CRC-16/CCITT-FALSE (`poly=0x1021`, `init=0xFFFF`, no reflection,
  xorout 0) over bytes `version` through the end of `auth_tag` (offset 2 to
  `24 + payload_len + 16`). Magic and CRC are excluded.

## TCP framing and parser behavior

TCP is a byte stream. One `recv()` may contain a partial frame, several frames,
or a frame boundary at any byte. The receiver accumulates at most 256 bytes,
uses `payload_len` only after the fixed header is present, and dispatches only
when the exact frame length is available. An invalid magic, version, length,
flags, reserved field, HMAC, or CRC is rejected. The parser then searches for
the next `RM` magic without retaining an unbounded buffer.

No command queue is permitted. The S3 keeps one latest command slot; a newer
accepted command overwrites the previous slot before it is handed to the
serialized `smartcar_service` task.

## Message types

| Type | Name | Direction | Payload |
| ---: | --- | --- | --- |
| `0x01` | `HELLO` | S3 -> Windows | empty |
| `0x02` | `HELLO_ACK` | Windows -> S3 | empty |
| `0x03` | `LEASE_REQUEST` | Windows -> S3 | empty |
| `0x04` | `LEASE_RESPONSE` | S3 -> Windows | `status u8`, `lease_ms u16` |
| `0x05` | `MOTION_CMD` | Windows -> S3 | `linear_v f32`, `angular_w f32` |
| `0x06` | `STOP` | Windows -> S3 | empty; explicit stop |
| `0x07` | `STATUS` | S3 -> Windows | `status u8`, or `message_id u8 (0x15) | encoded SRP v4 frame` |
| `0x08` | `ERROR` | S3 -> Windows | `error u8` |
| `0x09` | `HEARTBEAT` | Windows -> S3 | empty |

`sequence` is a strictly increasing 32-bit value within a session, using
serial-number arithmetic for wraparound. `HELLO_ACK` starts a new S3 session
identifier; every later Windows frame must echo that `session_id`. A duplicate
or older sequence is an error and causes an immediate safety stop.

## Session, lease, and validity

1. After TCP connect, S3 sends `HELLO` with a new non-zero `session_id`.
2. Windows always replies `HELLO_ACK` with `lease_id = 0`. If Windows motion
   authorization is enabled, it then sends `LEASE_REQUEST` with a new non-zero
   `lease_id`; S3 grants exactly one lease for that TCP session. If motion
   authorization is disabled, the session is telemetry-only: Windows has no ROS
   lease and sends no `LEASE_REQUEST`, `HEARTBEAT`, `MOTION_CMD`, or `STOP`.
   A telemetry-only session may still receive a valid `STATUS (0x15)` payload.
3. A telemetry-only session can be armed at runtime only by Windows explicitly
   sending a new `LEASE_REQUEST`; it remains ineligible to send a motion command
   until a valid `LEASE_RESPONSE` is received. Motion commands always require
   the current valid lease.
4. The lease TTL is 20..220 ms; the S3 default is 220 ms. Each accepted motion
   command refreshes the lease deadline only within that bound. Heartbeats may
   refresh liveness but never create a lease.
5. `MOTION_CMD` requires the current session, current lease, a newer sequence,
   finite values within the configured limits, and `ttl_ms` in 20..220 ms.
6. `STOP` is explicit and accepted only for the current session/lease. It
   clears the latest command and sends a zero-speed SRP command.
7. Lease expiry, malformed/unauthenticated input, sequence disorder, TCP
   disconnect, S3 restart, ROS process exit, or STM/S3 link loss revokes the
   lease and sends zero speed. The gateway rechecks the lease before and after
   its bounded 25 ms socket wait and 20 ms telemetry send, while the service
   task consumes stop requests every tick; the S3-to-STM zero frame is
   scheduled within **200..250 ms** of the event under those software
   scheduling assumptions.
8. After any revoke, recovery requires a new TCP handshake and a new lease;
   old session, lease, sequence, and command data are never replayed.

ROS lease validity is the single motion-control gate. While it is valid, BLE
non-zero motion commands are rejected. BLE explicit stop is always allowed and
has priority; BLE disconnect, STM BUS_OFF, and S3-STM SRP timeout also clear
the ROS lease before stopping.

## Authentication and configuration

When the feature is enabled, both directions use HMAC-SHA256 truncated to 16
bytes with a pre-shared key (PSK). The PSK is supplied only through local
ESP-IDF configuration (`sdkconfig`/secret provisioning) and is never printed,
serialized into diagnostics, or committed. Empty PSK or empty Windows host
configuration prevents the feature from starting. The Windows peer must be on
the explicit allow-list/ACL for the deployment; this v1 wire protocol does not
claim TLS or Internet exposure protection. SSID, host/IP, and PSK are likewise
never written to logs.

For v1, the PSK is one non-empty ASCII text line. The HMAC key is the original
ASCII bytes after removing one optional final CRLF or LF. Even if the text looks
hexadecimal, implementations must not hex-decode it, change its case, or trim
it.

## Status and telemetry

`STATUS` reports lease/session state and bounded error codes. The existing
validated SRP `CHASSIS_STATE (0x15)` frame is copied through the secondary
`smartcar_service` telemetry sink and sent as a latest-only `STATUS` payload to
Windows. Its telemetry payload is exactly `message_id u8 (0x15) | encoded SRP v4 wire frame`;
the prefix is not part of the SRP frame. Its 0x15 layout, SRP CRC, and source
path remain unchanged; Windows may use it as the `/odom` input. This telemetry path is
independent of radar S3RD and port 8765.

## Fixed error/stop rules

| Event | S3 action |
| --- | --- |
| CRC/HMAC/length/version/flags error | reject, best-effort `ERROR`, close TCP, revoke lease, zero |
| duplicate/older sequence | best-effort `ERROR`, close TCP, revoke lease, zero |
| invalid session/lease or out-of-range speed | best-effort `ERROR`, close TCP, revoke lease, zero |
| command/lease deadline elapsed | best-effort `STATUS`, close TCP, revoke lease, zero |
| TCP EOF/reset or Wi-Fi loss | close socket, revoke lease, zero |
| STM BUS_OFF or SRP receive timeout | existing command bridge recovery, revoke lease, zero |
| S3 reset/power-up | service startup submits zero; no lease; handshake required before motion |

This document freezes the S3 implementation contract. It does not prove
Wi-Fi, TCP, UART, STM, ROS2, `/odom`, or vehicle behavior until a matching
flashed-image bench capture is completed.
