# iOS Architecture Audit

## Scope and Evidence Boundary

This is an independent, read-only audit of the initialized `IOS_APP` source
against `SMART_CAR_SYSTEM_ARCHITECTURE.md`, `IOS_APP_ARCHITECTURE.md`,
`SMART_CAR_PROTOCOL.md`, and `SAFETY_REVIEW_IOS_S3.md`. Only the audit file is
created by this agent. No implementation, firmware, UUID, or hardware files
were changed.

The source is an iOS/SwiftUI scaffold. It is not evidence of BLE
interoperability, S3 command admission, motor output, emergency-stop timing,
or vehicle safety.

## Positive Findings

- SwiftUI views are separated from `RemoteViewModel`, and protocol types do
  not import SwiftUI or CoreBluetooth.
- `BLEManager` is behind a `VehicleTransport` protocol and BLE UUIDs are
  runtime configuration values; no unverified hardware UUID is hard-coded.
- `VehicleLinkState` includes the important safety states (`ready`, stale,
  stopping, disconnected, fault, and emergency-stop-latched).
- Manual and emergency intents are distinct, and an autonomy command identifier
  is reserved rather than implemented as a motor-control path.
- The packet encoder uses little-endian fields, the documented SOF, and
  CRC-16/CCITT-FALSE parameters.
- L3 ownership remains outside the app: ROS2, SLAM, navigation, radar parsing,
  and low-level actuator control are not placed in this scaffold.

## Findings

### A-01: Transport abstraction is BLE-specific (High)

Evidence: `IOS_APP/SmartCarApp/Core/Service/VehicleTransport.swift:3-11`
exposes `BLEState` and `BLEDevice` directly. This couples
the domain-facing service to CoreBluetooth naming and makes a future Wi-Fi,
test, or ROS2-gateway adapter depend on BLE types. Introduce protocol-neutral
transport state and vehicle identity types; keep BLE mapping inside the BLE
adapter. Preserve one active command-bearing transport per session.

### A-02: Session and status flow is not wired (High)

Evidence: `IOS_APP/SmartCarApp/Core/UI/RemoteView/RemoteViewModel.swift:4-25`
only sends packets and never consumes `incomingPackets`; there is
no STATUS/ACK mapper, HELLO, SESSION_OPEN, capability negotiation, heartbeat,
session admission, or command outcome correlation. Consequently, `ready` is
never established from gateway evidence and manual input is blocked by the
fresh-status/readiness guard. Add an explicit asynchronous session state
machine and map gateway outcomes to accepted, rejected, timed-out, stale,
faulted, and unknown states.

### A-03: BLE discovery/connection lifecycle is incomplete (High)

Evidence: `IOS_APP/SmartCarApp/Core/Bluetooth/BLEManager.swift:30-42`:
`didDiscover` stores only a value object and `connect(to:)` calls
`retrievePeripherals(withIdentifiers:)`. A peripheral returned by discovery is
not retained for connection, so a newly discovered device may not be
retrievable. Retain the `CBPeripheral` by identifier and handle failed
connection, service/characteristic errors, notification errors, reconnect, and
the distinction between connected and protocol-ready.

### A-04: Control release and lifecycle stop semantics are incomplete (High)

Evidence: `IOS_APP/SmartCarApp/Core/UI/RemoteView/DirectionPadView.swift:3-6`,
`JoystickView.swift:15-26`, and `RemoteViewModel.swift:14-20`. The direction
pad emits a one-shot command and does not model press/hold or
release-to-stop. The joystick sends on drag updates but only stops in
`onEnded`; cancellation, app backgrounding, focus loss, and mode changes are
not centrally handled. Implement a single control coordinator that cancels
input, sends normal stop, and invalidates local readiness for every lifecycle
or transport transition. Emergency stop must remain available independently of
manual readiness.

### A-05: Packet decoder narrows the versioning contract (Medium)

Evidence: `IOS_APP/SmartCarApp/Core/Protocol/Packet.swift:54-68`. The protocol
allows additive header fields when their length is known, but
`PacketDecoder` rejects every `HEADER_LEN` other than the current fixed value.
It also drops decoded `flags`, `source`, and `acknowledgement` values by
constructing a packet with defaults. Parse and retain those fields, accept
known additive headers, and reject only unknown/unsafe layouts.

### A-06: Packet bounds and command contract are incomplete (Medium)

Evidence: `IOS_APP/SmartCarApp/Core/Protocol/Packet.swift:30-40`,
`Message.swift:3-13`, and `Command.swift:3-16`. `encoded()` converts payload
size directly to `UInt16` without enforcing the
negotiated transport limit. The app has no payload-bound error path. The
message mapper also lacks HELLO/session/heartbeat/ACK handling and does not
carry an explicit valid-until/lease value required by the protocol plan.
Freeze command IDs, field limits, error vocabulary, and negotiated BLE MTU
before hardware integration; reject oversized payloads locally.

### A-07: Status freshness is local-only (Medium)

Evidence: `IOS_APP/SmartCarApp/Core/Model/VehicleState.swift:17-33`.
`VehicleStatus.hasFreshStatus` uses a hard-coded two-second wall-clock check.
The protocol requires gateway source identity, session identity, command
freshness, and a monotonic timestamp. Store receive time plus gateway metadata,
make the freshness threshold negotiated/configured, and ensure stale status is
visible and suppresses motion.

### A-08: Device list identity is unstable (Low)

Evidence: `IOS_APP/SmartCarApp/Core/Bluetooth/BLEDevice.swift:3-12` and
`BLEManager.swift:37`. `BLEDevice.Equatable` includes RSSI. A changing RSSI therefore makes the same
device compare unequal and can append duplicates. Deduplicate by UUID and
update the RSSI/name record.

### A-09: Test and target evidence is absent (Medium)

Evidence: `IOS_APP/Package.swift:1-16`; `swift build` succeeds for the SwiftPM
package in the available environment.
`swift test` reports that no tests exist. The package target is not an Xcode
iOS runtime proof; CoreBluetooth behavior, SwiftUI lifecycle behavior, packet
round trips, transport simulation, and S3 compatibility remain unverified.

The workspace has no root `.git` metadata, so a repository-level `git diff`
or commit-scope check cannot be performed. File manifests and targeted static
inspection are the available change evidence.

## L3 Readiness Assessment

The ownership boundary is directionally correct: the app can remain an
operator/supervision client while S3 arbitrates authority and STM32 retains
final motion authority. The current implementation is not yet L3-ready as a
shared transport/domain seam because the transport API is BLE-shaped and the
status/session model is not protocol-event driven. Before adding map,
mission, or autonomy UI, introduce capability negotiation, explicit authority
mode/session identity, typed autonomy state, and a test transport without
embedding ROS2 or navigation logic in the app.

## Required Follow-up Order

1. Fix the transport-neutral session/state interfaces and wire STATUS/ACK
   events, admission, freshness, and heartbeat handling.
2. Correct BLE peripheral retention and error/reconnect/notification lifecycle;
   freeze UUID, MTU, pairing, and authentication contracts separately.
3. Enforce packet header compatibility, field preservation, payload limits, and
   protocol-command values with parser/property tests.
4. Centralize release/cancel/background stop behavior and add simulated
   transport tests before any vehicle test.
5. Validate the actual iOS target in Xcode, then perform staged bench and
   controlled vehicle tests; do not promote package build evidence to runtime
   or hardware acceptance.
