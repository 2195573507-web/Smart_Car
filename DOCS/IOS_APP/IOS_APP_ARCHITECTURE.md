# Smart Car iOS App Architecture Plan

## Scope and Evidence

This document defines the planned architecture of the future `IOS_APP` mobile
application for Smart Car L1 remote control and its L3 expansion path. It is a
static planning artifact only. It creates no Xcode project, Swift code, user
interface, protocol implementation, mobile connection, or vehicle behavior.

At review time, this repository has no populated `IOS_APP/` application
directory. [IOS_ARCHITECTURE.md](IOS_ARCHITECTURE.md) remains the existing
mobile responsibility boundary. This plan refines that boundary without
changing it. The exact packet fields and command identifiers must be supplied
by [SMART_CAR_PROTOCOL.md](../SMART_CAR_PROTOCOL.md) before implementation.

The app is an operator-facing control and status client. It does not own motor
PWM, low-level control loops, encoder/IMU interpretation, emergency braking
enforcement, lidar parsing, ROS2 drivers, SLAM, navigation, or autonomous path
execution. ESP32-S3 is the gateway and command-admission boundary; STM32H757
retains real-time vehicle control responsibility.

## Product Stages

| Stage | iOS responsibility | Included design scope | Deferred |
| --- | --- | --- | --- |
| L1 | One-vehicle operator console | Discovery, session, manual control, stop, and status | Map, SLAM, missions, autonomy |
| L2 | Connected robot operations console | Richer health, telemetry, diagnostics, and configuration after contracts exist | Navigation authority and map editing |
| L3 | Mission supervision client | Map, localization, route/task, autonomy status and intervention surfaces | Running SLAM, planning, navigation, or safety enforcement on the phone |

L1 is planned as iPhone-to-ESP32-S3 BLE and local Wi-Fi connectivity. L3 may
add a ROS2-facing gateway/service API. The iOS app must never communicate
directly with ROS2 topics, STM32 UART, or motor interfaces.

## Logical Architecture

```text
+------------------------ iPhone / IOS_APP -----------------------+
| SwiftUI -> Control domain -> Session / transport facade          |
|                         |                                        |
|                  BLE adapter | Wi-Fi adapter                     |
+-------------------------|--------------|------------------------+
                          |              |
                  BLE GATT / local IP application protocol
                          |              |
+----------------------- ESP32-S3 -------------------------------+
| command admission, validation, gateway safety, UART bridge,     |
|                     status aggregation                           |
+-------------------------|---------------------------------------+
                          | UART
                    STM32H757 -> motor / encoder / IMU
```

SwiftUI expresses operator intent. The domain validates the local session and
maps input to protocol-neutral commands. A selected transport serializes those
commands under the shared protocol. ESP32-S3 must make the final admission and
safety decisions when an iOS view is stale, backgrounded, or disconnected.

## Planned Project Structure

The future app should use a SwiftUI target with dependency direction toward a
platform-independent domain. These are planned names, not created source files.

```text
IOS_APP/
  SmartCarApp/        lifecycle, dependency composition, scenes
  Features/
    Connection/       discovery, pairing and transport selection UI
    Drive/            direction-pad and virtual-joystick UI
    Status/           health and telemetry presentation
    Safety/           stop and safety-state presentation
    Diagnostics/      development/service diagnostics surface
    Autonomy/         L3 placeholder only; no navigation logic
  Domain/
    VehicleSession/   state machine and transport selection
    Control/          control intent, modes and app-side arbitration
    Telemetry/        normalized status and freshness policy
    Protocol/         packet-independent command/event models
  Data/
    Transport/        VehicleTransport facade and contracts
    BLE/              CoreBluetooth GATT adapter
    WiFi/             Network-framework transport adapter
    Persistence/      non-secret preferences; future approved secret policy
  SharedUI/           reusable, accessible controls
  Tests/              unit, UI, contract tests and transport doubles
```

`Features` depend on `Domain` interfaces and observable view models. `Data`
implements the interfaces. `Domain` must not import SwiftUI, CoreBluetooth, or
socket APIs. The `SmartCarApp` composition root selects live or test adapters;
views must not contain packet or motor-control policy.

## Screens and Operator Flow

| Surface | L1 responsibility | Required state behavior |
| --- | --- | --- |
| Connection | Scan/select BLE device or configured Wi-Fi gateway; start/stop session | It is not vehicle-ready until the gateway admits the session and status is fresh. |
| Drive | Select manual mode; issue direction-pad or joystick control | Always show active transport, mode, readiness, freshness, stop/fault state, and emergency stop. |
| Status | Show gateway-provided power, drive enable, fault, version and telemetry | Unknown, stale, or conflicting data is unavailable, never healthy. |
| Safety detail | Present stop, fault, timeout, and rejection reason | It cannot clear a hardware or gateway fault locally. |
| Diagnostics | Later support/development surface | Must not expose secrets or become a control authority. |
| Autonomy | L3 map/localization/mission entry point | Disabled in L1 until the gateway and ROS2 safety contract permits it. |

Normal L1 flow: choose a vehicle and transport -> wait for an admitted, fresh
`ready` session -> select a manual control mode -> hold/send input while the
session remains valid -> release/center to stop -> disconnect only after an
acknowledged stop or a visibly uncertain result.

## Control Modes

### Direction-pad

The direction pad emits bounded forward, reverse, left, right, or stop intent.
Press is active only while held. Releasing or cancelling a touch, losing focus,
backgrounding, or changing mode must send a normal stop over the active
transport. Control updates and heartbeat cadence must follow the final protocol
so an old motion command is not assumed valid indefinitely.

### Virtual joystick

The virtual joystick maps a centered two-axis gesture to normalized linear and
turn intent. The domain applies documented dead-zone and negotiated L1 limits.
Cancellation, interruption, mode switch, and a centered/released gesture emit
zero linear and turn intent. Raw pixels or iPhone orientation are never vehicle
commands.

### Automatic mode reservation

`Automatic` is a future, distinct mode, not a variant of manual control. It is
disabled in L1. In L3 the app can request a gateway/ROS2-managed autonomy
session and supervise map, localization, route and task state. It cannot create
or execute a path. Manual takeover and emergency stop remain reachable subject
to the final safety contract.

### Command arbitration

The app has one selected control mode and one active command-bearing transport
per vehicle session. BLE and Wi-Fi may coexist only for discovery/status if the
protocol permits it. The client must not duplicate motion commands over both
links or silently fail over during a command. A transport change requires stop
intent -> confirmed or visibly uncertain result -> new session admission ->
fresh status -> operator-visible ready state. ESP32-S3 remains the final
arbiter among the phone, ROS2, and any future command sources.

## Session and Transport Design

### Session states

```text
idle -> discovering -> connecting -> negotiating -> synchronizing -> ready
                 |          |             |                 |
                 +----------+-------------+-----------------+-> faulted
ready -> stopping -> disconnected
ready -> stale_status -> stopping -> disconnected
```

`ready` requires identified gateway, protocol/version compatibility, session
admission, selected active transport, and fresh status. `faulted`,
`stale_status`, and `disconnected` suppress motion input. Inactive/background
transitions request stop, cancel input, and invalidate local readiness; gateway
timeouts must independently stop the vehicle if delivery fails.

### Transport facade

The domain layer should depend on a single asynchronous facade:

```text
VehicleTransport
  discover() / connect(identity) / disconnect()
  negotiate(protocolVersion, clientCapabilities)
  send(command) / requestStop() / requestEmergencyStop()
  statusEvents / connectionEvents / protocolEvents
```

The facade returns typed acknowledgement, rejection, timeout, and connection
results. It carries protocol session identity, command sequence, source, and
freshness metadata without exposing GATT characteristics or sockets to SwiftUI.

| Adapter | Intended use | Mandatory design constraints |
| --- | --- | --- |
| BLE / CoreBluetooth | Local L1 discovery and low-latency control | Finalize service/characteristic UUIDs, MTU/fragmentation, notifications, reconnect, and pairing/authentication in protocol/security design. |
| Wi-Fi / Network framework | Local higher-throughput telemetry and future gateway access | Finalize provisioning, identity, encryption/authentication, reconnect, endpoint discovery, and stream/datagram/HTTPS/WebSocket choice before code. |
| Test transport | Deterministic unit, UI, and protocol tests | Simulate ack loss, stale status, rejection, disconnect, conflict, and timeout without a vehicle. |

Adapters do not select motor limits, implicitly retry non-idempotent controls,
or translate a gateway fault into local success. Those decisions belong to the
approved protocol and gateway safety design.

## Communication and Status Boundary

Before iOS implementation, the protocol must define exact packet format, CMD
IDs, serialization/endian rules, CRC coverage, frame limits, version and
capability negotiation, acknowledgements, heartbeat, errors, and the matching
S3-to-STM32 rules. The app needs these normalized categories:

| Category | iOS use | Source of truth |
| --- | --- | --- |
| Session admission / compatibility | Enables or blocks controls | ESP32-S3 acknowledgement |
| Command acknowledgement / rejection | Displays accepted, rejected, timed-out, or uncertain outcome | Gateway response correlated to session and command |
| Safety and drive-enable state | Gates control affordances | Gateway-aggregated STM32/S3 status |
| Timestamped telemetry | Displays only fresh data | Gateway status stream |
| Stop and fault reason | Gives actionable operator state | Gateway / STM32 fault contract |
| ROS2 / autonomy state (L3) | Supervises map, localization and task | Versioned ROS2-facing gateway API |

An iPhone acknowledgement proves only the application-layer result specified by
the gateway. It does not prove that STM32 applied motor output or that the car
moved. A client freshness timer is only a UI guard; firmware-side watchdogs and
link-loss behavior remain mandatory.

## Safety UX Requirements

- The Drive surface retains a persistent emergency-stop action. Its exact
  wording and acknowledgement follow the final protocol; it cannot claim
  physical braking without hardware/integration evidence.
- Motion input is disabled until `ready` and on backgrounding, stale status,
  mismatch, fault, or transport replacement.
- Normal stop and emergency stop are separate command intents. Normal stop is
  used on release/disconnect; emergency stop has gateway/STM32-defined action.
- The UI distinguishes rejected, unacknowledged, link-lost, and vehicle-stopped
  outcomes rather than treating every stop request as success.
- Automatic reconnect never hides a fault. New admission and fresh status are
  required before control returns.
- Accessibility needs large targets, semantic labels, and color-independent
  safety indication; no color alone conveys a safety state.

## L3 Expansion Boundary

L3 adds `Autonomy`, `Map`, `Mission`, and possibly `RobotFleet` features over
the same session facade. They consume a versioned gateway/ROS2 service exposing
renderable map data, localization confidence, task lifecycle, route progress,
and restricted autonomy requests. They do not embed ROS2 middleware, SLAM,
navigation planning, lidar parsing, or real-time control in the phone.

The shared protocol must reserve version/capability extension so L3 telemetry
and autonomy state cannot reinterpret L1 motion frames. Future multi-vehicle
operation requires explicit vehicle and session identity; L1 supports one
actively controlled vehicle at a time.

## Entry Criteria and Validation Plan

Functional iOS work must wait for: (1) approved APP-to-S3 and S3-to-STM32
protocols including ownership, stop, heartbeat and identity; (2) gateway safety
review of timeout, emergency stop, status authority, reconnect and phone/ROS2
conflict; (3) minimum iOS version, signing, test, accessibility, BLE/local
network privacy, and credential policy; and (4) ESP32-S3 BLE/Wi-Fi capability
review against the contract.

Planned evidence layers are static architecture/protocol review, unit tests for
mode/session/packet conversion, UI tests for non-ready/fault states, simulated
BLE/Wi-Fi contract tests, and then controlled hardware plus end-to-end vehicle
tests. No later evidence has been obtained by this document.

## Static Planning Result

This is the sole artifact produced by the IOS Development Agent. It records a
future SwiftUI and transport design, not an implemented mobile client. BLE,
Wi-Fi, UART forwarding, command delivery, emergency stopping, telemetry,
motor control, ROS2 integration, SLAM, navigation, and vehicle behavior remain
unverified.
