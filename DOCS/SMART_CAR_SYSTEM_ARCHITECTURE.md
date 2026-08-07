# Smart_Car System Architecture

## Scope and Status

This document plans the system boundary for the `IOS_APP` and `ESPS3`
subprojects. It does not create an iOS application, ESP-IDF project, protocol
implementation, motor-control implementation, or ROS2 package. The existing
`STM32H757` and `ROS2_WIN` sources and documents are protected and unchanged.

The planned platform is a layered mobile robot:

```text
iPhone App
  |  BLE and/or WiFi, according to the active control session
ESP32-S3 Gateway
  |  validated low-level transport (target: UART)
STM32H757
  |  real-time actuator and sensor interfaces
Motor drivers / encoders / IMU
```

At L3, the host autonomy layer is added without moving low-level motor
ownership out of STM32H757:

```text
iPhone App <---- control, status, mission UI ----> ROS2_WIN
     |                                                |
     +---- local commissioning / direct control ---- ESP32-S3
                                                      |
                                                   STM32H757
                                                      |
                                      motors, encoders, IMU, future lidar
```

The diagrams express intended ownership, not an implemented topology. The
precise packet contract belongs to `SMART_CAR_PROTOCOL.md`; the iOS and S3
internal designs belong to their respective subproject architecture documents.

## Component Ownership

| Component | Primary responsibility | Must not own |
| --- | --- | --- |
| `IOS_APP` | Operator experience, session selection, command intent, mode selection, and status presentation | Direct motor electrical control, real-time control loops, lidar parsing, SLAM implementation, or ROS2 hardware drivers |
| `ESPS3` | Phone communication endpoint, control gateway, STM32 bridge, safety-session enforcement, future radar ingress, and future ROS bridge | Motor PWM generation, encoder counting, estimator/control-loop timing, or autonomous path planning |
| `STM32H757` | Deterministic low-level actuator control, encoder/IMU acquisition, local interlocks, and final motion authority | Mobile UI, WiFi/BLE endpoint behavior, ROS2 graph management, or map/task UX |
| `ROS2_WIN` | Future host-side ROS2 integration, SLAM, localization, navigation, map management, and autonomy orchestration | Direct motor electrical interfaces or lidar hardware parsing already owned by S3 |

`ESPS3` is the explicit ownership boundary between user/network-facing control
and low-level real-time motion. It translates validated high-level intent into
the S3-to-STM32 transport contract and forwards STM32 state upward. It must not
silently infer motor-safe behavior that belongs in STM32H757.

## L1: Mobile Remote Control

L1 establishes a supervised remote-control path. The selected phone transport
may be BLE, WiFi, or a future negotiated combination; it must result in one
active control session, not parallel unarbitrated command sources.

### Command Flow

```text
Operator input in IOS_APP
  -> control intent (direction, speed request, mode, sequence/session identity)
  -> BLE/WiFi transport
  -> ESPS3 connection and command admission
  -> S3-to-STM32 control frame
  -> STM32H757 validation and real-time actuator action
  -> motor driver
```

The command path is one-way only in this representation. Its acknowledgement,
freshness, retry, ordering, and CRC rules must be specified by the unified
protocol document before functional implementation.

### Status Flow

```text
STM32H757 state and fault indications
  -> S3-to-STM32 status frame
  -> ESPS3 state aggregation and session-safe forwarding
  -> BLE/WiFi
  -> IOS_APP operator status
```

S3 is allowed to aggregate communications state, but STM32H757 remains the
source for low-level actuator and sensor state. The iOS display is a consumer
of reported state and must distinguish a stale/unavailable status from a
healthy vehicle.

### L1 Control Modes

- Direction-button mode expresses discrete operator direction intent.
- Virtual-joystick mode expresses bounded continuous direction and speed intent.
- Automatic mode is reserved as a separate command authority for a future
  phase; it is not an L1 behavior and cannot bypass the same safety gates.

## L3: ROS2, SLAM, and Autonomous Navigation

L3 introduces ROS2_WIN as the autonomy and map-management domain. S3 remains
the physical communication gateway and STM32H757 remains the final low-level
motion authority.

### Autonomy and Observation Flows

```text
Future lidar / IMU / encoder observations
  -> ESPS3 acquisition or STM32H757 low-level reporting
  -> WiFi gateway contract
  -> ROS2_WIN ROS2 interfaces
  -> SLAM, localization, navigation, mission logic
  -> bounded motion intent
  -> ESPS3 command admission
  -> STM32H757 final validation and actuation
```

```text
IOS_APP
  <-> ROS2_WIN: map, task, autonomy state, and operator supervision
  <-> ESPS3: local setup, diagnostics, and explicitly authorized fallback control
```

ROS2_WIN may request autonomous motion but cannot obtain a privileged path to
motor control. All L3 commands must carry a declared authority/mode, session
identity, freshness information, and a state report that permits the App to
show whether autonomous control is available, active, stopped, or faulted.

## Module Relationships

| Relationship | Direction | Boundary rule |
| --- | --- | --- |
| `IOS_APP` to `ESPS3` | Bidirectional | Mobile transport and application framing are separate; only authenticated and admitted control intent reaches the STM32 bridge. |
| `ESPS3` to `STM32H757` | Bidirectional | Target low-level transport is UART; command authority, framing, CRC, timeout, emergency stop, and status semantics must be uniform. |
| `ESPS3` to radar | Inbound to S3 | Reserve a dedicated radar adapter so lidar acquisition/parsing does not couple to phone-session or UART bridge code. |
| `ESPS3` to `ROS2_WIN` | Bidirectional | A ROS bridge adapter exposes gateway data and admitted autonomy intent without placing ROS2 logic in the real-time bridge. |
| `IOS_APP` to `ROS2_WIN` | Bidirectional, L3 | The App owns user interaction; ROS2_WIN owns autonomy/map computation and reports results through an explicit API. |

The planned S3 internal seams are: mobile transport adapters, connection/session
manager, command admission and safety gate, STM32 bridge, state publisher,
radar adapter, and ROS bridge. Their actual ESP-IDF task/core allocation is
deferred to `ESPS3/docs/S3_ARCHITECTURE.md` and the performance plan.

## Phase Plan

| Phase | Planned outcome | Required boundary before advancement |
| --- | --- | --- |
| Architecture planning (current) | Document system, App, S3, protocol, safety, performance, and tests | No feature code, packet implementation, hardware test, or claim of runtime behavior |
| L1 contract | Freeze App-to-S3 and S3-to-STM32 frame/version/error contracts | Resolve the STM32 communication hardware route and review the safety/protocol contract |
| L1 implementation and validation | Implement supervised phone remote control and state feedback | Static, build, transport, then vehicle safety evidence recorded separately |
| L2 sensing/gateway expansion | Add validated lidar/radar and richer telemetry boundaries | Define data ownership, rates, backpressure, time base, and failure behavior |
| L3 integration | Add ROS2_WIN bridge, SLAM, localization, navigation, and mission UX | Define authority arbitration, map/task APIs, odometry/time contracts, and autonomous-stop behavior |

The current STM32H757 baseline records a blocking communication-route issue:
the supplied `PD3` and `PD4` nets are not a UART TX/RX pair for
`STM32H757XIH6`. The S3-to-STM32 UART target therefore cannot be treated as an
implemented or hardware-valid interface until an authorized hardware-valid
route is selected. This architecture does not select or alter that route.

## Safety Ownership

Safety must be enforced in layers; no upper layer is a substitute for a lower
layer's stop path.

| Layer | Safety ownership |
| --- | --- |
| `IOS_APP` | Expose an immediate operator stop command, show connection/mode/status freshness, and avoid presenting stale feedback as live control. |
| `ESPS3` | Admit one authority at a time, reject malformed/stale/unauthorized frames, detect session/transport loss, forward stop requests with highest priority, and report gateway health. |
| `STM32H757` | Retain final motion authority; apply command timeout, local fault interlocks, safe output behavior, and deterministic stop independent of App, WiFi, BLE, ROS2, or S3 availability. |
| `ROS2_WIN` | Produce only bounded autonomy intent, yield immediately to emergency/manual-stop policy, and publish autonomy health instead of assuming command delivery. |

The exact stop behavior, timing thresholds, reset policy, sequence behavior,
and fault codes are intentionally not invented here. They must be jointly
defined by the protocol and safety-review documents, then validated in staged
hardware and vehicle tests.

## Evidence Limits and Open Decisions

This document is static architecture evidence only. It does not prove iPhone
connectivity, BLE latency, WiFi throughput, UART electrical compatibility,
UART traffic, motor response, encoder/IMU data, lidar compatibility, ROS2 DDS
connectivity, SLAM, navigation, emergency-stop timing, or end-to-end vehicle
safety.

Open decisions that require dedicated, authorized work are:

- Select a hardware-valid STM32H757-to-S3 physical transport route.
- Freeze protocol versions, packet limits, CRC, acknowledgement, heartbeats,
  command authority, and fault/status vocabulary.
- Define the L1 BLE/WiFi discovery, pairing, authentication, and transport
  selection policy.
- Define S3 resource/task budgets and backpressure for control, telemetry,
  radar, and future ROS traffic.
- Define L3 authority arbitration between operator control, autonomous
  navigation, and emergency stop.

Related baseline documents are [PROJECT_ARCHITECTURE.md](PROJECT_ARCHITECTURE.md),
[ESP32_ROLE.md](ESP32/ESP32_ROLE.md),
[IOS_ARCHITECTURE.md](IOS_APP/IOS_ARCHITECTURE.md),
[STM32_ARCHITECTURE.md](STM32H757/STM32_ARCHITECTURE.md), and
[ROS2_ARCHITECTURE.md](ROS2/ROS2_ARCHITECTURE.md).
