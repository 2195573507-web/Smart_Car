# App-BLE V2 Session and Shared Core Design

Status: approved for implementation on 2026-08-23.

## Objective

Improve `SmartCar_Control_MAC` and `IOS-APP` control responsiveness and
stability while retaining wire compatibility with deployed App-BLE V1 clients.
The design adds a V2 session envelope at the App-to-S3 boundary, leaves
SCBP-CAN and STM32 safety authority intact, and moves shared non-UI Swift
logic into one package.

## Scope

| Area | Change | Kept unchanged |
| --- | --- | --- |
| macOS/iOS | Shared parser, V2 session, bounded outbound scheduler, bounded history/log stores | Platform UI and CoreBluetooth ownership |
| App BLE | V2 HELLO, heartbeat, command wrapper, extended ACK | GATT UUIDs, V1 outer frame, V1 command payloads |
| ESP32-S3 | V2 admission, session expiry, duplicate suppression, SCBP completion mapping | Existing V1 bridge and telemetry/log relays |
| STM32 | No new App-BLE handling required | SCBP ACK, 1000 ms command watchdog, force-stop and actuator authority |

## V2 Contract

The outer frame is unchanged except for `VERSION=0x02`:

```text
AA | 02 | TYPE | LEN_LO | LEN_HI | PAYLOAD[LEN] | CRC_LO | CRC_HI | 55
```

`HELLO (0x70)` negotiates a server-assigned session. `HEARTBEAT (0x72)`
maintains it. `COMMAND (0x75)` carries `session_id`, `command_seq`,
`valid_for_ms`, the unchanged V1 command type, and its unchanged payload.
The S3 replies with `COMMAND_ACK (0x74)` after S3 admission, STM acceptance,
or stop enqueueing. Telemetry may remain V1-framed because its payload contract
is already stable and is independent from command-session negotiation.

## App Scheduling and Runtime Model

The shared package exposes:

- a bounded byte-ring parser with fragmentation/noise recovery;
- a bounded two-lane outbound scheduler;
- a V2 session state machine with deterministic timers and V1 fallback;
- protocol constants and payload builders for both apps.

Reliable control/configuration frames use a fixed FIFO cap. Motion is a
single latest-state slot, so a slow BLE write cannot make a stale target wait
behind newer UI movement. Zero or emergency motion clears all unsent nonzero
motion and is dispatched ahead of ordinary traffic. BLE parsing stays off the
main actor; UI-visible state is batched except for state transitions, ACKs,
and faults.

## S3 State and Safety

S3 accepts V2 commands only for its active session, within the declared
receiver-relative validity duration, and with a new command sequence. Duplicate
commands are acknowledged without retransmitting their SCBP effect. Session
expiry or BLE disconnect serializes an explicit zero-wheel SCBP stream frame.
This does not replace STM32's existing command watchdog, BUS_OFF handling, or
local interlocks.

## Verification

1. Unit-test frame fragmentation, malformed recovery, V2 codec, session
   negotiation/fallback/expiry, and scheduler stop priority.
2. Build both Swift applications against the same shared package.
3. Build the S3 target and run targeted parser/bridge checks.
4. On hardware, record UI-to-ACK latency, BLE reconnect, session expiry,
   V1 fallback, UART propagation, and vehicle stop behavior separately.
