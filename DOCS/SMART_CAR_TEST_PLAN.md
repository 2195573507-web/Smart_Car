# Smart_Car Test Plan

## Evidence Levels

1. Static: Markdown links, packet tables, state-machine review, and scope
   checks. This phase is the only evidence obtained in the current task.
2. Host simulation: protocol parser/property tests, transport doubles, and
   deterministic session/timeout tests.
3. Bench: real BLE/Wi-Fi/S3/UART loopback with current probes and fault
   injection, without wheels contacting an unsafe surface.
4. Controlled vehicle: guarded motion, emergency stop, reconnect, and long-run
   tests with an independent observer and logged firmware versions.

## Test Matrix

| Area | Required cases | Evidence |
| --- | --- | --- |
| BLE | discovery, pairing, MTU/fragmentation, loss, reconnect, stale status | packet trace and App logs |
| Wi-Fi | provisioning, endpoint/auth, loss, roaming, throughput/backpressure | network trace and gateway metrics |
| UART | framing, CRC, sequence, burst load, invalid lengths, route loopback | logic-analyzer trace and CRC counters |
| App | pad/joystick bounds, release-to-stop, background, mode switch, fault UI, accessibility | unit/UI test report |
| S3 stress | concurrent sessions rejected, telemetry/radar load, heap pressure, watchdog, long duration | task/heap/queue telemetry |
| Safety | timeout, E-stop priority/latch, source conflict, power/link restart | timestamped stop evidence |
| L3 preparation | capability negotiation, autonomy intent rejection in L1, map/task state schema | contract tests only |

## Exit Criteria

No L1 feature implementation begins until the protocol, UART route, safety
review, and performance budgets are approved. No vehicle acceptance is claimed
from host tests or a successful build. Every failure records firmware/app
version, transport, command/session sequence, timestamps, and whether the
vehicle was physically moving.
