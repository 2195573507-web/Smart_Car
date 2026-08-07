# Multi-Agent Rules

## Role Separation

| Role | Authorized work | Prohibited work | Required output |
| --- | --- | --- | --- |
| Implementation Agent | Create approved directories, Markdown scaffolding, STM32H757 base-project artifacts, IOC, GPIO and basic SPI/I2C/UART/timer configuration | Business code, algorithms, third-party libraries, ESP32 feature changes | Changed-file list and actual static/CubeMX/build evidence |
| Review / Audit Agent | Independently inspect directory structure, documentation coverage, hardware-net records, IOC conflicts, and peripheral multiplexing | Any modification to implementation artifacts | Read-only audit report with findings and evidence limits |
| Documentation Agent | Maintain the required `DOCS/` hierarchy, architecture, workflow, pin-map, build, and subsystem boundary documents | IOC edits, source/configuration changes, business implementation | Document index and static documentation check |

## Mutual Exclusion

- The reviewer must inspect the implementation as found and must not edit it before reporting.
- The documentation role must not alter the IOC or compensate for an implementation defect by changing technical facts.
- The implementation role must not present its own self-check as the independent audit.
- All roles may report ambiguities; only the task owner may authorize a scope expansion.

## Handoff Contract

Each handoff includes the task scope, files inspected or changed, command results, unresolved questions, and the applicable validation level. A role must preserve an unresolved fact as pending rather than turn it into a conclusion.

## Initialization Constraints

This task is limited to engineering structure and base configuration. The following are out of scope: motor algorithms, encoder processing algorithms, SLAM, navigation, iOS features, lidar frame parsing, ESP32 functional edits, flashing, and runtime hardware testing.

## Audit Criteria

The independent audit checks that required documents exist, document links resolve, required pins appear once in the pin map, IOC selections have no pin conflict, and the claimed external peripheral functions match the configured alternate functions. Any unresolved hardware-net/alternate-function discrepancy is a finding, not a passing result.
