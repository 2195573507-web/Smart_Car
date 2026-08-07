# Smart_Car Codex Boot

## Project

Smart_Car is a staged STM32H757 + ESP32-S3 + macOS control system. STM32 owns
deterministic sensing, calibration, attitude, actuation, local safety, and final
motion authority. ESP32-S3 owns the gateway, BLE GATT transport, STM UART
transport, and current radar UART/PWM integration. ROS2_WIN is a future host
domain for SLAM, navigation, and autonomy.

## Current Stage

The current workspace contains source-level integration work but no single
released end-to-end acceptance. Read [PROJECT_STATUS.md](../PROJECT_STATUS.md)
for dated status, risks, and open confirmations.

## Git Structure

Smart_Car uses one repository rooted at `Smart_Car/.git`. All project modules
are managed through that root. Do not create nested Git repositories or Git
submodules inside STM32H757, ESPS3, IOS_APP, ROS2_WIN, Hardware, Tools, or
documentation directories. Read [the Git workflow](../docs/development/git_workflow.md)
before a repository-wide staging or history operation.

## Mandatory Reading

Read these files before any code change:

- [MEMORY.md](MEMORY.md): stable facts and evidence vocabulary.
- [RULES.md](RULES.md): scope, ownership, and verification rules.
- [INDEX.md](INDEX.md): module-to-document-to-source navigation.
- [PROJECT_STATUS.md](../PROJECT_STATUS.md): current state and blockers.

Then read the affected module page and its source map entry.

## Working Principle

Understand first, analyze second, modify third, verify last. Keep implemented
behavior, planned design, and historical evidence visibly separate. Never infer
hardware or runtime acceptance from documentation or a build.

## Hard Stops

- Do not modify hardware definitions, GPIO, IOC/CubeMX, or protocol bytes
  without explicit authorization.
- Do not merge the App BLE frame contract with the STM32-S3 transport frame.
- Do not claim a complete BLE/UART/vehicle chain when a callback, parser, or
  physical capture is missing.
- Do not widen a narrow task into an architecture rewrite.
