# Task Plan: STM32H757 to ESP32-S3 SCBP Protocol Specification

## Goal

Produce a source-grounded Markdown specification of the STM32H757 <-> ESP32-S3
serial protocol, including physical UART settings, active SCBP-V3 framing,
message catalog, payload layouts, parser behavior, and ACK/retry evidence.

## Evidence boundary

Read-only source and documentation audit. Do not edit firmware, build files, or
existing user-owned planning documents. Source/build facts are not hardware or
runtime acceptance.

## Phases

- [x] Read workspace rules, indexes, and protocol baseline documents
- [x] Inventory STM32 and S3 protocol/transport source files
- [x] Extract frame constants, CRC, IDs, payload layouts, and state machines
- [x] Reconcile active SCBP-V3 with legacy adapters and document conflicts
- [x] Write and self-review the protocol specification
- [x] Run static consistency checks and report evidence limits
