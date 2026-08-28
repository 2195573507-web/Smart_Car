# Task Plan: App Speed Command Dispatch Repair

## Goal

Make the iOS and macOS speed-control paths deliver the current App-BLE wheel
and chassis frames reliably, including on the default BLE ATT MTU.

## Scope

- Split outbound App-BLE writes according to CoreBluetooth's negotiated write
  length while preserving complete-frame ordering.
- Keep in-flight frame fragments together when coalescing newer motion input.
- Remove the macOS-only legacy percentage speed slider that emits the retired
  `CONTROL/SPEED_CONTROL` path.
- Preserve `0x15`, `0x2A`, `0x2B`, `0x2D`, RR/RF/LR/LF ordering, and disconnect stop.

## Phases

- [x] Confirm source/protocol mismatch and receive-side fragmentation support.
- [x] Implement the shared BLE queue repair in both app targets.
- [x] Remove the macOS legacy speed control UI path.
- [x] Build both Swift packages and run static contract checks.
- [x] Record remaining BLE/hardware validation limits.
- [x] Repair continuous motion dispatch and stop priority across App, S3, and
      the existing STM command authority.
- [x] Re-run shared-link, Swift, ESP-IDF, CM7, and diff checks after the final
      cancellation API change.

## Verification

- `swift build --package-path IOS-APP`
- `swift build --package-path SmartCar_Control_MAC`
- Confirm 24-byte `0x15`/`0x2D` frames are queued as ordered chunks and parser
  input remains a byte stream.
- Confirm macOS UI no longer references `updateSpeed` or `sendSpeed`.
- `git diff --check`

## Status

Complete for source/build scope. The final cancellation API regression passed
on 2026-08-23. Live BLE MTU negotiation, S3 receive events, SCBP ACKs, UART
delivery, and physical motor response remain unverified.
