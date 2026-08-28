# Task Plan: SRP CRC Diagnostic Repair

## Goal

Make an SRP CRC mismatch diagnosable on ESP32-S3 and protect the 16-byte
handshake contract with host tests, while preserving the shared codec and wire
bytes.

## Phases

- [x] Audit current SRP source, protocol documents, and worktree boundaries
- [x] Obtain design approval
- [x] Record approved design and implementation notes
- [x] Add S3 CRC/raw-frame diagnostics
- [x] Add layout assertions and handshake golden tests
- [x] Run host, ESP-IDF, CM7, and static verification
- [x] Report source/build versus hardware evidence

## Protected Boundaries

- `Common/SRP` remains the only STM32-S3 codec implementation.
- Preserve `AA 55`, CCITT-FALSE CRC, little-endian fields, UART2 GPIO17/18,
  USART2 PA2/PA3, and existing sync IDs/state behavior.
- Do not accept alternate CRCs or claim hardware recovery from a build.
