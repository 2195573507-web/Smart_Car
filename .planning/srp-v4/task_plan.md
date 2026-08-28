# SRP v4 Full Switch - Execution Plan

## Goal

Finish the already-approved SRP v4 full protocol switch while preserving GPIO
assignments and upper-layer business behavior.

## Phases

- [x] Shared SRP codec/link library and host tests
- [x] STM32 UART2 DMA/IDLE link rewrite and CM7 build
- [x] ESP32 UART bridge rewrite and queue isolation
- [x] Repair ESP-IDF build and complete SYS_CONFIG baud-rate handling
- [x] Remove remaining active SCBP compatibility/source references
- [x] Add/update SRP v4 documentation and protocol indexes
- [x] Run host, CM7, CM4, ESP-IDF builds and static checks

## Acceptance Boundary

Builds and host tests provide source-level evidence only. UART electrical,
DMA runtime, EMG latency, dynamic baud synchronization, and 24-hour CRC
pressure criteria require hardware captures and remain explicitly unverified
unless such evidence is produced.
