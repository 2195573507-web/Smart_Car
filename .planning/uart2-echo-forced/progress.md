# Progress

## 2026-08-25

- Confirmed the user's forced-test request supersedes the previous optional,
  default-off Echo configuration.
- Inspected active source, CMake targets, UART MSP configuration, and prior
  test documentation.
- Applied forced S3 and CM7 test source graphs. The CM7 test uses a direct
  blocking loop with no RTOS, DMA, SRP, service, or motor transport source.
- CM7 configure attempt 1 reached the forced target but failed in the
  post-build binary generator expression. Corrected it before retrying.
- CM7 build passed: 21,284 B flash, 1,880 B RAM, and no RAM_D2 allocation.
- S3 build passed, but the first defaults file carried four unknown BT symbols
  after service components were excluded. Removed those no-op assignments and
  forced the IDF target before the clean rebuild.
- Final symbol audit found no SRP, `uart_link`, `s3_service`, `stm_uart`, BLE,
  radar, or project-protocol symbols in either ELF. Corrected one S3 source
  literal that would have sent the two-character text `\\n` instead of LF.
- Final Clean Build passed for both targets. CM7 produced a 21,284 B binary;
  S3 produced a 196,976 B app binary and a 21,312 B bootloader binary.
- No device was flashed or monitored; physical TX/RX acceptance remains
  pending.
- Added S3 TX hex logging so both directions are emitted as text and raw hex.
- The final audit command initially used an invalid Xtensa `nm` path and was
  not accepted as evidence; it will be rerun with the toolchain's nested
  `xtensa-esp-elf/bin` path.
- Rebuilt both forced targets from clean outputs. CM7 is 21,284 B and S3 is
  196,992 B. The corrected Xtensa symbol audit passed: no SRP, legacy UART
  service, UART-DMA, or `gdma_*` runtime symbols are linked. The ELF exports
  one absolute `GDMA` peripheral-address linker symbol only; it is not DMA
  code or a DMA runtime service.
- The forced Echo target was superseded by the one-way PA2-to-GPIO18 level
  test. CM7 now drives PA2 low/high at 1 Hz with GPIO only, while S3 samples
  GPIO18 without pulls or interrupts every 100 ms. Fresh CM7 and S3 builds
  passed; the CM7 ELF has no UART/DMA/SRP symbols and the S3 map has no UART
  driver, SRP, service, BLE, or radar component. See `DOCS/GPIO18_LEVEL_TEST.md`.
