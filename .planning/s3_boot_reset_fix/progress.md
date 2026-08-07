# Progress

## 2026-08-01

- Read the reset analysis, current firmware source, configuration, and existing
  workspace planning files.
- Selected bounded stdin recovery plus reset/stack diagnostics.
- No source or configuration changes had been made before user approval.
- Updated `radar_test.c`: stdin failures call `clearerr(stdin)`, emit a
  rate-limited warning with command-task stack high-water mark, delay 250 ms,
  and retry forever. Startup now logs `esp_reset_reason()`.
- Updated `CMakeLists.txt` with the explicit `esp_system` dependency.
- Updated `S3_BOOT_RESET_ANALYSIS.md` to distinguish the repaired path from
  residual WDT, brownout, stack, and hardware evidence.
- Isolated ESP-IDF 5.5.4 build passed; no flash, monitor, or hardware test was
  run.
