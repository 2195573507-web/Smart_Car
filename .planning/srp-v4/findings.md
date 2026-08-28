# SRP v4 Findings

- The worktree contains unrelated user changes; preserve them.
- `Common/SRP` is the active shared protocol implementation.
- CM7 Debug build already passes after the UART link rewrite.
- The last ESP-IDF build stopped at `log_bridge.c` because `SRP_MSG_ID_LOG`
  was missing from its include graph; `log_bridge.h` now includes
  `srp_registry.h`.
- Runtime baud-rate TLV handling is wired through both STM32 and ESP32 service
  layers.
- Runtime `SYS_CONFIG` TLV handling is now wired on both endpoints. Only
  `921600` and `115200` are accepted; ACK is sent before the guarded switch.
- Active source and protocol index references now use SRP directly. Deprecated
  SCBP documents remain as historical records.
- CM7 Debug, CM4 Debug, ESP-IDF and host codec tests pass after the final
  changes. The root `STM32H757/build/Debug` directory has no Ninja build file
  and was not treated as a build target.
- STM32 TX now uses four statically allocated FreeRTOS queues; static RTOS
  allocation was enabled with an explicit idle-task memory hook. D2 SRAM use
  remains 17,536 B / 288 KiB in the CM7 image.
