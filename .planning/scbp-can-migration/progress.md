# Progress

> Historical/deprecated record from before the SRPv4 full switch. Current
> implementation and verification must use `Common/SRP` and the active CMake
> source lists.

- Phase 1-5 completed: shared SCBP-CAN implementation, STM32H757 migration,
  and ESP32-S3 migration are in the worktree.
- ESP32-S3 build passed with ESP-IDF 5.5.4 using the active `ESPS3/build`
  directory. No flash or monitor command was run.
- Phase 6 completed: active and historical protocol documentation are
  separated, module indexes use the shared SCBP-CAN names, and final static
  scans/build/test checks passed.
