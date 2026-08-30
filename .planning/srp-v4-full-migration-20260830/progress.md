# Progress

## 2026-08-30

- Resumed task from conversation `01a04d49-0a18-7f22-9923-42b3f072332a`.
- Confirmed branch `4`, dirty worktree, and prior SRPv4 source migration.
- Created this dedicated plan so the prior read-only audit plan remains intact.
- Next: build and inspect failures, then complete active documentation cleanup
  and residual scans.
- ESP-IDF 5.5.4 build passed in `ESPS3/build`; SRP codec/link and S3 gateway
  linked successfully.
- CM7 `cmake --preset Debug` failed before generation because the migrated
  CMake references `Application/Chassis/chassis_*.c`, which are present on
  `origin/3` but absent from branch 4. This is now the first repair item.
- Compared `Common/SRP` against `origin/3`; all 11 files match the branch-3
  SRPv4 baseline byte-for-byte.
- Active source and canonical build graphs contain no `SCBP`, `scbp_*`,
  `sc_frame`, `SmartCar_Frame`, or `Common/SCBP_CAN` references. Updated the
  remaining active index/status text and ROS2 decoder error labels to SRPv4
  names. Empty legacy source directories were removed; ignored historical
  build directories were intentionally preserved.
- Final evidence: `git diff --check` passed; active-source scan is clean;
  canonical `compile_commands.json` files are clean; `Common/SRP` matches
  `origin/3` in all 11 files; SRP codec, radar host tests, ESP-IDF build,
  CM7 Debug build, and ROS2 telemetry decoder syntax check passed.
- Residual old-protocol text is limited to explicitly historical audit/report
  files, deprecation notes, and the independent App BLE `SmartCarProtocol`
  model. Ignored historical build directories remain outside source cleanup.
- Device log review found an independent S3 startup stack overflow after the
  experimental radar uplink started. `radar_uplink_task` now has a 6144-byte
  stack, reports its high-water mark, and only reapplies Wi-Fi credentials
  after the disconnect/rotation event; the overflow hook preserves the raw
  task name/handle and reports that task's watermark. ESP-IDF rebuild passed.
- The stack change is source/build verified only. The pasted device image was
  built at `Aug 30 2026 01:35:35` with ELF SHA prefix `68a97e731`; it must be
  reflashed before interpreting the new diagnostics or claiming runtime
  recovery.
- Reconfigured and rebuilt canonical `STM32H757/CM7/build/Debug` from a clean
  target; 73 compile/link steps passed and the ELF now uses only `Common/SRP`.
- Ran `idf.py fullclean` followed by a complete ESP-IDF 5.5.4 build in
  `ESPS3/build`; 1119 application/bootloader steps passed and the generated
  image uses only SRPv4 sources.
- Removed the exact stale CM7 generated `Common/SCBP_CAN` object subtree by
  moving it to `/tmp/smartcar-stale-scbp.H0lQgS`; no stale SCBP objects remain
  in either canonical build directory.
- Updated active indexes and BMI323 diagnostic reports so current routes name
  SRPv4, while old SCBP/V3 paths are explicitly historical/superseded. The
  protected `.codex/MEMORY.md` and historical planning/archive records were
  intentionally not edited.
- Fresh host checks passed: SRPv4 codec test, radar parser/FIFO/uplink/queue
  tests, and `git diff --check`.
