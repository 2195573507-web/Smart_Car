# Findings

- `radar_command_task()` currently exits when `fgets()` returns `NULL`.
- ESP-IDF task wrapper aborts if a task function returns; panic is configured
  for immediate reboot.
- USB Serial/JTAG stdin can be temporarily unavailable when disconnected.
- No runtime evidence currently supports changing WDT, brownout, or stack size.
