# Progress: S3 Console and SRP UART2 Handshake

2026-08-24: Read repository boot/rules/index/status and affected S3/STM32
source. Worktree is intentionally dirty; unrelated changes are preserved.

2026-08-24: Updated the active S3 console configuration and defaults to USB
Serial/JTAG, added S3 UART2 task/queue counters, and added STM32 USART2 DMA
RX/TX/SRP boundary diagnostics. STM32 CM7 and S3 isolated builds completed
successfully; generated S3 configuration confirms UART Console is disabled.

Remaining acceptance: flash matching images and capture the diagnostic lines on
the live board. No physical GPIO44 release or UART handshake is claimed from
the source/build evidence alone.

2026-08-24 follow-up: the flashed gateway image reaches the ROM bootloader
entry (`0x403c8990`) but the USB device node disappears before an IDF `boot:`
banner. The current and prior successful images differ in console selection
(USB Serial/JTAG primary versus UART0 primary + USB secondary). Hardware state
is currently unavailable because the old monitor is still attached to
`/dev/ttys001` and no `/dev/cu.usbmodem*` node is present. Pending manual
BOOT/reset and read-only `chip_id`/`flash_id` evidence; no source patch or
flash operation is authorized from this observation alone.

2026-08-24 device re-enumeration: the target
`/dev/cu.usbmodem5B901601171` pair is present and not directly owned by a
monitor process. A second `usbmodem143101` pair is present; capture and flash
commands remain pinned to the known target path.

2026-08-24 native USB capture: PySerial identified `5B901601171` as an
external VID `0x1A86` USB-UART and `143101` as the ESP32-S3 VID/PID
`0x303A/0x1001` native USB JTAG/serial device. Monitoring `143101` produced
the full bootloader and app log, including `Calling app_main()`, UART2
GPIO17/18 at 921600 baud, `UART2 ready`, BLE, Radar, and SRP diagnostics. The
missing logs on `5B...` were caused by selecting the external UART port.

2026-08-24 final hardware pass: rebuilt `smartcar_s3_gateway.bin` (`0xb1480`),
flashed bootloader/app/partition/OTA images through
`/dev/cu.usbmodem5B901601171`, and verified all writes by SHA. A subsequent
monitor on `/dev/cu.usbmodem143101` captured bootloader logs, `main_task:
Calling app_main()`, `UART2 init TX=GPIO17 RX=GPIO18 baud=921600`, `UART2
ready`, BLE/Radar startup, and repeated SRP diagnostics.

2026-08-24 CM7 RX/handshake follow-up: changed
`STM32H757/Middleware/Communication/UART_Link/uart_link.c` so circular DMA RX
is marked active before HAL enables IDLE/DMA interrupts, preventing a pending
first event from being discarded; UART error callbacks now wake the recovery
worker immediately. Extended `SRP_S3` in
`STM32H757/Middleware/Communication/Services/s3_service.c` with combined UART
RX/event/rearm, TX DMA, error, and ring-buffer counters. CM7 Debug build,
host SRP codec test, and `git diff --check` passed. Event-byte accounting now
uses the actual bytes committed to the ring across circular wraparound.
Hardware remains pending:
flash the matching CM7 image and classify `uart_rx`, `ev`, `sy`, and `bi`.

S3 rebuild retry was blocked before compilation because the configured
ESP-IDF Python environment
`/Users/zhiqin/.espressif/tools/python_env/idf5.5_py3.9_env/bin/python` is
missing; `idf.py` was not available after exporting ESP-IDF. This is an
environment prerequisite failure, not a source diagnostic.

2026-08-25 continuation: clean-built `STM32H757/CM7/build/Debug` target
`Smart_Car_H757_CM7` (73/73, no compiler warnings/errors). `git diff --check`
and the Common/SRP host codec test passed. ELF inspection confirms
`DMA1_Stream0_IRQHandler`, `DMA1_Stream1_IRQHandler`, `USART2_IRQHandler`,
`HAL_UARTEx_RxEventCallback`, and `HAL_UART_ErrorCallback` are linked; the RX
buffer starts at `0x30000000` in `.dma_buffer` and all DMA buffers end at
`0x30004480`. The existing ESP32-S3 Ninja build completed its generated-image
size checks with the available py3.14 environment; a fresh `idf.py` configure
was not attempted because the configured py3.9 environment is absent.

Static verification is complete. Flashing the matching CM7 image and live
UART2 capture remain required to classify `SRP_UART2_DIAG` (`rx/events/rearm`)
and `SRP_S3` (`sy/bi`) counters; no physical handshake is claimed here.

2026-08-25 targeted RX-blocking follow-up: static inspection found no D2
non-cacheable MPU overlay and no compact runtime evidence for IRQ entry,
DMAMUX request IDs, or NDTR. Phase 8 patch is limited to that MPU protection
and scalar diagnostics; PA2/PA3 and manual DMA ownership remain unchanged.

2026-08-25 Phase 8 implementation: added the 32 KB D2 MPU overlay, startup
DMAMUX request validation, IRQ/callback/NDTR/error snapshots, and a packed PA3
GPIO state. Incremental CM7 build and `git diff --check` pass; clean build and
host SRP test evidence remain recorded above, with one final clean rebuild
after the last diagnostic field addition.

2026-08-25 final verification: clean CM7 Debug build completed all 73 steps
with no compiler warnings/errors; FLASH usage is 199108 B and `.dma_buffer`
remains at `0x30000000` with size `0x4480`. Common/SRP codec test passes with
`-std=c11 -Wall -Wextra -Werror`, `git diff --check` passes, and ELF symbols
for MPU setup, both DMA IRQs, USART2 IRQ, Rx event, and UART error callback are
present. No matching CM7 image was flashed in this turn, so live RX counters
and electrical PA3 continuity remain pending hardware acceptance.
