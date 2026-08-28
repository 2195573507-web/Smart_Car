# Findings: S3 Console and SRP UART2 Handshake

## Confirmed

- Before regeneration, the local `ESPS3/sdkconfig` selected
  `CONFIG_ESP_CONSOLE_UART_DEFAULT=y`, `CONFIG_ESP_CONSOLE_UART=y`, and UART
  number 0; that historical state left GPIO43/44 reserved by UART0.
- The current regenerated `ESPS3/sdkconfig` selects
  `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` with no secondary console, matching
  the intended defaults. ESP-IDF 5.5.4's Kconfig choice confirms that USB must
  be the primary choice for bidirectional console input.
- S3 UART2 source configures `UART_NUM_2`, TX GPIO17, RX GPIO18, 921600 8N1,
  2048-byte RX/TX driver buffers, and an RX task calling `uart_read_bytes`.
- STM32 `uart_link_init()` configures USART2, DMA1 Stream0/1 with USART2 RX/TX
  requests, starts `HAL_UARTEx_ReceiveToIdle_DMA`, and places DMA storage in
  the linker `.dma_buffer` section at D2 SRAM origin `0x30000000`.
- STM32 main starts `uart_link_init()` before the scheduler and starts the UART
  worker task before the S3 service. The current source has no RX-event/TX-DMA
  counters exposed in the log, so `rec=0` cannot distinguish wire silence from
  missing service parsing.

## Hypotheses to verify on hardware

- The latest S3 image may have been built from the stale `ESPS3/sdkconfig`,
  leaving UART0 on GPIO43/44 despite the defaults edit.
- If S3 `tx_bytes` rises but STM32 `uart_rx_bytes`/RX-event count stays zero,
  the loss is between S3 TX and STM RX (wiring, level, baud, or pin mux).
- If STM32 RX events rise but `SRP_HOST_SYNCED` never appears, the loss is in
  the STM32 ring/parser/service path; if `BOOT_INFO enqueue failed` appears,
  the STM32 TX queue/DMA path is the next boundary.

## 2026-08-24 boot-monitor follow-up

- The current `ESPS3/build/bootloader/bootloader.bin` is a valid 24,848-byte
  ESP32-S3 image with entry `0x403c8990`; the current app image is a valid
  726,048-byte image with entry `0x4037578c`. The flash log records a verified
  write for all four images at `0x0`, `0x30000`, `0x8000`, and `0x29000`.
- The current bootloader Kconfig resolves to USB Serial/JTAG as the primary
  console (`ESP_CONSOLE_USB_SERIAL_JTAG=true`, ROM serial port `4`, no
  secondary console). The prior same-board successful image resolved to UART0
  primary with USB Serial/JTAG secondary. This is the material configuration
  difference associated with the new silent monitor session.
- The observed session reaches the ROM `entry 0x403c8990` line but has no IDF
  `boot:` banner. Current source scans show no GPIO19/20 reconfiguration and
  `CONFIG_PM_ENABLE` is disabled, so application sleep or USB-pin ownership is
  not supported as the immediate cause by source evidence.
- The old monitor process still owns `/dev/ttys001`, while the configured
  `/dev/cu.usbmodem5B901601171` node is absent. A manual BOOT (GPIO0 low) plus
  reset/re-enumeration and read-only `chip_id`/`flash_id` query are required
  before choosing a console configuration patch.

## 2026-08-24 device re-enumeration follow-up

- Both `/dev/cu.usbmodem5B901601171` and `/dev/tty.usbmodem5B901601171` are
  present again. A separate `/dev/cu.usbmodem143101` pair is also present and
  must not be assumed to be the S3 target.
- No process currently owns the configured S3 callout device. The only stale
  shell association is `/dev/ttys001`; it is not evidence that the target USB
  callout is open.

## 2026-08-24 native USB console port identification

- PySerial identifies `/dev/cu.usbmodem5B901601171` as VID `0x1A86`, Product
  `USB Single Serial`; it is an external USB-UART path and is suitable for
  esptool flashing, but it is not the ESP32-S3 native USB Serial/JTAG console.
- PySerial identifies `/dev/cu.usbmodem143101` as VID `0x303A`, PID `0x1001`,
  Product `USB JTAG/serial debug unit`, serial `90:E5:B1:CC:EE:40`; this is
  the native USB Serial/JTAG console port.
- Running `idf.py monitor` on `/dev/cu.usbmodem143101` produced the complete
  bootloader and application log, including `Calling app_main()`, `UART2 init`
  on GPIO17/18 at 921600 baud, and `UART2 ready`. The earlier silent capture
  on `5B...` was therefore a port-selection error, not a firmware console
  registration failure.

## 2026-08-25 STM32 RX audit

- Confirmed: current PA2/PA3 MSP setup uses GPIOA clock, AF7, AF push-pull,
  pull-up, and USART2 clock; the IOC assigns USART2 to CM7.
- Confirmed: current CM7 vector routes DMA1 Stream0/1 and USART2 to the UART
  link handlers, and the link buffer is linked at `0x30000000` in D2 SRAM.
- Confirmed: the HAL marks a normal-mode ReceiveToIdle transaction READY before
  invoking the Rx event callback, but DMA abort/UART reinitialization are task
  context operations in this project.
- Risk: HAL abort/deinit is not ISR-safe. The repair keeps the lightweight
  `HAL_UARTEx_ReceiveToIdle_DMA()` rearm in the Rx callback as required by the
  receive contract, while reserving full abort/deinit/reconfigure recovery for
  the UART worker; the callback still captures bytes and clears line errors.

## 2026-08-25 static verification

- A clean `STM32H757/CM7/build/Debug` build of `Smart_Car_H757_CM7` completed
  all 73 build steps with no compiler warnings or errors. The linked ELF
  contains `DMA1_Stream0_IRQHandler`, `DMA1_Stream1_IRQHandler`,
  `USART2_IRQHandler`, `HAL_UARTEx_RxEventCallback`, and
  `HAL_UART_ErrorCallback`.
- The linker places `s_dma_rx` at `0x30000000`, `s_tx_queue_buffer` at
  `0x30000200`, and `s_tx_dma_buffer` at `0x30004280` in `.dma_buffer`, ending
  at `0x30004480`; this is D2 SRAM and is reachable by DMA1.
- The Common/SRP codec host test passes with `-std=c11 -Wall -Wextra -Werror`,
  and `git diff --check` is clean. The existing ESP32-S3 Ninja build completes
  its app/bootloader size checks using the available py3.14 environment;
  fresh `idf.py` regeneration remains blocked by the missing configured py3.9
  environment.
- No live CM7 counters were captured after this ELF build. Hardware acceptance
  still requires flashing the matching CM7 image and observing STM
  `SRP_UART2_DIAG`/`SRP_S3` counters alongside the S3 `STM_UART_DIAG`.

## 2026-08-25 targeted RX-blocking follow-up

- Confirmed: the current `.dma_buffer` image footprint is `0x30000000` through
  `0x30004480`, but CM7 `MPU_Config()` only defines a 4 GB Region 0 with a
  subregion mask. There is no higher-priority region explicitly declaring D2
  DMA storage non-cacheable.
- Confirmed: `DMA_REQUEST_USART2_RX`/TX are 43/44 in the checked-in H7 HAL;
  `HAL_DMA_Init()` computes `DMAmuxChannel` and writes the request ID. The
  current `__HAL_LINKDMA()` placement after each init is therefore valid.
- Confirmed: PA3 is the IOC-selected USART2_RX pin. PD3/PD4 are legacy GPIO
  labels and no current source configures PD6 as USART2_RX. A wire mismatch
  cannot be repaired from firmware without board evidence.
- New runtime boundary: count DMA RX/TX and USART IRQ entries, count rejected
  Rx callbacks, and snapshot RX NDTR, DMA error code, DMAMUX request IDs, and
  USART error code. These values distinguish a silent pin from a dead DMA or
  parser path on the flashed image.
- Added PA3 runtime snapshot packing `MODER[7:6]`, `PUPDR[7:6]`,
  `AFR[15:12]`, and `IDR3` into one diagnostic word. Expected AF7 input with
  pull-up is `gpio=0x076` when the line is low and `gpio=0x176` when idle high;
  the exact value is read from the flashed image rather than assumed.
- STM32 log frames are intentionally blocked before host sync by the existing
  SRP state gate. Therefore these counters are debugger/USART1 evidence, not
  proof that S3 will display diagnostics while its RX remains at the startup
  self-test count.
