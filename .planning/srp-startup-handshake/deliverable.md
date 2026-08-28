# Delivery Notes

## Source changes

- `ESPS3/components/smartcar_service/command_bridge.c`: dispatch and validate
  `RSP_BOOT_INFO`, enter `SYNCED`, retry sync after timeout, and keep motion
  transport gated until synchronization.
- `STM32H757/Middleware/Communication/Services/s3_service.c`: validate all
  sync payload bytes and enqueue the echoed `RSP_BOOT_INFO` response.
- `STM32H757/Middleware/Communication/UART_Link/uart_link.c`: circular DMA
  receive event handling, cache/error recovery, priority TX DMA queues, and
  safe event-type lookup after handle validation.
- `ESPS3/sdkconfig.defaults`: select USB Serial/JTAG as the primary console so
  UART1 RX GPIO44 is reserved for radar.

## Verification

- `cc -std=c11 -Wall -Wextra -Werror -pedantic ... Common/SRP/tests/test_srp_codec.c`:
  passed.
- `idf.py -D SDKCONFIG=build-srp-handshake-verify/sdkconfig -D SDKCONFIG_DEFAULTS=sdkconfig.defaults -B build-srp-handshake-verify build`:
  passed; generated `smartcar_s3_gateway.bin` and confirmed USB Serial/JTAG
  console with `CONFIG_ESP_CONSOLE_UART_NUM=-1`.
- `cmake --build STM32H757/CM7/build/Debug --target Smart_Car_H757_CM7 -j2`:
  passed; generated `Smart_Car_H757_CM7.elf`.
- `git diff --check`: passed.

## Acceptance boundary

No device was flashed. Matching image versions, bidirectional 921600-bps
capture, and runtime `SRP SYNCED`/motion acceptance still require bench
verification.
