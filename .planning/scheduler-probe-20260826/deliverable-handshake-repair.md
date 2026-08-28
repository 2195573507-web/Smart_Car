# Handshake Repair Delivery Notes

## Intended outcome

For a valid S3 `CMD_SYNC_REQ`, CM7 must emit the 20-byte SRP v4
`RSP_BOOT_INFO` frame even while the ordinary STM32-to-S3 session gate is still
closed. Normal logs and business traffic remain gated until that response is
sent successfully.

## Verification boundary

The CM7 build, host SRP codec test, and static checks prove source/build
integration only. A matching flashed CM7/S3 pair, USART1 markers, and a logic
analyzer capture of the 0x08 request followed by the 0x09 response are still
required for physical acceptance.

## Build evidence

- `cmake --build STM32H757/CM7/build/Debug --target Smart_Car_H757_CM7 -j2`:
  passed; FLASH `189,232 B`, RAM `63,808 B`.
- Strict host SRP codec compilation and test passed with `-Wall -Wextra
  -Werror -pedantic`.
- `git diff --check` and `arm-none-eabi-nm -u` passed.
- The Debug ELF contains the required raw markers: `UART2_RX_ARM_OK`,
  `UART2_RX_ARM_PRIMASK`, `UART2_RX_ARM_BASEPRI`, `SCHEDULER_START`,
  `RTOS_PROBE_CREATE_OK`, `RTOS_PROBE_PRE_SYNC`, and `UART2_TX`.

## Bench acceptance

1. Flash the matching CM7 Debug image and matching S3 image; reset both.
2. On USART1, require `UART2_RX_ARM_OK`, `PRIMASK=0`, `BASEPRI=0`,
   `RTOS_PROBE_CREATE_OK`, and `SCHEDULER_START` before assessing UART2.
3. Drive/replay a valid S3 `CMD_SYNC_REQ` (type `0x08`) and require
   `UART2_TX=HAL_BEGIN`, then `HAL_DONE value=0x00000000`.
4. On the logic analyzer, require the 16-byte request followed immediately by
   the 20-byte `RSP_BOOT_INFO` type `0x09`, with the request sequence echoed
   in boot-info payload byte 4.
5. If `BOOT_INFO TX_FAIL` appears, retain its `state`, `err`, and `hal` fields
   together with the UART2 RX event/error counters; do not classify it as an
   electrical fault without the TX-pin capture.
