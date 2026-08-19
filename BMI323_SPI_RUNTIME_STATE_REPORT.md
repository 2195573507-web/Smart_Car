# BMI323 SPI First-Access Runtime State Report

## Purpose

This diagnostic captures the first BMI323 SPI read around
`HAL_SPI_TransmitReceive()` so that a boot stop after `[BMI323][SPI_CONFIG]`
can be separated into pre-transfer, HAL-transfer, and returned paths. It is
diagnostic-only: SPI settings, CS control order, transaction contents,
timeouts, initialization order, protocol framing, LSM303, and `imu_manager`
are unchanged.

The active SPI handle is the BSP-private `hspi1_bsp`, not a generated global
`hspi1`. The required HAL reads therefore occur inside the SPI BSP.

## Instrumentation Locations

| Location | Responsibility |
| --- | --- |
| `STM32H757/Middleware/Sensor/BMI323/bmi323.c:208-217` | Emits `cs_before` immediately before `bmi323_port_spi_read()`. |
| `STM32H757/Middleware/Sensor/BMI323/bmi323.c:405-413` | Reads CS before/after the probe transaction and emits the returned-state report only after `bmi323_port_spi_read()` has finished. |
| `STM32H757/BSP/SPI/bsp_spi.c:139-150` | After CS has been asserted by the existing port layer and immediately before HAL, samples `cs_active`, `HAL_SPI_GetState(&hspi1_bsp)`, `HAL_SPI_GetError(&hspi1_bsp)`, and `SPI1->SR`. |
| `STM32H757/BSP/SPI/bsp_spi.c:152-168` | Immediately after the HAL call returns, samples state/error/SR, raw HAL result, and `rx0`/`rx1`. |
| `STM32H757/BSP/SPI/bsp_spi.c:201-209` | Returns the captured one-shot record to BMI323 after the port layer has released CS. |

`bmi323_spi_probe()` remains guarded by its existing `bmi323_probe_done` flag.
The BSP capture is additionally gated by `diag_count == 0U`; `diag_count` is
incremented after the first `HAL_SPI_TransmitReceive()` returns. Invalid or
not-ready calls do not consume the one-shot capture.

## Runtime Output

Every record uses the existing log transport and begins with
`[BMI323_SPI_STATE]`. Records are split to remain at or below the existing
96-byte text payload limit.

```text
[BMI323_SPI_STATE]
cs_before=<0|1>

[BMI323_SPI_STATE]
cs_active=<0|1>
spi_state_before=<HAL_SPI_StateTypeDef numeric value>

[BMI323_SPI_STATE]
spi_error_before=0xXXXXXXXX
spi1_sr_before=0xXXXXXXXX

... HAL_SPI_TransmitReceive() ...

[BMI323_SPI_STATE]
cs_before=<0|1>
cs_active=<0|1>
cs_after=<0|1>

[BMI323_SPI_STATE]
spi_state_before=<value>
spi_error_before=0xXXXXXXXX

[BMI323_SPI_STATE]
spi1_sr_before=0xXXXXXXXX

[BMI323_SPI_STATE]
spi_state_after=<value>
spi_error_after=0xXXXXXXXX

[BMI323_SPI_STATE]
spi1_sr_after=0xXXXXXXXX
hal_result=<HAL_StatusTypeDef numeric value>

[BMI323_SPI_STATE]
rx0=0xXX
rx1=0xXX
```

`cs_before` is read before the existing port transaction, `cs_active` is read
after the existing CS-low operation and its 2 us setup delay, and `cs_after`
is read after the existing transaction finalizer drives CS high. GPIO values
use `0` for low and `1` for high.

The first three records are sent before the HAL call. Therefore, if the
system stops inside `HAL_SPI_TransmitReceive()`, they remain observable. The
`spi_state_after`, `spi_error_after`, `SPI1->SR` after-value, `hal_result`,
and RX bytes cannot exist until that function returns.

## Interpreting Values

| Field | Interpretation |
| --- | --- |
| `spi_state_before` | Expected normal entry value is `HAL_SPI_STATE_READY` (`1`). A non-ready value causes HAL to return `HAL_BUSY` rather than entering its polling transfer. |
| `spi_state_after` | A returned call normally restores state to ready; preserve the numeric value for direct comparison with HAL state constants. |
| `spi_error_before/after` | Raw `HAL_SPI_GetError()` bitmask. `0x00000100` is HAL SPI timeout; do not infer a physical bus failure from this value alone. |
| `spi1_sr_before/after` | Direct `SPI1->SR` snapshots before and after the blocking HAL call. |
| `hal_result` | Raw `HAL_StatusTypeDef`: `0` is `HAL_OK`, `2` is `HAL_BUSY`, and `3` is `HAL_TIMEOUT`. |
| `rx0/rx1` | Raw bytes from the first probe transaction (`tx={0x80,0x00}`); they are reported without changing BMI323 parsing. |

## Timing and Scope

The pre-HAL records use the established link-level log function and are
emitted after CS is low. This intentionally exposes the last reachable point
when the HAL call does not return. It adds one bounded diagnostic emission to
the first transaction only; no transaction parameters or CS write operations
are changed. The returned-state records are emitted only after the existing
port finalizer has released CS.

No changes were made to:

- `STM32H757/Middleware/Sensor/imu_manager.c`
- LSM303 code
- SPI1 configuration or CubeMX/IOC files
- BMI323 transaction format, timeout (`20 ms`), initialization ordering, or
  communication protocol

## Verification

The following clean CM7 build completed successfully:

```sh
cd /Users/zhiqin/Projects/Smart_Car/STM32H757/CM7
cmake --build build/Debug --target clean
cmake --preset Debug
cmake --build build/Debug --parallel 4
```

Result: all 60 build steps completed and
`build/Debug/Smart_Car_H757_CM7.elf` linked successfully. Reported usage was
Flash `100220 B / 1 MB` (9.56%) and RAM `40128 B / 128 KB` (30.62%).
`git diff --check` also completed without whitespace errors.

No firmware was flashed and no target, UART, BLE, SPI waveform, or hardware
register capture was performed. Consequently, this report defines the runtime
evidence that will be produced on target; it does not claim any current
`HAL_SPI_GetState()`, `HAL_SPI_GetError()`, `SPI1->SR`, CS, or RX value.
