# BMI323 Delay Trace Report

## Scope

This change adds only BMI323 initialization-stage log records. It does not
change SPI configuration or access, CS control, timeout constants, protocol
framing, delay arguments, delay loop condition, or initialization branches.
No firmware was flashed.

## Added Trace Points

| File | Location | Output |
| --- | --- | --- |
| `STM32H757/Middleware/Sensor/BMI323/bmi323_port.c:172-185` | `bmi323_port_delay_ms()` entry | `[BMI323_DELAY] start ms=%lu` using the existing `timer_get_ms()` value. |
| `STM32H757/Middleware/Sensor/BMI323/bmi323_port.c:172-185` | `bmi323_port_delay_ms()` exit | `[BMI323_DELAY] done` after the existing delay loop exits. |
| `STM32H757/Middleware/Sensor/BMI323/bmi323.c:341` | Before the existing first 10 ms initialization delay | `[BMI323_INIT] before_delay` |
| `STM32H757/Middleware/Sensor/BMI323/bmi323.c:343` | After the existing first 10 ms initialization delay returns | `[BMI323_INIT] after_delay` |

All new records use the established `uart_log_write()` path and its existing
100 ms log timeout constant. No timeout value was changed.

## Expected Startup Sequence

After the existing `[BMI323][SPI_CONFIG]` output, the initial delay path should
produce the following sequence:

```text
[BMI323_INIT] before_delay
[BMI323_DELAY] start ms=<HAL tick>
[BMI323_DELAY] done
[BMI323_INIT] after_delay
```

`bmi323_port_delay_ms()` is also used for the existing 2 ms post-soft-reset
wait. That call emits only its own `BMI323_DELAY` start/done pair; the
`BMI323_INIT` markers deliberately surround only the first 10 ms delay after
SPI configuration.

## Runtime Interpretation

| Observed final record | Source-level conclusion |
| --- | --- |
| `[BMI323_INIT] before_delay` is absent | Execution did not reach the first initialization delay call. |
| `before_delay` appears but `BMI323_DELAY start` is absent | Execution did not enter `bmi323_port_delay_ms()` or stopped in the newly added log call. |
| `BMI323_DELAY start` appears but `BMI323_DELAY done` is absent | The original `while ((timer_get_ms() - start) < delay_ms)` loop did not complete. Inspect HAL tick/SysTick progress on target. |
| `BMI323_INIT after_delay` appears | The first 10 ms delay returned; diagnosis proceeds to `bmi323_spi_probe()`. |

This is a runtime decision aid only. A host build cannot establish actual HAL
tick movement, UART delivery, or the program counter on the STM32 target.

## Verification

The CM7 firmware was clean-built with:

```sh
cd /Users/zhiqin/Projects/Smart_Car/STM32H757/CM7
cmake --build build/Debug --target clean
cmake --preset Debug
cmake --build build/Debug --parallel 4
```

All 60 build steps completed and
`build/Debug/Smart_Car_H757_CM7.elf` linked successfully. Reported usage:
Flash `100396 B / 1 MB` (9.57%), RAM `40128 B / 128 KB` (30.62%).
`git diff --check` completed without whitespace errors.

No target was flashed and no UART, SPI, or device runtime trace was captured.
