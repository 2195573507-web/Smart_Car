# BMI323 Zero-Delay Fix Report

## Change

Modified only the BMI323 port delay implementation:

`STM32H757/Middleware/Sensor/BMI323/bmi323_port.c:173-195`

`bmi323_port_delay_ms(delay_ms)` now checks `delay_ms == 0U` before calling
`timer_get_ms()` or evaluating the existing delay loop. The zero-delay path
returns without entering the tick-based wait.

```c
if (delay_ms == 0U) {
    if (bmi323_port_zero_delay_logged == 0U) {
        bmi323_port_zero_delay_logged = 1U;
        (void)uart_log_write("[BMI323_DELAY]\\r\\nskip_zero_delay\\r\\n",
                             BMI323_PORT_LOG_TIMEOUT_MS);
    }
    return;
}
```

The one-shot flag is set before the log call, so repeated zero-delay requests
do not emit a second validation record.

## Runtime Output

The first call with `delay_ms == 0U` emits:

```text
[BMI323_DELAY]
skip_zero_delay
```

No further zero-delay calls emit this record. The requested validation log
uses the existing `uart_log_write()` interface and existing 100 ms log timeout;
the zero path does not execute a delay tick read or busy-wait loop. The log
call itself remains an intentional, requested diagnostic operation.

## Nonzero Behavior

For `delay_ms != 0U`, the code retains its previous behavior unchanged:

1. Read `start = timer_get_ms()`.
2. Emit `[BMI323_DELAY] start ms=...`.
3. Wait while `(timer_get_ms() - start) < delay_ms`.
4. Emit `[BMI323_DELAY] done`.

No changes were made to `bmi323.c`, `imu_manager.c`, SPI configuration,
LSM303, timeout values, or communication protocol.

## Verification

The CM7 target was clean-built with:

```sh
cd /Users/zhiqin/Projects/Smart_Car/STM32H757/CM7
cmake --build build/Debug --target clean
cmake --preset Debug
cmake --build build/Debug --parallel 4
```

All 60 build steps completed and
`build/Debug/Smart_Car_H757_CM7.elf` linked successfully. Reported usage:
Flash `100468 B / 1 MB` (9.58%), RAM `40128 B / 128 KB` (30.62%).
`git diff --check` completed without whitespace errors.

No firmware was flashed and no device runtime log was captured. The one-shot
log and immediate no-loop return are source- and build-verified; target UART
delivery and runtime tick behavior remain unverified.
