# STM32H757 Boot Log Design

## Scope

Add a CM7 startup trace for the existing USART1 path. The trace is best-effort
diagnostic output only: it must not change peripheral or sensor initialization
order, add delays, change FreeRTOS task parameters, or modify the BMI323 and
LSM303 drivers or the `imu_runtime.c` data path.

The logger emits one normalized line through the existing USART1 BSP logger:

```text
[BOOT][MODULE] STATUS t=<elapsed>ms
```

The public API is:

```c
void boot_log(const char *module, const char *status);
```

Elapsed time uses the existing HAL tick. The boot logger records its baseline
before the generated peripheral sequence. Because USART1 is currently
initialized after GPIO, I2C4, and timer setup, those early events are emitted
after USART1 becomes ready with their measured elapsed times; the ordering of
the hardware calls is unchanged.

## Startup events

`main.c` records `SYSTEM START`, then the completion of clock and GPIO setup,
the USART1 setup, and the UART BSP readiness point. After the existing runtime
startup function returns, it records `RTOS READY` and `SYSTEM READY` with the
total elapsed time. Existing unrelated boot text is removed from this path so
the normalized boot trace remains easy to parse.

`imu_manager.c` wraps only the current LSM303 initialization call with:

```text
[BOOT][IMU] INIT START
[BOOT][LSM303] INIT START
[BOOT][LSM303] DEVICE CHECK
[BOOT][LSM303] INIT OK|FAIL
[BOOT][BMI323] SKIPPED
[BOOT][IMU] READY
```

The BMI323 line is an explicit state marker. No BMI323 initialization,
SPI probe, or WHO_AM_I/CHIP_ID read is added.

`imu_runtime.c` reports the result at each existing `xTaskCreate` call:

```text
[BOOT][TASK] IMU_TASK CREATE OK|FAIL
[BOOT][TASK] DEBUG_TASK CREATE OK|FAIL
```

Task entry points, priorities, stack sizes, argument values, and creation
order remain unchanged.

## Error handling and concurrency

Logging is best-effort and does not alter the status returned by existing
initialization APIs. The module uses the existing UART transmit mutex. No
logging call is added to the 100 Hz data publication path. A bounded local
format buffer prevents dynamic allocation and the module reports no data if
its arguments are invalid.

## Verification

Run a clean CM7 CMake/Ninja build and inspect the linked ELF and compile graph
for the new source. Static checks must confirm no BMI323 API call was added by
the boot-log change, the existing LSM303 call remains in place, and USART1 is
still configured for 115200 8N1. Flash and capture USART1 at 115200 8N1 when a
connected board is available. Hardware serial output and live LSM303 behavior
remain separate from build evidence.
