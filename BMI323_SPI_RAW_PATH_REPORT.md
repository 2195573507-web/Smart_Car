# BMI323 SPI Raw Path Report

Status: `SUPERSEDED` by `BMI323_SPI_TRACE_V2.md`. The V1 `tx0=0x80` command filter is no longer present in source.

## Source Fact

The active BMI323 source has no `bmi323_read_reg()` function. `bmi323_read_who_am_i()` calls `bsp_spi_write_read()` with `tx = {0x80, 0x00, 0x00}`. The actual `HAL_SPI_TransmitReceive()` call is inside `STM32H757/BSP/SPI/bsp_spi.c::bsp_spi_write_read()`.

## Diagnostic Location

The diagnostic is immediately after `HAL_SPI_TransmitReceive()` returns, before the existing HAL-to-BSP status mapping and return. It therefore executes for both HAL success and failure whenever the BMI323 WHO_AM_I command (`tx0=0x80`) reached the HAL call.

The existing SPI BSP is shared, so the diagnostic explicitly matches `tx0=0x80`. In the current initialization flow, this is the first BMI323 SPI read and encodes address `0x00`.

## Output Contract

```text
[BMI323][SPI_RAW]
addr=0x00
tx0=0x80
tx1=0x00
rx0=0xXX
rx1=0xXX
hal=0/1
```

`diag_count` is a static counter and the output condition is `diag_count++ == 0`, so only the first matching transaction per MCU boot is emitted. `hal=0` means `HAL_OK`; `hal=1` means a non-OK HAL result. The unmodified `spi_last_hal_status` retains the complete HAL enum value.

## Transport

The output uses the existing `bsp_uart_log_write_link_level()` interface. It emits the established `SC_TYPE_LOG (0x30)` frame over USART2, where the existing S3 log bridge forwards it to FFE3 for the App logger.

The interface is available in the current BSP, so no `printf()` fallback was added.

## Scope And Impact

Changed files:

- `STM32H757/BSP/SPI/bsp_spi.c`
- `BMI323_SPI_RAW_PATH_REPORT.md`

BMI323 driver control flow, LSM303, IMU Manager, and calibration code are unchanged. The diagnostic allocates no heap memory, runs only after a blocking SPI transaction in normal task/startup context, and formats at most one log line per boot.

In the current source, the BMI323 caller releases CS after `bsp_spi_write_read()` returns. The required direct log send therefore extends the CS-low hold time for this one diagnostic transaction. It does not change command bytes, status mapping, or return behavior, but CS timing must be checked with a device capture before treating the result as hardware-safe.

## Verification

Run the CM7 configure, clean, and build sequence:

```sh
cd STM32H757/CM7
cmake --preset Debug
cmake --build build/Debug --target clean
cmake --build build/Debug -j2
```

A successful build only confirms compilation and linking. It does not confirm SPI clocks, chip select, MOSI/MISO levels, UART transport, BLE notification, or App display. No firmware flash is included.
