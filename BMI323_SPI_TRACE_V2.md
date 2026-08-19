# BMI323 SPI Trace V2

Status: `IMPLEMENTED` source change. SPI electrical behavior, log transport, BLE delivery, and App display are `UNVERIFIED`; this task does not flash or monitor a device.

## Scope

Changed files:

- `STM32H757/BSP/SPI/bsp_spi.c`
- `BMI323_SPI_TRACE_V2.md`

The BMI323 driver logic, LSM303, IMU Manager, and calibration code are unchanged.

## Capture Location

`bsp_spi_write_read()` owns the active SPI1 `HAL_SPI_TransmitReceive()` call. The V2 capture begins immediately after that function returns and before the existing HAL-to-BSP status mapping and return.

There is no `tx0` or register-address filter. `bsp_spi_write_read()` uses the SPI1 handle (`hspi1_bsp`), and its static `diag_count++ == 0` gate records only the first SPI1 transaction that reaches the HAL call after boot.

## Output

```text
[BMI323][SPI_TRACE]
len=xx
tx:
xx xx xx xx

rx:
xx xx xx xx

hal=x
```

`len` is the full transaction length. The trace prints the first four TX and RX slots. If `len` is smaller than four, slots beyond `len` are zero-filled and are not transmitted or received bytes. If `len` exceeds four, the remaining bytes are intentionally omitted from this bounded one-shot diagnostic.

`hal` is the raw `HAL_SPI_TransmitReceive()` return enum stored in `spi_last_hal_status`, before the existing BSP status mapping.

## Transport And Impact

The trace uses the existing `bsp_uart_log_write_link_level()` interface, which emits the established `SC_TYPE_LOG (0x30)` frame through USART2 and the existing S3 FFE3/App logger path. No protocol, message ID, or BLE code changes are made.

No heap allocation, DMA ownership, or interrupt behavior is added. The trace uses fixed 4-byte snapshots and one local format buffer once per boot. In the current source, the caller releases BMI323 CS after `bsp_spi_write_read()` returns, so the direct log send extends the CS-low time of the captured transaction. This does not change transaction bytes, HAL status, or return behavior, but requires device waveform validation before it can be treated as hardware-safe.

## Verification

```sh
cd STM32H757/CM7
cmake --preset Debug
cmake --build build/Debug --target clean
cmake --build build/Debug -j2
```

Build success verifies compilation and linking only. It does not prove SPI clocks, CS timing, MOSI/MISO values, UART forwarding, BLE notification, or App display. No firmware flash is part of this task.
