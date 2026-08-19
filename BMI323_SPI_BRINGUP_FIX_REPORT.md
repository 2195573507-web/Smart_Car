# BMI323 SPI Bring-up Fix Report

## Scope

This repair is limited to the BMI323 bottom driver and port adapter. No
changes were made to `imu_manager.c`, LSM303, attitude code, or communication
protocol code.

## Modified Files

- `STM32H757/Middleware/Sensor/BMI323/bmi323.c`
- `STM32H757/Middleware/Sensor/BMI323/bmi323.h`
- `STM32H757/Middleware/Sensor/BMI323/bmi323_port.c`

`STM32H757/Middleware/Sensor/BMI323/bmi323_port.h` was inspected and remains
unchanged; it is listed in the authorized scope but required no interface edit.

## SPI Parameters

The existing BSP SPI1 initialization already selects the required electrical
format and is left unchanged:

| Parameter | Value |
| --- | --- |
| SPI instance | SPI1 |
| Mode | Mode 0 |
| CPOL | 0 |
| CPHA | 0, first edge |
| Frame | 8-bit |
| Bit order | MSB first |
| BMI323 bring-up prescaler | `/256` |
| Target clock | `240 MHz / 256 = 937.5 kHz` (below 1 MHz) |

Because the BSP owns a private HAL handle and is outside the authorized edit
set, `bmi323_port_init()` applies the `/256` MBR value directly to the disabled
SPI1 peripheral after `bsp_spi_init()` completes. The port does not replace or
export the BSP HAL handle.

## Read Protocol

`bmi323_read_reg()` now holds CS low for one complete transaction and calls the
existing full-duplex HAL-backed BSP operation with `len + 1` bytes:

```text
TX: [address | 0x80, 0x00, ...payload clocks...]
RX: [ignored, data[0], ...data[len - 1]]
```

The returned payload is copied from `rx[index + 1]`. CS setup and hold delays
are 2 us, and CS is explicitly driven high after every read or write, including
error paths.

## Initialization and Diagnostics

`bmi323_init()` initializes the CS GPIO, drives CS high, waits 10 ms, and calls
the one-shot `bmi323_spi_probe()` for register `0x00`. The probe emits:

```text
[BMI323_PROBE]
tx0=0x80
tx1=0x00
rx0=0xXX
rx1=0xXX
```

Only `rx1` is compared with `0x43`. A mismatch or SPI failure logs
`WHOAMI_FAIL` and returns before reset or later sensor configuration.

The retained counters `read_ok`, `read_fail`, and `last_status` remain in the
diagnostics structure. The new `last_whoami`, `last_rx0`, and `last_rx1` fields
are included in `[BMI323][DEBUG]` output.

## Build Verification

Command:

```text
cmake --build STM32H757/CM7/build/Debug --parallel 2
```

Result: **PASS**. The CM7 ELF linked successfully. `nm` confirms:

```text
0800b760 T bmi323_spi_probe
```

The ELF also contains the probe/failure/configuration diagnostic strings.

## Hardware Validation

**Not tested.** No firmware was flashed and no logic-analyzer, UART runtime,
sensor response, or physical WHO_AM_I capture was available. Therefore this
report provides source/build evidence only; it does not claim that a connected
BMI323 has already returned `0x43` on hardware.
