# BMI323 RAW Runtime Check Report

> Historical snapshot: the path table below predates the SRPv4 full switch and
> is retained as diagnostic evidence only. It is not a current UART contract.

## Scope and conclusion

This is a read-only check. No C/C++ source, CMake file, protocol, UART
bridge, S3, App, or build configuration was changed. The report itself is the
only new file.

The local Debug ELF at
`STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf` contains the current split
diagnostic strings:

```text
[BMI323][WHOAMI]
[BMI323][SPI]
```

It does not contain the requested historical/force markers:

```text
WHOAMI_RAW       absent
[BMI323][SPI_RAW] absent
WHOAMI_FORCE     absent
READ_REG_TRACE   absent
```

The token `SPI_RAW` is present only as the substring of the compiled symbol
`BMI323_SPI_RAW_BYTES`; it is not present as a `[BMI323][SPI_RAW]` log string.

Therefore, a device printing the old `[BMI323][WHOAMI_RAW]`/`[BMI323][SPI_RAW]` records cannot be
running this local Debug ELF. ELF contents alone cannot prove which image is
currently flashed on the target.

## 1. Source marker search

The source search returned no `WHOAMI_RAW`, `WHOAMI_FORCE`, or
`READ_REG_TRACE` definitions/string literals. `SPI_RAW` only matches the
substring in the raw-buffer symbol `BMI323_SPI_RAW_BYTES` at
`STM32H757/Middleware/Sensor/BMI323/bmi323.c:10,28-29,54-55`; there is no
`[BMI323][SPI_RAW]` log string.

Historical Markdown reports still contain those words; they are documentation
and are not compiled. Examples include:

| File | Lines | Classification |
| --- | --- | --- |
| `BUILD_FIRMWARE_TRACE_REPORT.md` | 17-22, 71-76, 107-111 | historical build report |
| `BMI323_WHOAMI_RAW_REPORT.md` | 33, 38, 64 | historical raw-log report |
| `BMI323_SPI_RAW_PATH_REPORT.md` | 18, 40 | historical path report |
| `BMI323_LOG_PATH_REPORT.md` | 13, 60, 64 | historical logger report |

## 2. ELF check

Command used:

```sh
strings STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf | grep BMI323
```

The relevant output is:

```text
[BMI323][SPI_TRACE]
BMI323_NO_RESPONSE_ZERO
BMI323_BUS_FLOAT
BMI323_WHOAMI_OK
BMI323_INVALID_WHOAMI
[BMI323][WHOAMI]
[BMI323][SPI]
[BMI323][DEBUG]
[BMI323][INIT]
[BMI323][SPI_CONFIG]
[BMI323][CS] %s
BMI323_CS_LOW
BMI323_CS_HIGH
[BMI323][ERROR]
[BMI323][INIT]
[BMI323][INIT]
[BMI323][INIT]
[BMI323][DEBUG]
/Users/zhiqin/Projects/Smart_Car/STM32H757/Middleware/Sensor/BMI323
/Users/zhiqin/Projects/Smart_Car/STM32H757/Middleware/Sensor/BMI323
/Users/zhiqin/Projects/Smart_Car/STM32H757/Middleware/Sensor/BMI323
BMI323_SPI_RAW_BYTES UINT16_C(2)
BMI323_CS_Pin GPIO_PIN_4
BMI323_INT1_GPIO_Port GPIOB
BMI323_INT1_Pin GPIO_PIN_2
BMI323_CS_GPIO_Port GPIOC
BSP_GPIO_BMI323_CS
BSP_GPIO_BMI323_INT1
BMI323_DIAG_STATUS_CONFIG_FAIL
BMI323_REG_ACC_DATA_X UINT8_C(0x03)
BMI323_DIAG_STATUS_SPI_RX_FAIL
BMI323_DIAG_STATUS_DATA_NOT_READY
BMI323_REG_TEMP_DATA UINT8_C(0x09)
BMI323_DIAG_STATUS_SPI_READ_FAIL
BMI323_WHO_AM_I_REG BMI323_REG_CHIP_ID
BMI323_ERROR_WHO_AM_I_MISMATCH
BMI323_SOFT_RESET_LSB UINT8_C(0xAF)
BMI323_ERROR_GYRO_CONFIG
BMI323_DIAG_STATUS_SPI_TX_FAIL
BMI323_TEMP_SCALE 512.0f
BMI323_ERROR_NONE
BMI323_ERROR_DATA_WRITE
BMI323_DIAG_STATUS_WHO_AM_I_VALUE_ERROR
BMI323_ERROR_WHO_AM_I_TIMEOUT
SMARTCAR_SENSOR_BMI323_REG_H
BMI323_ACC_RANGE_G 4.0f
BMI323_GRAVITY_MPS2 9.80665f
BMI323_REG_GYR_DATA_X UINT8_C(0x06)
BMI323_STATUS_GYR_DATA_READY UINT8_C(0x40)
BMI323_SPI_READ_MASK UINT8_C(0x80)
BMI323_ERROR_POST_RESET_READ
BMI323_REG_GYR_CONF UINT8_C(0x21)
BMI323_RESET_DELAY_MS UINT32_C(2)
BMI323_ERROR_WHO_AM_I_VALUE
BMI323_DEG_TO_RAD 0.01745329251994329577f
SMARTCAR_SENSOR_BMI323_H
BMI323_DIAG_STATUS_WHO_AM_I_FAIL
BMI323_REG_STATUS UINT8_C(0x02)
BMI323_ERROR_ACCEL_CONFIG
/Users/zhiqin/Projects/Smart_Car/STM32H757/Middleware/Sensor/BMI323/bmi323.c
BMI323_LOG_TIMEOUT_MS UINT32_C(100)
BMI323_DIAG_STATUS_WHO_AM_I_TIMEOUT
BMI323_ERROR_SOFT_RESET
BMI323_MAX_WRITE_BYTES UINT16_C(2)
BMI323_ERROR_SPI_RX_FAIL
BMI323_MAX_READ_BYTES UINT16_C(26)
SMARTCAR_SENSOR_BMI323_PORT_H
BMI323_REG_CMD UINT8_C(0x7E)
BMI323_SPI_TIMEOUT_MS UINT32_C(20)
BMI323_ERROR_WHO_AM_I_READ
BMI323_SOFT_RESET_MSB UINT8_C(0xDE)
BMI323_ERROR_DATA_READ
BMI323_ERROR_SPI_TX_FAIL
BMI323_GYRO_RANGE_DPS 500.0f
BMI323_WHO_AM_I_VALUE UINT8_C(0x43)
BMI323_REG_CHIP_ID UINT8_C(0x00)
BMI323_TEMP_OFFSET_C 23.0f
BMI323_DIAG_STATUS_OK
BMI323_REG_ACC_CONF UINT8_C(0x20)
BMI323_ERROR_SPI_INIT
BMI323_STATUS_ACC_DATA_READY UINT8_C(0x80)
BMI323_PORT_CS_DELAY_US UINT32_C(2)
BMI323_PORT_LOG_TIMEOUT_MS UINT32_C(100)
/Users/zhiqin/Projects/Smart_Car/STM32H757/Middleware/Sensor/BMI323/bmi323_port.c
```

The ELF timestamp checked was `2026-08-09 09:57:46`.

The active target path is also confirmed by `STM32H757/CM7/CMakeLists.txt:24-25`
and the generated `build.ninja` entries for `bmi323.c` (line 202),
`bmi323_port.c` (line 212), and the final ELF link rule (line 420).

## 3. BMI323 log call path

| Stage | Evidence |
| --- | --- |
| BMI323 formatting | `bmi323.c:82-115` builds two records and calls `uart_log_write()` twice |
| WHO_AM_I call site | `bmi323.c:274-300` invokes the one-shot diagnostic during `bmi323_init()` |
| UART wrapper | `bsp_uart.h:68-71` maps `uart_log_write()` to `bsp_uart_log_write()` |
| Level | `bsp_uart.c:246-248` uses `BSP_UART_LOG_LEVEL_INFO` |
| USART2 framing | `bsp_uart.c:186-217` encodes `SC_TYPE_LOG` and calls `uart_link_send()` |
| Type mapping | `sc_frame.h:125` defines `SC_TYPE_LOG = 0x30`; `sc_frame.c:136` maps it to `SCBP_MSG_ID_LOG` |

The BMI323 records do not use `printf()` or the RTOS `log_service` queue.

## 4. Filtering and truncation

No `LOG_FILTER` or `log_filter` implementation was found in the STM32 C/H
sources. The logger only rejects an invalid level (`> ERROR`) and truncates
USART2 text longer than `BSP_UART_LOG_TEXT_MAX` (96 bytes) at
`bsp_uart.c:198-200`.

The current split records are below 96 bytes, so this source-level truncation
does not remove their fields. Transport queue failure or an uninitialized UART
can still prevent runtime delivery; that requires target logs to distinguish.

## Root-cause judgment

- **Confirmed:** the local Debug ELF is built from the split BMI323 logger and
  has no `WHOAMI_RAW`, `[BMI323][SPI_RAW]`, `WHOAMI_FORCE`, or
  `READ_REG_TRACE` strings. `SPI_RAW` only appears inside
  `BMI323_SPI_RAW_BYTES`.
- **Confirmed:** no BMI323-specific log filter exists in the inspected STM32
  path.
- **Not provable from source/ELF:** whether this exact ELF is the image flashed
  on the running board, whether `bmi323_init()` executes, and whether USART2,
  S3, BLE, or App delivery succeeds.

If runtime output still contains the old markers, the board is running another
image or an older flash artifact. If no split records appear, capture the
startup path and UART link status; do not infer that from this ELF inspection.
