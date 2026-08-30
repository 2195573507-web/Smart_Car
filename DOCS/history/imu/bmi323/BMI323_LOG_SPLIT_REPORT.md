# BMI323 Log Split Report

## Scope

The BMI323 startup diagnostic was changed so the WHO_AM_I value and the raw
SPI bytes are emitted as two independent text log records. No SRPv4 UART,
USART2 UART bridge, S3, App, or other STM32 module was changed.

## Modified file

- `STM32H757/Middleware/Sensor/BMI323/bmi323.c`

The raw trace buffers now retain the two-byte WHO_AM_I transaction only. The
existing `whoami_trace_done` one-shot guard is set before either write, so the
pair is emitted at most once per process lifetime, including when the read
fails.

## Log records

Record 1:

```text
[BMI323][WHOAMI]
expected=0x43
actual=0xXX
result=BMI323_...
```

Record 2:

```text
[BMI323][SPI]
reg=0x00
tx0=0xXX
tx1=0xXX
rx0=0xXX
rx1=0xXX
status=XX
```

The format uses CRLF separators as in the existing logger. Worst-case format
lengths, including CRLF, are 76 bytes for the WHO_AM_I record and 85 bytes for
the SPI record; both are below `BSP_UART_LOG_TEXT_MAX` (96 bytes).

The existing result classification is unchanged:

- `BMI323_NO_RESPONSE_ZERO`
- `BMI323_BUS_FLOAT`
- `BMI323_INVALID_WHOAMI`

## Verification

Build-only verification (no flashing):

```sh
cd STM32H757/CM7
cmake --preset Debug
cmake --build build/Debug --parallel 2
git diff --check
```

Hardware response, UART delivery, and BLE visibility remain unverified by this
source/build-only change.
