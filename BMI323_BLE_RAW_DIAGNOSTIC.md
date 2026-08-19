# BMI323 BLE Raw Diagnostic

Status: `IMPLEMENTED` source change; runtime, UART, BLE, and hardware behavior are `UNVERIFIED` because this task does not flash or monitor a device.

## Scope

Only `STM32H757/Middleware/Sensor/BMI323/bmi323.c` is changed. The change does not modify LSM303, IMU Manager, IMU Calibration, ATTITUDE, SCBP framing, message IDs, or BLE code.

## Failure Path

`bmi323_init()` calls `bmi323_read_who_am_i()` with the SPI read command for register `0x00`. A failure is reported when the SPI transaction fails, or when the received value differs from the expected BMI323 ID `0x43`.

Before either existing failure return, the driver emits this text once for the lifetime of the boot image:

```text
[BMI323][RAW]
reg=0x00
tx=0x80
rx=0xXX
spi_status=XX
```

`rx` is the received WHO_AM_I byte (`rx[2]` after the command and dummy phases). `spi_status` is the existing `bsp_spi_write_read()` status value captured in the WHO_AM_I trace.

## Transport

The diagnostic uses the existing STM32 `bsp_uart_log_write_link_level()` interface at error level. That interface serializes the established `SC_TYPE_LOG (0x30)` payload and sends it through STM32 USART2. The existing S3 `log_bridge_handle()` forwards valid log records through FFE3, which the App parses with `SmartCarLogParser`.

No BLE characteristic, BLE payload format, SCBP message ID, frame type, or parser is added or changed.

## Existing Diagnostics

The existing IMU Manager output remains unchanged:

```text
[BMI323][DEBUG]
read_ok=...
read_fail=...
last_status=WHO_AM_I_FAIL
```

The raw output has its own static one-shot guard. Subsequent failed reads during the same boot do not add raw diagnostic frames or alter the existing read counters/status values.

## Impact And Verification

The change adds one startup-path format operation and at most one existing LOG frame. It performs no allocation, does not run in an interrupt, and does not change sensor control, retry, initialization, or calibration state.

Verification is limited to source inspection and the CM7 build. A successful build does not prove SPI electrical activity, STM32-to-S3 UART delivery, BLE notification, or App display. No firmware flash is part of this task.
