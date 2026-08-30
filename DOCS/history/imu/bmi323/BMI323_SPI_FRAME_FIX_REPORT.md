# BMI323 SPI Read Frame Fix Report

Date: 2026-08-09

## Scope

Modified only the active BMI323 driver:

- `STM32H757/Middleware/Sensor/BMI323/bmi323.c`

Not modified:

- IMU Manager
- LSM303
- attitude
- calibration
- protocol
- SPI mode, IOC, BSP SPI transaction ownership, or CS handling

## Reason

`BMI323_BOSCH_SPI_COMPARE.md` established that the active driver placed the
address byte inside its full-duplex HAL buffer but requested only `len + 1`
bytes and copied from `rx[index + 1]`. This omitted the clock byte required
after the command/dummy prefix.

## Change

`bmi323_read_reg()` now uses this wire-frame layout:

```text
TX[0]     = reg | 0x80
TX[1]     = 0x00
TX[2..]   = 0x00
RX[0]     = command phase
RX[1]     = dummy phase
RX[2..]   = register payload
length    = len + 2
```

The TX/RX stack buffers were increased from `BMI323_MAX_READ_BYTES + 1` to
`BMI323_MAX_READ_BYTES + 2`, preserving the 26-byte maximum payload without
an out-of-bounds access.

CS LOW -> one `HAL_SPI_TransmitReceive()` -> CS HIGH remains in
`bmi323_port_spi_read()` and `bsp_spi_write_read()`. SPI mode remains mode 0
in the unchanged BSP configuration.

## Diagnostic

The existing one-shot WHO_AM_I trace now records three RX bytes and emits one
additional bounded record after the transaction:

```text
[BMI323][SPI_FRAME]
frame_length=<n>
rx0=0xNN
rx1=0xNN
rx2=0xNN
```

The format buffer is 96 bytes and the maximum formatted record is below the
existing 96-byte UART/SRP text limit. It executes after the port transaction
has released CS.

## Impact and Risk

- The first 1-byte CHIP_ID read now clocks `80 00 00` and reads `rx[2]`.
- Multi-byte reads gain exactly one leading dummy/prefix byte; payload offsets
  move from `rx[1..]` to `rx[2..]`.
- The added byte increases each read transfer by one SPI byte. At the static
  1.875 MHz configuration, this is approximately 4.3 microseconds per read;
  the existing blocking HAL call, 20 ms timeout, CS timings, and task policy
  are otherwise unchanged.
- Build/static evidence cannot prove physical SPI waveform, chip identity, or
  sensor operation.

## Verification

| Check | Result |
| --- | --- |
| `cmake --preset Debug` from `STM32H757/CM7` | PASS |
| `cmake --build build/Debug --target Smart_Car_H757_CM7 --clean-first -j2` | PASS; 59/59 compile/link steps |
| `git diff --check` | PASS |
| Read buffer bounds | PASS; max payload 26 plus two prefix bytes fits `+2` buffers |
| Protected module audit | PASS; this task made no edits to IMU Manager, LSM303, attitude, calibration, or protocol. Pre-existing worktree changes in some protected paths were preserved and are not part of this task. |
| Flash/reset/serial/SPI waveform/device operation | NOT RUN by request |

Build artifact: `STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf`.
