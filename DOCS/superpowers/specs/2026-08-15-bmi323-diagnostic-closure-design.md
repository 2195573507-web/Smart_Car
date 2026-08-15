# BMI323 Diagnostic Closure Design

## Scope

Improve the active STM32H757 BMI323 startup diagnostic so a WHO_AM_I failure
can be distinguished between SPI transport failure, low-byte ID mismatch, and
high-byte/frame mismatch. The change is limited to
`STM32H757/Middleware/Sensor/BMI323/`.

The LSM303 path, IMU scheduling, SCBP/LOG transport, CubeMX configuration,
GPIO assignments, SPI transaction shape, and BMI323 failure isolation remain
unchanged.

## Current Evidence

The active probe sends a four-byte WHO_AM_I frame and validates `RX[2] == 0x43`
and `RX[3] == 0x00`. The periodic debug record only exposes `RX[0]` and
`RX[1]`, so a log containing `whoami=0x43` and
`WHO_AM_I_VALUE_ERROR` hides the byte that caused rejection.

The probe result is latched. A later initialization/recovery attempt can reuse
the first result and leave stale raw/status fields in the diagnostic output.

## Design

1. Keep the current four-byte probe and its `0x0043` validation contract.
2. Expose `RX[2]` and `RX[3]` in the bounded `[BMI323][DEBUG]` record.
3. Add explicit diagnostic statuses for low-byte and high-byte value errors,
   while retaining the existing generic enum value for compatibility.
4. Set the precise status in the probe before returning; transport errors keep
   their existing timeout/RX-failure classification.
5. Reset one-shot probe/trace state at the beginning of a new top-level init so
   a recovery attempt cannot report stale raw bytes. The first probe remains
   the only physical transaction in each init attempt.

## Error Handling

- `HAL_TIMEOUT` remains `WHO_AM_I_TIMEOUT`.
- Other HAL/BSP failures remain `SPI_RX_FAIL`.
- `RX[2] != 0x43` becomes `WHO_AM_I_LOW_BYTE_ERROR`.
- `RX[2] == 0x43 && RX[3] != 0x00` becomes
  `WHO_AM_I_HIGH_BYTE_ERROR`.
- A passing probe sets `BMI323_DIAG_STATUS_OK` and does not alter the
  manager-level failure isolation.

## Verification

- `git diff --check`.
- Clean CM7 configure/build for the active target.
- Static checks confirm only the active BMI323 middleware files and this spec
  are changed.
- Hardware acceptance remains pending until a flashed target provides the
  complete `[BMI][RAW]` RX frame or a CS/SCK/MOSI/MISO capture.
