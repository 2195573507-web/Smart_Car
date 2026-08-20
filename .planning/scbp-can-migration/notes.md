# SCBP-CAN Migration Notes

## Confirmed Decisions

- Scheme A is approved: replace the STM32H757 <-> ESP32-S3 UART V3 protocol in
  full, without `SC_TYPE_*` compatibility adapters.
- App BLE remains a separate envelope; the S3 may re-envelope new UART payloads
  but does not forward raw SCBP-CAN frames as App BLE frames.
- SCBP-CAN header/trailer are fixed at 8/4 bytes and payload begins at offset 8.
- The UART link reserves destination node value `3` for broadcast; application
  node addressing is not used on this physical UART transport.
- The 10-bit message field requires explicit mappings: `CAL_EVENT=0x001` and
  `LOG=0x3F0`; old `0x0401` and `0xF000` are not silently truncated.

## Preservation Rules

- Existing working-tree changes, especially removal of vibration-related IMU
  behavior, belong to the user and must not be restored.
- Source/build checks cannot be reported as UART, BLE, sensor, radar, PWM, or
  vehicle acceptance.

## Verification

- Shared host test passes with C11 `-Wall -Wextra -Werror -pedantic`.
- CM7 Debug configure/build passes with `SMARTCAR_BMI323_DEBUG_ONLY=OFF`.
- ESP-IDF 5.5.4 S3 build passes in `ESPS3/build`.
- Active source scan has no `SC_TYPE_*`, `sc_frame`, `scbp_frame`, or
  `SC_FRAME_*` references. Active protocol docs now describe SCBP-CAN; V3
  references are historical/deprecated.
- No device flash, monitor capture, UART waveform, BLE capture, sensor test,
  radar/PWM test, or vehicle test was performed.
