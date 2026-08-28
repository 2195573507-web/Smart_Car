# SRP v4 Integration Manual

## Ownership

The shared contract lives in `Common/SRP/`. STM32 owns sensor, calibration,
attitude, chassis, motor and log producers. ESP32-S3 owns the UART gateway,
radar service, App BLE envelope and selected telemetry/log relay. Upper-layer
business services call `s3_service_send_message()` or `srp_link_send()` and do
not access UART DMA directly.

## Adding A Message

1. Reserve an unused 8-bit type in `Common/SRP/include/srp_registry.h`.
2. Define an exact payload length and, for fixed data, a `#pragma pack(4)` host
   view with static size assertions.
3. Define little-endian scalar/float serialization using `srp_wire.h`; never
   cast a byte buffer to a packed struct for unaligned reads.
4. Add the STM32 dispatch branch in `s3_service_on_frame()` and the ESP32
   branch in `command_bridge_on_frame()`. Validate length, reserved bytes,
   finite floats and state admission before changing business state.
5. Use `SRP_FLAG_ACK_REQUIRED` only for commands that need transaction
   confirmation. Return a four-byte ACK/ERROR response with the request type
   and sequence. Stream telemetry with `SRP_FLAG_STREAM_DATA`.
6. If the payload is extensible or low-rate, set `SRP_FLAG_TLV`; unknown tags
   must be skipped and malformed records rejected.
7. Update this manual, `SRP_v4_Spec.md`, and the active protocol index with
   direction, payload offsets, state/error behavior and verification commands.

## Testing

Run the host suite:

```sh
cc -std=c11 -Wall -Wextra -Werror -pedantic \
  -ICommon/SRP/include Common/SRP/srp_crc.c Common/SRP/srp_wire.c \
  Common/SRP/srp_codec.c Common/SRP/srp_link.c \
  Common/SRP/tests/test_srp_codec.c -o /tmp/test_srp_codec
/tmp/test_srp_codec
```

At CM7 startup, `s3_service`, `attitude_gate`, and `chassis` are admitted
independently of IMU task success. An IMU startup failure is reported in the
`IMU_CAL_STATUS` error field and keeps `g_attitude_is_ready=0`, so the chassis
continues publishing locked state after SRP sync while PWM remains stopped.
Pre-sync CM7 LOG frames are retained in a bounded queue and retried over the
SRP LOG path after synchronization; USART1 remains the local boot-log source.

Then build the CM7 Debug image and the ESP-IDF image. Treat these as source
integration evidence. Hardware acceptance additionally requires matching
flashed images, a bidirectional 921600-bps capture, EMERGENCY latency below
2 ms, dynamic baud synchronization below 500 ms, and the stated 24-hour CRC
stress run.

## Compatibility Rule

SCBP-CAN/V3 source and executable compatibility are removed. Historical
documents remain for traceability only and are marked deprecated. New code
must use SRP names and APIs directly.
