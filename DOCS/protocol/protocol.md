# Protocol Module

## Function

Describe the framing and parser ownership at each transport boundary.

## Source Location

- STM32: `STM32H757/Middleware/Communication/SmartCar_Frame/`
- S3: `ESPS3/components/smartcar_protocol/`
- App: `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/Model/SmartCarProtocol.swift`

## Entry Files

`sc_frame.c`, `frame.c`, `parser.c`, and Swift `SmartCarProtocol.swift`.

## Data Input

Arbitrary fragmented byte chunks from UART or BLE.

## Data Output

Validated frame views, parser callbacks, CRC/error counters, and encoded bytes.

## External Interfaces

`sc_frame_encode/decode`, `sc_frame_parser_init/feed`, Swift
`SmartCarProtocol.encode`, and `SmartCarProtocol.Parser.feed`.

## Dependencies

`stdint`, bounded buffers, CRC16-MODBUS, transport-owned callbacks.

## Current Status

Both C endpoints implement the STM-S3 source envelope. The App implements the
BLE envelope. The codecs are source-established but do not by themselves prove
that a gateway translates and relays messages.

## Known Issues

The name AA55 is ambiguous: C starts with bytes `AA 55` and has no trailing
byte; Swift starts with `AA`, version `01`, and ends with `55`. Type IDs differ
for several calibration/radar messages. This is a high-priority integration
decision.

## Modification Notes

Never change a byte, type, length, CRC range, or payload offset in one endpoint
only. Update both transport participants and the App model, then run fragmented,
bad-CRC, length, and golden-vector checks. Do not call a source-only check an
end-to-end acceptance.
