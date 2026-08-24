# SRP CRC Diagnostic and Handshake Regression Design

## Scope

Repair the STM32H757 to ESP32-S3 SRP handshake observability without changing
the active SRP v4 wire contract. Both firmware targets continue to consume the
single implementation in `Common/SRP`.

## Confirmed Contract

- Frame: `AA 55 | LEN_LE | HEADER_LE | PAYLOAD | CRC16_LE | 0D 0A`.
- CRC: CRC-16/CCITT-FALSE, polynomial `0x1021`, initial value `0xFFFF`, no
  reflection, `xorout=0`.
- CRC coverage: the six bytes beginning at `LEN` through the final payload
  byte. Magic, CRC, and EOF are excluded.
- CRC and all scalar wire fields are little-endian.
- `CMD_SYNC_REQ` is type `0x08`, payload `{4, 0, 0, 0}`; STM32 replies with
  `RSP_BOOT_INFO` type `0x09` and an eight-byte payload. A four-byte fast
  response therefore produces a 16-byte frame.

## Changes

1. Add bounded ESP32-S3 CRC diagnostics to the existing parser error callback.
   On parser error 4, compute the CRC from the received raw frame using the
   canonical coverage, decode the received CRC as little-endian, and print
   both values plus the raw bytes. The parser still rejects the frame.
2. Add explicit compile-time wire-layout assertions and packed protocol view
   annotations only where they do not create unaligned runtime state. Runtime
   parser/link structs remain naturally aligned because they are not serialized.
3. Extend the host codec test with deterministic 16-byte sync/fast-response
   frames, CRC byte-order assertions, and parser callback coverage.

## Error Handling

Diagnostics are bounded to the parser callback and do not accept alternate CRC
algorithms, magic values, or byte orderings. A bad CRC continues to increment
REC/parser error accounting and is discarded. A frame can only reach the sync
state machine after normal decode succeeds.

## Verification

- Run the standalone C11 SRP codec test with warnings as errors.
- Run `git diff --check`.
- Build ESP-IDF in an isolated build directory and build the STM32 CM7 Debug
  target after the source change.
- Treat a runtime `SRP sync state=ESTABLISHED/READY` line as hardware evidence
  only when matching images are flashed and the UART capture is available.

## Risks and Boundaries

The change adds logging work to the S3 service task, so the raw-frame string is
bounded and only built for CRC failures. No UART pins, baud rate, DMA owner,
BLE UUID, safety behavior, or message IDs are changed. Source/build success
does not prove physical UART integrity or flashed-image provenance.
