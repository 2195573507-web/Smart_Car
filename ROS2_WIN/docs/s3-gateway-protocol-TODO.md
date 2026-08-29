# S3 gateway protocol status

The TCP receiver implements the task-provided experimental candidate envelope;
it is not a frozen or released protocol.

Candidate layout (little-endian fields): `S3RD` magic (4), version (1),
message type (1, `RAW_YDLIDAR_FRAME=1`), flags (2), device ID (4), stream ID
(4), sequence (4), timestamp_ms (4), payload length (2), payload, CRC16-Modbus
(2). The fixed header before payload is 26 bytes. CRC covers version through
payload, excluding magic and the final CRC. Payload is a complete original
AA 55 YDLIDAR frame.

The implementation accepts flags `0x0000` and `0x0001` by default, rejects
unknown bits, and requires flags bit 0 to match the payload CT bit 0. It also
rejects unsupported version/type, configured identity mismatches, invalid
payload bounds, and CRC errors. A zero-position packet starts a revolution and
the next zero-position packet emits the preceding complete scan; it is never
used to fabricate a partial scan. No
`ESPS3/main/radar/radar_uplink_protocol.c` exists in this repository, so
compatibility with a current S3 firmware implementation is unverified and the
candidate remains experimental.

The project materials inspected on 2026-08-28 do not freeze the S3 Wi-Fi
envelope. `ESPS3/docs/S3_YDLIDAR_X3PRO_TEST.md` stops at UART1/GPIO44 raw-byte
capture and explicitly leaves networking and point decoding unimplemented;
`DOCS/architecture/data_flow.md` marks the radar-to-ROS path as future; and
`DOCS/ESP32/S3_ARCHITECTURE.md` says the ROS2 bridge contract remains future
work. In particular, the following values are unknown and must not be
guessed:

- transport and connection direction (LAN TCP, relay, or other);
- magic, protocol version, message type, and device/stream identity;
- payload length field width and byte order;
- sequence-number reset, duplicate, and wrap rules;
- outer CRC/authentication and failure behavior;
- timestamp semantics and maximum accepted age;
- whether a payload contains exactly one YDLIDAR packet or a bounded batch.

The existing `AA 55` SCBP frame documented for STM32 <-> S3 control/status is
not treated as a radar Wi-Fi envelope. Shared magic bytes do not establish a
shared payload layout or checksum contract.

The bridge consequently refuses live mode and exposes only interfaces for a
future approved extractor. The minimum handoff needed to enable live data is:

1. one written S3 envelope specification with a maximum frame size;
2. at least one captured valid frame and captures for bad length/checksum and a
   sequence jump;
3. confirmation that the payload is the complete original YDLIDAR byte frame;
4. an agreed frame id, timestamp policy, and stale timeout;
5. an end-to-end capture showing S3 UART1/GPIO44 to Wi-Fi to the bridge.

Until then, no live `/scan` or vehicle-chain completion claim is valid.
