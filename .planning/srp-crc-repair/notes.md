# Findings: SRP CRC Diagnostic Repair

- Both CMake targets compile `Common/SRP/srp_crc.c` and `srp_codec.c`.
- `srp_encode()` computes CCITT-FALSE over `out[2]` for `6 + payload_length`
  bytes and writes CRC low byte first; `srp_decode()` performs the same
  calculation and reads CRC low byte first.
- A 16-byte frame is structurally valid for a four-byte payload: 8-byte header,
  4-byte payload, 2-byte CRC, and 2-byte EOF.
- Existing S3 parser logging only reports error number, byte count, REC, and
  TEC, so it cannot distinguish wrong CRC bytes from a changed input range.
- The active sync request is type `0x08`, version `4.0`, payload length 4;
  `RSP_BOOT_INFO` is type `0x09`, version/state payload length 8.
- Protocol encoding is byte-wise. Existing `#pragma pack(push, 4)` views do not
  participate in wire serialization; forcing runtime parser/link state packed
  would risk unaligned ARM accesses.
- S3 diagnostics now print computed CRC, received little-endian CRC, and up to
  64 raw bytes only for a structurally complete CRC error frame.
- Host golden vectors cover a 16-byte SYNC_REQ (`56 BC`) and fast ACK (`38 65`)
  plus parser CRC rejection; both CM7 and ESP-IDF builds compile the shared
  codec after the change.
