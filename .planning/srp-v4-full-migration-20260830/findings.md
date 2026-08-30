# Findings

## Confirmed

- `Common/SRP` is present and contains codec, wire, CRC, link, registry, and
  host tests.
- STM32 CM7 and ESP32-S3 CMake files now compile `Common/SRP`; the tracked
  `Common/SCBP_CAN` implementation is staged for deletion.
- Current residual SCBP references are concentrated in historical/audit/design
  documents plus the App BLE boundary statement; active source references
  must be checked after builds.
- App BLE, YDLIDAR, SmartCarLog, and S3RD are distinct envelopes and are not
  candidates for deletion as STM-S3 UART legacy protocol.
- `Common/SRP` is identical to the SRPv4 implementation on `origin/3`.
- The only remaining old-protocol matches outside ignored build output are
  historical audit/design reports, explicit deprecation notes, and the App
  BLE model name `SmartCarProtocol`; none are active STM-S3 UART sources.
- Ignored build directories from earlier tasks still contain stale object-path
  names such as `Common/SCBP_CAN`. They are not part of the current build graph
  and were not deleted because they are outside this cleanup's source scope.

## Verification limits

- A host or firmware build proves source integration only; it does not prove
  UART wiring, flashed-image matching, BLE delivery, or vehicle behavior.

## Device log follow-up

- The pasted S3 log contains valid SRPv4 frames (`AA 55`, little-endian length,
  type `0x12`/`0x11`, CCITT frame CRC, `0D 0A`), so it is not evidence of the
  old SCBP-CAN wire format.
- The crash occurs during the experimental radar uplink Wi-Fi retry path. The
  old hook hid every non-service task as `<unknown>` and reported a stale
  service watermark, so it could not identify the overflowing task.
- The current minimal mitigation increases `radar_uplink_task` from 4096 to
  6144 bytes, logs its watermark, avoids `esp_wifi_set_config()` while a
  connection is in progress, and reports raw hook task identity. Reflash and
  capture the first 5 seconds before deciding whether more stack reduction or
  a task-specific fix is needed.

## Final Cleanup Evidence

- Active source scan (`*.c`, `*.h`, `*.cpp`, `*.hpp`, `*.swift`, and CMake)
  returns no `SCBP`, `scbp_*`, `sc_frame`, `SmartCar_Frame`, or `SC_TYPE_*`
  references outside the independent App BLE vocabulary.
- Canonical `STM32H757/CM7/build/Debug` and `ESPS3/build` compile databases and
  Ninja graphs contain no legacy protocol path; generated stale-object count is
  zero after the clean rebuild.
- Remaining textual matches are confined to protected memory, historical
  planning/audit/migration documents, and archived experiment material. They
  are not active build inputs and are labeled or path-scoped as historical.
- Current build artifact hashes: `ESPS3/build/smartcar_s3_gateway.bin`
  `47d4f8b24fa35806ed71619ca3f90376ac3c53109106cfa52d637e218dcc1145`;
  `STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf`
  `c5e7016d861a0f65aa27cc2fe17906c8369696198c29b1317de8f3e0aa9ee8b0`.
- No serial device is currently enumerated beyond `/dev/cu.debug-console` and
  Bluetooth; no S3 flash or runtime capture was performed in this pass.
