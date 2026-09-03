# SRP v4 Chassis-State Wire Contract

## Scope and ownership

S3RD outer `message_type=2` carries exactly one complete SRP v4
chassis-state frame as its payload. The S3RD envelope remains owned and
validated by `s3_ydlidar_bridge`; the 36 payload bytes below are decoded by the
independent in-memory decoder in `smartcar_state_bridge`.

Type 2 is opaque to the S3RD extractor and must never be passed to the
YDLIDAR decoder. The ROS host does not depend on or copy `Common/SRP`.

## SRP frame layout

All multi-byte integers and the four IEEE-754 binary32 values are little
endian.

| Frame offset | Bytes | Field | Required value |
| ---: | ---: | --- | --- |
| 0 | 2 | magic | `AA 55` |
| 2 | 2 | payload length | `24` (`18 00`) |
| 4 | 4 | logical header | Layout below |
| 8 | 24 | chassis payload | Layout below |
| 32 | 2 | CRC16-CCITT-FALSE | Little endian |
| 34 | 2 | EOF | `0D 0A` |

The complete frame is exactly 36 bytes. The logical header is a little-endian
`uint32` with these logical bit fields:

| Bits | Field | Required value |
| ---: | --- | --- |
| 31..24 | priority | `2` |
| 23..16 | type | `0x15` |
| 15..8 | sequence | Producer-owned `uint8` |
| 7..0 | flags | `0` |

## Chassis payload

Offsets below are relative to frame offset 8.

| Payload offset | Bytes | Field | Contract |
| ---: | ---: | --- | --- |
| 0 | 1 | schema | Must be `1` |
| 1 | 1 | flags | Only mask `0x0F` is allowed |
| 2 | 2 | reserved | Must be zero |
| 4 | 4 | timestamp_ms | Source monotonic `uint32` milliseconds |
| 8 | 4 | x_mm | Finite IEEE-754 binary32 |
| 12 | 4 | y_mm | Finite IEEE-754 binary32 |
| 16 | 4 | yaw_deg | Finite IEEE-754 binary32 |
| 20 | 4 | total_dist_m | Finite IEEE-754 binary32 |

Payload flag `ODOMETRY_VALID=0x04` must be set. A frame with that bit clear is
invalid and cannot produce odometry or TF. Unknown payload flag bits, non-zero
reserved bytes, and any NaN or infinity are rejected.

## CRC

The CRC is CRC16-CCITT-FALSE:

- Initial value: `0xFFFF`
- Polynomial: `0x1021`
- No input or output reflection
- No final XOR
- Covered bytes: the 30 bytes beginning at frame offset 2, namely the
  2-byte length, 4-byte logical header, and 24-byte payload
- Stored order: low byte, then high byte

Magic, the stored CRC field, and EOF are not in the CRC input.

## ROS conversion and freshness

- `x_mm` and `y_mm` are divided by 1000 for metres.
- `yaw_deg` is multiplied by pi/180 for radians and converted to a unit yaw
  quaternion.
- STM pose is authoritative. ROS does not integrate pose again.
- Consecutive valid positions are differenced and rotated from `odom` into the
  previous valid `base_link` orientation to produce body-frame linear twist.
- Angular twist uses the shortest yaw difference, including `179 -> -179`.
- The first valid frame after start, invalidity, stale state, disconnect, or a
  connection-epoch change only establishes a baseline and is not published.
- `timestamp_ms` is used only for source `dt` and freshness checks. Duplicate,
  rollback, stale, and unreasonable intervals are rejected.
- ROS `header.stamp` is the host ROS clock at receive handling. The same
  already-stamped odometry header is copied to optional `odom -> base_link` TF.

## Golden vector

The fixed vector uses sequence `0x2A`, `timestamp_ms=1000`, `x_mm=1000.0`,
`y_mm=-500.0`, `yaw_deg=179.0`, and `total_dist_m=12.5`.

```text
AA 55 18 00 00 2A 15 02 01 04 00 00 E8 03 00 00
00 00 7A 44 00 00 FA C3 00 00 33 43 00 00 48 41
7F C0 0D 0A
```

CRC input (30 bytes, beginning at the length field):

```text
18 00 00 2A 15 02 01 04 00 00 E8 03 00 00 00 00
7A 44 00 00 FA C3 00 00 33 43 00 00 48 41
```

Expected CRC value: `0xC07F`. Stored CRC bytes: `7F C0`.

Expected decoded SI values:

```text
priority=2 type=0x15 sequence=0x2A header_flags=0
schema=1 chassis_flags=0x04 timestamp_ms=1000
x_m=1.0 y_m=-0.5 yaw_rad=3.12413936106985 total_dist_m=12.5
```

This vector is an offline protocol fixture, not a live STM32/S3 capture or
evidence of vehicle odometry, TF, SLAM, or mapping readiness.
