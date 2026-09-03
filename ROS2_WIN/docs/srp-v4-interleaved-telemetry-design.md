# SRP v4 Interleaved Telemetry Design

## Scope

This host-only change corrects SRP v4 type-2 routing and sequence admission in
`ROS2_WIN/`. It does not change S3 or STM firmware, protocol bytes, vehicle
control, RViz/SLAM workflows, or any false-by-default safety parameter.

## Audit result

The live sender uses one outer S3RD sequence for the complete TCP stream. A
120-frame capture was exactly contiguous from `30934` through `31053`, even
when type 1 appeared between type-2 frames. Type-2 is a multiplexed SRP stream,
not a chassis-only stream: the capture contained `0x10`, `0x11`, `0x14`, and
`0x15`. Chassis inner sequence therefore advances across other SRP messages
and cannot be required to increment by one between chassis samples.

## Decoder and dispatch

1. A public, transport-independent SRPv4 decoder validates the complete common
   envelope: magic, declared payload length versus exact frame size, CRC16-
   CCITT-FALSE, EOF, and logical-header extraction.
2. Every outer type-2 payload passes through that decoder before routing.
3. Valid `0x10` IMU, `0x14` wheel, and other non-chassis messages are counted
   and ignored by chassis odometry. They do not increment chassis rejects or
   clear the chassis baseline.
4. Only a common-decoded `message_id=0x15` enters the chassis decoder and odom
   admission path.
5. The 0x15 decoder retains exact 36-byte/24-byte-payload requirements and its
   priority, header flags, schema, payload flags, reserved, odometry-valid,
   finite-value, CRC, and EOF checks. No length rule is relaxed.
6. A malformed common SRP frame is rejected. If its available header identifies
   0x15, it is also a chassis decode reject; rejection invalidates the chassis
   baseline. Unknown malformed type-2 input is conservatively invalidating but
   is not mislabeled as a valid non-chassis message.

## Sequence and lifecycle

- One `SequenceTracker` is owned by each TCP connection and observes every
  accepted S3RD frame regardless of message type. Device and stream identity
  remain enforced by the outer extractor, so no type-specific sequence map is
  needed.
- Actual outer duplicate, backward, or gap classifications are preserved and
  are never hidden in ROS.
- Chassis inner sequence uses uint8 modulo ordering. Equal values are
  duplicates; a half-range backward delta is rollback; any unambiguous forward
  delta is permitted. Forward gaps are evaluated by source timestamp, host
  freshness, and configured dt/stale limits.
- Disconnect/reconnect and epoch changes reset the session and baseline.
  Chassis decode, sequence, timestamp, dt, host-time, and stale failures clear
  the velocity baseline. Valid non-chassis telemetry does not touch it.

## Tests and live gates

Focused tests cover IMU/wheel/chassis interleaving, forward non-contiguous
chassis sequence with anchor then accepted odometry, non-chassis counter and
baseline isolation, strict wrong-size 0x15 rejection, duplicate/rollback/time
rollback/stale/reconnect, and global outer continuity across type 1/type 2.

After a full container build/test pass, live validation proceeds in two hard-
gated stages: 30 seconds with all four parameters false, then only if real
type-2 and valid 0x15 counters grow without sequence/decode faults, a temporary
120-second all-true `/odom` and `odom -> base_link` observation while the
vehicle remains stationary. A finally-style cleanup restores and verifies all
four parameters false even on timeout or failure. No replay, synthetic live
input, RViz, SLAM, or mapping claim is allowed.
