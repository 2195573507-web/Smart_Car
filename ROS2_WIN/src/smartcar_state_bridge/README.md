# smartcar_state_bridge

This package is the host-side, read-only state adapter for P1 mapping. It
does not open a TCP or serial device and it does not publish `/cmd_vel`.
The existing S3 gateway remains the sole network owner.

## Protocol boundary

`TelemetryDecoder` accepts a structured result from an approved SCBP parser or
an explicitly labelled offline fixture. It never guesses a wheel payload layout
from raw bytes. Wheel status is accepted only when the source sample is marked
`valid` and includes `sample_tick` and `sample_seq`; the default rejects live
gateway input because the S3/SCBP contract and captures are not frozen.

The constants for `0x210`, `0x201`, and `0x207` are protocol identifiers only.
This package does not duplicate `Common/SCBP_CAN` definitions or implement a
second CRC/parser.

## SRP v4 chassis-state boundary

`SrpV4ChassisDecoder` independently validates one complete 36-byte SRP v4
frame carried by S3RD outer `message_type=2`. It checks magic, exact length,
logical priority/type/flags, CRC16-CCITT-FALSE, EOF, schema, reserved bytes,
payload flags, `ODOMETRY_VALID`, and all four float values. It has no transport
or `Common/SRP` dependency.

`ChassisOdomTracker` converts the authoritative STM x/y/yaw pose to SI units
and derives only body-frame twist from consecutive valid source timestamps.
It never reintegrates pose. The first frame is baseline-only; invalidity,
timestamp faults, stale input, disconnect, and connection-epoch changes clear
the baseline so recovery cannot reuse pre-fault velocity history.

## Wheel odometry boundary

`WheelKinematics` keeps the firmware order `[RR, RF, LR, LF]`, applies four
independent configured signs (default `[+1,+1,+1,+1]`), converts mm/s to m/s,
and computes differential-drive linear/angular velocity. `WheelOdom` owns a
bounded FIFO and integrates only monotonically fresh samples. Duplicate,
out-of-order, gapped, stale, invalid, backward-time, epoch-change, and FIFO
overflow conditions latch `odom_invalid` until `beginSession()` is called.

The standalone node publishes diagnostics by default. `/odom` and
`odom -> base_link` TF publication are opt-in and disabled in
`config/odom.yaml`; enabling them does not create a transport or authorize an
unreviewed wire contract.

## Verification

Inside the ROS 2 Humble container:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select smartcar_state_bridge
colcon test --packages-select smartcar_state_bridge
colcon test-result --verbose
```

The tests cover SRP framing/CRC/schema, units, quaternion, authoritative pose,
body twist, yaw wrap, timestamp/epoch/disconnect/stale behavior, default
publication gates, plus the existing wheel decoder, kinematics, freshness,
and FIFO behavior. Passing these tests proves host/offline behavior only; it
does not prove a real STM/S3 link, vehicle odometry, or live SLAM.
