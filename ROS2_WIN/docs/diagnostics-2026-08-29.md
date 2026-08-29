# S3 -> Windows ROS2 runtime verification

Date: 2026-08-29  
Workspace: `D:\Smart_Car\ROS2_WIN`  
Target: `192.168.31.101:8765`

This was a read-only runtime check. The bridge container was left running. No
bridge source, S3 project, S3RD field, or credential was changed or recorded.

## Container discovery and host port

The requested running-container selection returned:

```text
container=docker-ros2-dev-run-e3c73fadb323 id=68e26698d316
```

Running containers:

```text
NAMES                              STATUS        PORTS
docker-ros2-dev-run-e3c73fadb323   Up 3 hours    0.0.0.0:8765->8765/tcp, [::]:8765->8765/tcp
docker-ros2-dev-run-b03afd846a9c   Up 10 hours
```

`docker logs --since 2m 68e26698d316` produced no output.

The bridge container is still running:

```text
name=/docker-ros2-dev-run-e3c73fadb323 status=running running=true container_pid=7664
cmd=["ros2","run","s3_ydlidar_bridge","s3_ydlidar_bridge_node","--ros-args","-p","transport:=tcp","-p","tcp_listen_port:=8765"]
```

Inside the container, PID 1 and the bridge process are:

```text
  PID  PPID STAT CMD
    1     0 Ss   /usr/bin/python3 /opt/ros/humble/bin/ros2 run s3_ydlidar_bridge s3_ydlidar_bridge_node --ros-args -p transport:=tcp -p tcp_listen_port:=8765
   22     1 Sl   /ws/install/s3_ydlidar_bridge/lib/s3_ydlidar_bridge/s3_ydlidar_bridge_node --ros-args -p transport:=tcp -p tcp_listen_port:=8765
```

Windows `Get-NetTCPConnection -LocalPort 8765`:

```text
      State LocalAddress   LocalPort RemoteAddress  RemotePort OwningProcess
     Listen ::1                 8765 ::                      0         26272
     Listen ::                  8765 ::                      0         37144
Established 192.168.31.101      8765 192.168.31.239      64851         37144
```

Therefore Windows port 8765 has an `ESTABLISHED` connection from
`192.168.31.239:64851` to `192.168.31.101:8765`.

## ROS 2 topic checks

The literal requested `docker exec ... bash -lc "ros2 ..."` commands returned
`ros2: command not found` because a direct non-entrypoint shell does not source
the ROS environment. The equivalent read-only checks were then run through the
container entrypoint. `command -v ros2` returned:

```text
/opt/ros/humble/bin/ros2
```

`ros2 topic list -t` returned:

```text
/clicked_point [geometry_msgs/msg/PointStamped]
/diagnostics [diagnostic_msgs/msg/DiagnosticArray]
/goal_pose [geometry_msgs/msg/PoseStamped]
/initialpose [geometry_msgs/msg/PoseWithCovarianceStamped]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/scan [sensor_msgs/msg/LaserScan]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]
```

`/scan` has type `sensor_msgs/msg/LaserScan`, one publisher named
`s3_ydlidar_bridge`, BEST_EFFORT reliability, VOLATILE durability, and zero
subscriptions. `/diagnostics` has type `diagnostic_msgs/msg/DiagnosticArray`,
one publisher named `s3_ydlidar_bridge`, RELIABLE reliability, VOLATILE
durability, and zero subscriptions.

The diagnostics one-shot message reported:

```text
message: stale
tcp_connection_state: connected
accepted_connections: 8
disconnects: 7
recv_bytes: 1526410
received_packets: 0
magic_errors: 0
crc_errors: 0
length_errors: 0
version_errors: 0
message_type_errors: 0
flags_errors: 835
identity_errors: 14103
ydlidar_checksum_errors: 0
duplicate_sequences: 0
out_of_order_sequences: 0
sequence_jumps: 0
sequence_wraps: 0
published_scans: 0
recent_valid_packet_time_ns: never
stale: true
published_scan_frequency_hz: 0.000000
```

The requested `/scan --once` check produced no message and exited after its
10-second timeout. The requested `/scan topic hz --window 10` check produced
no sample output and exited after its 20-second timeout; no frequency could be
calculated.

Both `/scan` and `/diagnostics` exist, so no substitute topic was needed. The
topic list above is the actual related ROS 2 topic state.

## Conclusion

未收到 Windows ROS2 数据。

Diagnostics were received and the TCP peer stayed connected, but no valid
packet was accepted and no `LaserScan` was published during this check. This
result does not claim vehicle operation or formal protocol acceptance.

## Protocol field audit (2026-08-29)

All checks in this section were read-only. The running bridge container was not
stopped, restarted, or recreated. No bridge source, S3 project, S3RD field, or
credential was modified or recorded.

### Runtime evidence

Container selection returned `container=docker-ros2-dev-run-e3c73fadb323`
with ID `68e26698d316`. Its entrypoint is `/entrypoint.sh` and its command is
`ros2 run s3_ydlidar_bridge s3_ydlidar_bridge_node --ros-args -p transport:=tcp
-p tcp_listen_port:=8765`. It remained `Up 3 hours` with
`0.0.0.0:8765->8765/tcp` and `[::]:8765->8765/tcp`.

`docker logs --since 5m` produced no output. The ROS graph contains
`/s3_ydlidar_bridge`; related topics are `/diagnostics
[diagnostic_msgs/msg/DiagnosticArray]` and `/scan
[sensor_msgs/msg/LaserScan]`.

The requested parameter dump path `/s3_ydlidar_bridge_node` returned `Node not
found`; the actual node `/s3_ydlidar_bridge` returned:

```text
transport: tcp
tcp_listen_address: 0.0.0.0
tcp_listen_port: 8765
s3_expected_version: 1
s3_expected_message_type: 1
s3_expected_flags: 0
s3_expected_device_id: 0
s3_expected_stream_id: 0
s3_min_payload_bytes: 10
s3_max_payload_bytes: 65535
```

The final diagnostics one-shot reported:

```text
tcp_connection_state: listening
accepted_connections: 8
disconnects: 8
recv_bytes: 1980498
received_packets: 0
magic_errors: 0
crc_errors: 0
length_errors: 0
version_errors: 0
message_type_errors: 0
flags_errors: 1059
identity_errors: 18217
ydlidar_checksum_errors: 0
duplicate_sequences: 0
out_of_order_sequences: 0
sequence_jumps: 0
sequence_wraps: 0
published_scans: 0
recent_valid_packet_time_ns: never
stale: true
published_scan_frequency_hz: 0.000000
```

At the final Windows TCP sample, port 8765 had listeners on `::1` and `::`; no
ESTABLISHED row was present at that exact sample. The previously observed S3
peer was `192.168.31.239`.

### Bridge versus supplied S3 candidate

| Field | Supplied S3 candidate | Bridge implementation/runtime | Comparison |
| --- | --- | --- | --- |
| magic | ASCII `S3RD` | Four-byte `S3RD` check | match |
| version | `1` | expected `1` | match |
| message_type | `1` | expected `1` | match |
| flags | little-endian; allowed bit `0x0001` | little-endian read; exact equality to expected `0x0000` | mismatch if S3 sends `0x0001`; not a bitmask |
| device_id | `1` | expected `0` | mismatch |
| stream_id | `1` | expected `0` | mismatch |
| sequence | little-endian `uint32` | offset 16, little-endian `uint32` | match |
| timestamp_ms | little-endian `uint32` | offset 20, little-endian `uint32` | match |
| header size | 26 bytes | fixed `kHeaderBytes=26` | match |
| payload length | little-endian `uint16` | offset 24, little-endian `uint16` | match |
| payload | complete YDLIDAR-validated `AA 55` frame | passed after header to official decoder; no valid packet accepted | not runtime-verified |
| CRC | CRC16-Modbus little-endian; version through payload | offsets 4 through `26 + payload_length - 1`, excluding magic and final CRC | match |

In `src/s3_ydlidar_bridge/src/framing.cpp`, `readU16/readU32` implement the
little-endian decoding. Fields are read at offsets 4/5/6/8/12/16/20/24. The
CRC call starts at offset 4 and ends before the trailing CRC, so it excludes
the four-byte magic and the final two-byte CRC.

### Direct causes and change direction

`flags_errors=1059` is most directly explained by the exact
`flags == expected_flags` check with runtime `expected_flags=0`; a wire value
`0x0001` is rejected even though the supplied S3 candidate names that bit as
allowed. `identity_errors=18217` is most directly explained by runtime
`expected_device_id=0` and `expected_stream_id=0` versus supplied S3 values
`1` and `1`.

If the supplied S3 candidate is the intended current wire configuration, the
change target would be the Windows runtime/configuration parameters, not bridge
source, after confirming whether `flags` is a fixed value `0x0001` or a mask;
no change was executed. No S3 configuration change is justified by this
read-only evidence alone.

Final counters: `received_packets=0`, `published_scans=0`.

## Minimal runtime parameter correction (2026-08-29)

This was a startup-parameter-only correction. The bridge source, S3 project,
S3RD fields, and STM32 were not modified. The previous bridge container
`68e26698d316` was stopped as explicitly requested; no S3-side restart or
configuration action was performed.

The source/config search found `s3_expected_device_id`,
`s3_expected_stream_id`, and `s3_expected_flags`. No `allowed_flags_mask`
parameter or implementation exists. The implementation compares flags by exact
equality.

The new foreground command was:

```text
docker compose -f .\docker\compose.yaml run --rm --service-ports ros2-dev ros2 run s3_ydlidar_bridge s3_ydlidar_bridge_node --ros-args -p transport:=tcp -p tcp_listen_port:=8765 -p s3_expected_device_id:=1 -p s3_expected_stream_id:=1 -p s3_expected_flags:=0
```

New container:

```text
container=docker-ros2-dev-run-422c01f47459 id=e07f0e286f81
status=running running=true container_pid=9941
```

The actual `/s3_ydlidar_bridge` parameter dump confirmed:

```text
transport: tcp
tcp_listen_address: 0.0.0.0
tcp_listen_port: 8765
s3_expected_version: 1
s3_expected_message_type: 1
s3_expected_flags: 0
s3_expected_device_id: 1
s3_expected_stream_id: 1
```

Therefore flags remain exact `0x0000`; there is no configured mask
`0x0001`. Ordinary S3 points with `flags=0x0000` can match this configuration,
while zero-position packets with `flags=0x0001` remain rejected by the
existing exact comparison.

After restart, `Get-NetTCPConnection -LocalPort 8765` showed listeners on
`::1` and `::`, with no `ESTABLISHED` connection at the verification sample.
The bridge's diagnostics one-shot reported `accepted_connections=0`,
`recv_bytes=0`, `received_packets=0`, `flags_errors=0`,
`identity_errors=0`, `published_scans=0`, `stale=true`, and
`published_scan_frequency_hz=0.000000`.

The requested `/scan --once` check received no message within 10 seconds. The
requested 20-second `/scan topic hz --window 10` check produced no samples and
no frequency result. The bridge remained listening, but the S3 peer did not
reconnect during this verification, so acceptance of an ordinary point frame
could not be observed.

## RViz2 radar display configuration (2026-08-29)

The RViz2 configuration was added at:

```text
Host:    D:\Smart_Car\ROS2_WIN\config\rviz\radar_scan.rviz
Container: /ws/config/rviz/radar_scan.rviz
```

The file sets `Fixed Frame=rviz_world`, `TopDownOrtho`, `Scale=10`, keeps a
Grid display, and adds a high-contrast yellow `LaserScan` display for `/scan`.
The scan uses `Style=Points`, `Size (m)=0.05`, `Alpha=1.0`,
`Reliability Policy=Best Effort`, and `Durability Policy=Volatile`.

The display-only helper is:

```text
D:\Smart_Car\ROS2_WIN\docker\verify_radar_rviz2.ps1
```

With the existing `docker-ros2-dev-run-b03afd846a9c` container running, use
two terminals in this order:

```powershell
Set-Location 'D:\Smart_Car\ROS2_WIN'
.\docker\verify_radar_rviz2.ps1 -Mode StaticTf
```

This runs the requested temporary transform command in the container:

```text
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 rviz_world laser_frame
```

In a second terminal, start RViz2 with:

```powershell
Set-Location 'D:\Smart_Car\ROS2_WIN'
.\docker\verify_radar_rviz2.ps1 -Mode Rviz
```

The helper resolves the running `ros2-dev` container, starts/reuses VcXsrv at
`host.docker.internal:0.0`, and executes:

```text
docker exec --interactive -e DISPLAY=host.docker.internal:0.0 <ros2-dev-container> /entrypoint.sh bash -lc "rviz2 -d /ws/config/rviz/radar_scan.rviz"
```

Verification results:

- `ros2 topic info /scan -v`: `sensor_msgs/msg/LaserScan`, one publisher
  (`s3_ydlidar_bridge`), `BEST_EFFORT`, `VOLATILE`.
- `timeout 10s ros2 topic echo /scan --once`: no sample received.
- `/diagnostics`: bridge `listening`, `stale=true`, `published_scans=0`.
- RViz2 read the new configuration successfully and initialized OpenGL 4.5;
  no configuration or display-plugin error was reported during the 5-second
  launch check.

Screenshot note: no scan-point screenshot can be confirmed for this run because
the live `/scan` topic produced no message. Once a real `/scan` sample arrives,
the expected view is a dark `TopDownOrtho` canvas with a gray XY grid and
yellow points in the `laser_frame` plane. Therefore `/scan` is **not currently
visualized**; the RViz display is configured and ready but has no data to draw.

The `rviz_world -> laser_frame` transform above is temporary and exists only
for RViz display verification; it is not the formal robot TF. No bridge source,
S3/S3RD protocol, or STM32 files were modified.

## Experimental S3RD flags correction and runtime verification (2026-08-29)

Scope: ROS2_WIN bridge source/tests/configuration documentation only. S3,
STM32, TCP port, device/stream identity values, CRC16-Modbus, and the 26-byte
little-endian header were not changed. `frame_id` remains `laser_frame`.

Minimal source correction: `last_valid_packet_ns_` is now updated for a
protocol-valid zero-position packet before incrementing `zero_packets`, and
for an ordinary packet only after the official YDLIDAR decoder succeeds. A
decoder failure therefore does not clear the stale state. The framing rules
accept flags `0x0000` and `0x0001` through `s3_allowed_flags_mask=0x0001`,
reject unknown bits, and require flags bit 0 to equal payload CT bit 0.
Zero-position packets are counted but do not publish a fabricated LaserScan.

Build/test evidence:

```text
docker exec s3-ydlidar-bridge ... colcon build --symlink-install --packages-select s3_ydlidar_bridge
Summary: 1 package finished
docker exec s3-ydlidar-bridge ... colcon test --packages-select s3_ydlidar_bridge
Summary: 1 package finished
docker exec s3-ydlidar-bridge ... colcon test-result --verbose
Summary: 28 tests, 0 errors, 0 failures, 0 skipped
docker exec s3-ydlidar-bridge ... colcon build --symlink-install
Summary: 1 package finished
docker exec s3-ydlidar-bridge ... colcon test
Summary: 1 package finished
docker exec s3-ydlidar-bridge ... colcon test-result --verbose
Summary: 28 tests, 0 errors, 0 failures, 0 skipped
```

The bridge was started with the verified runtime command:

```text
docker compose -f .\docker\compose.yaml run -d --rm --service-ports --name s3-ydlidar-bridge ros2-dev ros2 run s3_ydlidar_bridge s3_ydlidar_bridge_node --ros-args -p transport:=tcp -p tcp_listen_port:=8765 -p s3_expected_device_id:=1 -p s3_expected_stream_id:=1 -p s3_allowed_flags_mask:=1
```

Runtime container:

```text
container=s3-ydlidar-bridge id=6d0b37634036 status=Up
ports=0.0.0.0:8765->8765/tcp, [::]:8765->8765/tcp
pid1=ros2
pid1 command=/usr/bin/python3 /opt/ros/humble/bin/ros2 run s3_ydlidar_bridge s3_ydlidar_bridge_node --ros-args -p transport:=tcp -p tcp_listen_port:=8765 -p s3_expected_device_id:=1 -p s3_expected_stream_id:=1 -p s3_allowed_flags_mask:=1
```

The 125-second diagnostics subscription reported `tcp_connection_state=listening`
and `accepted_connections=0` throughout. The final sample was:

```text
received_packets=0
flags_errors=0
identity_errors=0
crc_errors=0
ydlidar_checksum_errors=0
zero_packets=0
published_scans=0
published_scan_frequency_hz=0.000000
stale=true
```

Windows `Get-NetTCPConnection -LocalPort 8765` showed only listeners on
`::1:8765` and `:::8765`; there was no `ESTABLISHED` connection. Container
logs during the final five-minute window were empty (no S3 client connected).
`ros2 topic echo /scan --once` timed out after 10 seconds with no message, and
`ros2 topic hz /scan --window 10` timed out after 20 seconds with no samples or
frequency result. This is runtime evidence of no received Windows ROS2 data,
not a vehicle or formal protocol acceptance claim.

## RViz2 QoS correction and live verification (2026-08-29)

This section records the requested RViz2-only adjustment. No S3 project,
bridge source, protocol field, or TCP implementation was changed.

### Current publisher and RViz configuration

The running `ros2-dev` container selected by the requested command was
`docker-ros2-dev-run-b03afd846a9c` (container ID `20a0f84872d3`). The live
`ros2 topic info /scan -v` output reports:

```text
Publisher Reliability Policy: BEST_EFFORT
Publisher Durability Policy: VOLATILE
Publisher: s3_ydlidar_bridge
```

`config/rviz/radar_scan.rviz` contains the requested LaserScan settings:

```text
Topic Value: /scan
RViz Reliability Policy: Best Effort
Durability Policy: Volatile
Size (m): 0.05
Style: Points
```

The target values were already present when inspected, so no unrelated RViz
or application settings were changed.

### Live diagnostics and TCP state

The requested one-shot `/diagnostics` sample reported:

```text
tcp_connection_state: connected
stale: false
published_scans: 8671
published_scan_frequency_hz: 14.999387
```

At the same host-side sample, port `8765` still had an `ESTABLISHED` TCP
connection:

```text
192.168.31.101:8765 -> 192.168.31.239:49997
```

### RViz2 restart requirement

If RViz2 is already running with an older copy of this configuration, restart
RViz2 (or reload `radar_scan.rviz`) for the QoS setting to take effect. The
bridge/container and its TCP connection do not need to be restarted for this
RViz-only change. If RViz2 is not running, launch it normally with the updated
configuration.

## Read-only bridge radar ingress check (2026-08-29 14:31:31 +08:00)

This was a read-only runtime check. No bridge, RViz2, Static TF, S3, source,
configuration, or protocol process/file was stopped, restarted, or modified.
The only file operation was appending this report section.

### Container and ROS graph

The selected running container was:

```text
name=s3-ydlidar-bridge
id=6d0b37634036
status=Up 2 hours
ports=0.0.0.0:8765->8765/tcp, [::]:8765->8765/tcp
```

Its actual bridge process remained:

```text
/usr/bin/python3 /opt/ros/humble/bin/ros2 run s3_ydlidar_bridge s3_ydlidar_bridge_node --ros-args -p transport:=tcp -p tcp_listen_port:=8765 -p s3_expected_device_id:=1 -p s3_expected_stream_id:=1 -p s3_allowed_flags_mask:=1
```

The actual ROS node names were `/s3_ydlidar_bridge`, `/rviz`, and
`/transform_listener_impl_5af13f1cc470`. The bridge node name is
`/s3_ydlidar_bridge` (not `/s3_ydlidar_bridge_node`).

### TCP and diagnostics

The host had an `ESTABLISHED` connection on port `8765`:

```text
192.168.31.101:8765 -> 192.168.31.239:49726
```

The remote IP `192.168.31.239` matches the S3 peer recorded by the prior
runtime checks, so this connection is treated as the S3 TCP connection. An IP
address alone is not device authentication.

The latest diagnostics sample reported:

```text
tcp_connection_state: connected
accepted_connections: 2
disconnects: 1
recv_bytes: 7777058
received_packets: 73294
flags_errors: 0
identity_errors: 0
crc_errors: 0
published_scans: 54315
stale: true
published_scan_frequency_hz: 0.000000
```

During the short A/B recheck, `received_packets` increased from `73166` to
`73294` and `recv_bytes` increased from `7763716` to `7777058`, while
`published_scans` stayed at `54315`. `out_of_order_sequences` increased from
`15799` to `15927`; the flags, identity, and CRC error counters remained zero.
This confirms bytes and accepted protocol frames are still entering the
bridge, but no new scan has been published. The condition "ESTABLISHED but
received_packets does not grow" did not occur in this check, so the requested
protocol-parse diagnosis for that condition does not apply. If a future check
shows an `ESTABLISHED` connection with a flat `received_packets` counter, that
would indicate a protocol parsing/acceptance path problem per the diagnostic
rule.

### `/scan` result

`/scan` had one BEST_EFFORT publisher (`/s3_ydlidar_bridge`) and one BEST_EFFORT
RViz subscription with VOLATILE durability. The requested
`ros2 topic echo /scan --once` timed out after 15 seconds with no new message.
The requested `ros2 topic hz /scan --window 10` timed out after 30 seconds with
no samples and no frequency result. Therefore `/scan` did not receive a new
message during this check, and no live frequency could be calculated.

The bridge is receiving traffic, but its current published LaserScan stream is
stale (`published_scans` unchanged and `stale=true`). The recent bridge log
window was empty.

## Read-only ingress and publication classification (2026-08-29 14:41:29 +08:00)

This was a read-only diagnostic run. No S3, bridge, protocol, TCP, RViz
configuration, Static TF, source, or runtime process was modified, stopped, or
restarted. The only file operation was appending this section.

### Diagnostics snapshots

The selected container remained `s3-ydlidar-bridge` (`6d0b37634036`). Snapshot
1 and snapshot 2 were separated by the requested 15 seconds:

| Field | Snapshot 1 | Snapshot 2 | Increment |
| --- | ---: | ---: | ---: |
| `recv_bytes` | 8796656 | 8828660 | +32004 |
| `received_packets` | 82828 | 83134 | **+306** |
| `published_scans` | 54315 | 54315 | **+0** |
| `zero_packets` | 3024 | 3024 | +0 |
| `out_of_order_sequences` | 25461 | 25767 | +306 |

Both snapshots reported `tcp_connection_state=connected`,
`accepted_connections=2`, `disconnects=1`, `flags_errors=0`,
`identity_errors=0`, `crc_errors=0`, and `stale=true`. The latest follow-up
sample was `recv_bytes=9128364`, `received_packets=85933`,
`published_scans=54315`, `zero_packets=3024`,
`out_of_order_sequences=28566`, and `stale=true`.

### Requested last-value fields

The diagnostics API does not expose fields named `last_valid_scan_age`,
`last_sequence`, or `last_flags`. It exposes
`recent_valid_packet_time_ns=55192471584056`; using the read-only container
monotonic clock sample `56834536407828`, its packet age was approximately
`1642.065` seconds (27 minutes 22.1 seconds). This is a last-valid-packet age,
not a scan-specific field. The available sequence evidence is
`out_of_order_sequences=28566`, `sequence_jumps=57343`,
`duplicate_sequences=0`, and `sequence_wraps=0`. No last-frame flags value is
retained in diagnostics.

### Receive thread and `/scan`

`docker top` showed the unchanged bridge process (PID 22 inside the container),
and `ps -T -p 22` showed the process plus its receive/server thread set (TIDs
22-33). The increases in `recv_bytes` and `received_packets` across the two
snapshots confirm that the TCP receive path was active during this check.

`/scan` still had one publisher (`/s3_ydlidar_bridge`) and one RViz
subscription, both BEST_EFFORT with VOLATILE durability. The explicit QoS CLI
form in the requested command is unsupported by this ROS 2 CLI and returned
`unrecognized arguments`; the supported fallback
`ros2 topic echo /scan --once` timed out after 15 seconds, and
`ros2 topic hz /scan --window 10` timed out after 30 seconds. No new `/scan`
message or live frequency was observed.

### Classification

The result is **D: other**.

- **A is false:** `zero_packets` did not increase while `received_packets`
  increased by 306, so all received packets were not zero-position packets.
- **B is not supported:** `published_scans` did not increase, but the same 306
  frames were counted as out-of-order. The source path drops out-of-order
  frames before decode/publish; there is no evidence that a publisher thread
  stopped.
- **C is false:** the `/scan` publisher exists and RViz has a matching QoS
  subscription, but no new LaserScan was published during the sample window.
- **D evidence:** accepted frames and bytes continued to arrive, while
  `out_of_order_sequences` increased one-for-one and `published_scans` stayed
  flat. Flags, identity, and CRC rejection counters stayed at zero. This is a
  sequence/order handling condition in the incoming stream, not the
  `ESTABLISHED`-with-no-`received_packets` protocol-parse condition.

## Sequence tracker source audit (2026-08-29 14:50:24 +08:00)

This was a read-only source/runtime inspection. No S3 device, bridge, RViz2,
Static TF, TCP endpoint, source, configuration, protocol, or protected service
process was stopped, restarted, or modified; only the requested diagnostic CLI
processes were launched. This section was appended as the only file write.

### Runtime snapshot

The running container was `s3-ydlidar-bridge` (`6d0b37634036`). The current
diagnostics sample reported `tcp_connection_state=connected`,
`accepted_connections=2`, `disconnects=1`, `recv_bytes=10074842`,
`received_packets=94744`, `flags_errors=0`, `identity_errors=0`,
`crc_errors=0`, `zero_packets=3024`, `duplicate_sequences=0`,
`out_of_order_sequences=37377`, `sequence_jumps=57343`,
`sequence_wraps=0`, `published_scans=54315`, and `stale=true`.
The TCP sample remained `ESTABLISHED` from `192.168.31.239` (the S3 peer) to
the bridge listener on port 8765. The supplied 15-second pair remains
`received_packets=82828 -> 83134` (+306) and
`published_scans=54315 -> 54315` (+0), with `out_of_order_sequences` also
increasing by 306 and `zero_packets` unchanged.

### Source findings

The implementation is in `src/s3_ydlidar_bridge/src/framing.cpp:235-256` and
the drop/publish handling is in `src/s3_ydlidar_bridge/src/bridge_node.cpp:194-209`.

1. **Comparison rule:** strict `+1` is not required for acceptance. The first
   sequence is accepted; equal is `kDuplicate`; lower is `kOutOfOrder`; a
   greater value other than `last + 1` is `kJump` and is accepted; and
   `0xFFFFFFFF -> 0` is the special accepted `kWrap` case.
2. **`sequence_jumps` and `out_of_order` conditions:** `sequence_jumps` is
   incremented only for `kJump` (greater than `last` and not `last + 1`).
   `out_of_order_sequences` is incremented only for `kOutOfOrder` (strictly
   lower than `last`). Duplicates use their separate counter. Duplicate and
   out-of-order frames return before decode and publish; jumps and wraps
   update `last_` and continue through decode/publish.
3. **TCP accept reset:** a new `accept()` does not clear `last_`. The
   `sequence_tracker_` is a `BridgeNode` member (`include/s3_ydlidar_bridge/bridge_node.hpp:29`).
   `TcpServerTransport::run()` creates a per-connection assembler and updates
   connection counters (`src/s3_ydlidar_bridge/src/transport.cpp:149-164`),
   but never calls `SequenceTracker::reset()`. The only `reset()` definition
   is the unused tracker method at `framing.cpp:259-262`.
4. **S3 restart/lower sequence:** a S3-only restart or reconnect does not
   automatically start a new epoch. If the new sequence is below the retained
   `last_`, it is classified out-of-order and dropped until values exceed the
   old high-water mark. A bridge process restart starts with `have_last_=false`;
   `0xFFFFFFFF -> 0` is accepted only as the explicit wrap case, not as S3
   reboot detection.
5. **Non-`+1` handling:** the bridge does not count every non-`+1` value as
   out-of-order. Forward gaps are `sequence_jumps` and are accepted; only
   lower values are out-of-order, with equal values counted as duplicates and
   the maximum-to-zero transition counted as a wrap.
6. **Epoch/stream identity:** there is no connection-epoch or `boot_id`
   mechanism. A `stream_id` field exists in the frame header and is checked
   against `s3_expected_stream_id` (`framing.cpp:67, 99-103`), but it is an
   identity check only and does not reset or partition the sequence tracker.
7. **Diagnostics visibility:** diagnostics expose aggregate
   `duplicate_sequences`, `out_of_order_sequences`, `sequence_jumps`, and
   `sequence_wraps`, plus `recent_valid_packet_time_ns` and `stale`
   (`bridge_node.cpp:278-287`). They do **not** expose
   `last_sequence`, `last_packet_sequence`, `first_sequence`, `last_flags`,
   or `last_valid_scan_age`; `last_` is private tracker state.

### Sequence diagnosis

The observed one-for-one increase of `received_packets` and
`out_of_order_sequences` is therefore not the normal latest-only skipped-frame
case: skipped forward values would increase `sequence_jumps` and still be
processed. It indicates frames arriving below the bridge's retained sequence
high-water mark (for example, an unrecognized post-reconnect/reboot epoch or
reordered older data), and those frames are intentionally dropped before
decode/publish. This source conclusion requires no repair action.

## TCP connection epoch repair and live verification (2026-08-29 15:19:56 +08:00)

### Scope and files changed

Only the ROS2 bridge implementation, its focused tests, and this diagnostic
document were changed. No S3 or STM32 source, S3RD field, 26-byte header, CRC,
`device_id`, `stream_id`, TCP protocol, RViz configuration, or Static TF was
modified. The changed implementation/test files are:

- `src/s3_ydlidar_bridge/include/s3_ydlidar_bridge/framing.hpp`
- `src/s3_ydlidar_bridge/include/s3_ydlidar_bridge/transport.hpp`
- `src/s3_ydlidar_bridge/include/s3_ydlidar_bridge/bridge_node.hpp`
- `src/s3_ydlidar_bridge/src/framing.cpp`
- `src/s3_ydlidar_bridge/src/transport.cpp`
- `src/s3_ydlidar_bridge/src/bridge_node.cpp`
- `src/s3_ydlidar_bridge/test/test_framing.cpp`
- `src/s3_ydlidar_bridge/test/test_tcp_server.cpp`

### Repair behavior

`TcpServerTransport` now increments a bridge-local `connection_epoch` for each
successful `accept()` and emits ordered open/close events. The frame callback
also carries that epoch; it is not encoded into S3RD and does not alter the
existing `stream_id=1` identity check.

On open, `BridgeNode` calls `SequenceTracker::beginConnection(epoch)`, which
clears `have_last`, `last`, `first_sequence`, and `last_flags` before any frame
from that connection is handled. On close,
`SequenceTracker::endConnection(epoch)` clears the same sequence state for the
closing epoch. Consequently, the first protocol-valid packet in a new TCP
connection is `kFirst`, even if its sequence is lower than the previous TCP
connection's high-water mark.

Within one connection the prior behavior is preserved: a forward non-`+1`
sequence is accepted and increments `sequence_jumps`; an equal sequence is
dropped as `duplicate`; a lower sequence is dropped as `out_of_order`; and only
the exact `uint32` transition `0xFFFFFFFF -> 0` is accepted as a wrap. Flags
remain validated with their existing CT-bit relationship and allowed-mask
semantics.

Diagnostics now include `connection_epoch`, `first_sequence`, `last_sequence`,
`last_flags`, `last_valid_packet_time_ns`, and `last_valid_scan_age`.
`last_valid_scan_age` is a monotonic-clock age in nanoseconds, or `never` when
no valid LaserScan has been published. The pre-existing
`recent_valid_packet_time_ns` field is retained as a compatibility alias.

### Tests and static validation

`SequenceTracker` tests cover one connection receiving `100 -> 103` (accepted
jump), a duplicate `103` (dropped), an old `102` (dropped), epoch-state release
on disconnect, and a new connection whose first sequence is `1` (accepted as
first with no inherited state). The TCP test covers an actual half-frame,
sticky frames, two connections, open/close event order, and frame epochs.
Existing protocol tests continue to cover flags `0` and `1`, CRC,
`device_id`, `stream_id`, and unknown-flags errors.

```text
colcon build                         PASS
colcon test                          PASS
colcon test-result --verbose         29 tests, 0 errors, 0 failures, 0 skipped
git diff --check                     PASS
```

The build emitted only pre-existing warnings from the vendored YDLIDAR SDK.

### Live TCP bridge observation

The previous bridge container was stopped only to release port 8765 and run the
updated binary. The supplied command's executable token
`s3_ydlidar_bridge` is not installed (`No executable found`); the actual CMake
installed executable `s3_ydlidar_bridge_node` was run with the supplied TCP,
device, and stream parameters. `s3_expected_flags` is not a declared bridge
parameter, so that supplied override did not change behavior; the declared
and effective existing parameter remained `s3_allowed_flags_mask=1`. The
updated bridge remained running at the end of this observation. No S3 flash or
reset action was performed.

The concurrent `/scan` frequency sampler ran for 120 seconds. The first
diagnostics snapshot and a later in-window snapshot were:

| Field | First snapshot | Later snapshot |
| --- | ---: | ---: |
| `connection_epoch` | 1 | 1 |
| `received_packets` | 711 | 2518 |
| `published_scans` | 676 | 2390 |
| `sequence_jumps` | 710 | 2517 |
| `duplicate_sequences` | 0 | 0 |
| `out_of_order_sequences` | 0 | 0 |
| `first_sequence` | 423621 | 423621 |
| `last_sequence` | 428213 | 439931 |
| `stale` | false | false |

The final post-window diagnostic continued to advance:

```text
tcp_connection_state: connected
connection_epoch: 1
received_packets: 5759
published_scans: 5456
sequence_jumps: 5758
duplicate_sequences: 0
out_of_order_sequences: 0
first_sequence: 423621
last_sequence: 460930
last_flags: 0
last_valid_packet_time_ns: 59140168844276
last_valid_scan_age: 40903451
stale: false
```

`/scan --once` received a `sensor_msgs/msg/LaserScan`. The 120-second
`ros2 topic hz /scan --window 10` output was consistently about `16 Hz`
(the last observed lines were 16.2--16.6 Hz). A separate range check counted
128 `+inf` values, so the 360-sample scan contained 232 finite ranges. Port
8765 remained `ESTABLISHED` as
`192.168.31.101:8765 -> 192.168.31.239:49750`.

### RViz2 result

RViz2 was reopened in the running bridge container with
`/ws/config/rviz/radar_scan.rviz`; it ran with OpenGL 4.5 and has a matching
BEST_EFFORT/VOLATILE `/scan` subscription. However, its log repeatedly reports
that `laser_frame` messages are dropped because the message-filter queue is
full. No `rviz_world -> laser_frame` Static TF publisher is present in this
runtime, so RViz2 cannot presently render the finite scan points. Static TF was
not restarted or changed. Therefore the bridge now publishes live scans, but
the RViz2 display did **not** reappear with visible scan points in this run.

## Full-revolution scan assembly implementation (2026-08-29)

This change is limited to `ROS2_WIN` bridge source, tests, and documentation.
ESP32-S3, STM32, S3RD fields, GPIO, TCP transport contract, and all firmware
were left unchanged. The vendored YDLIDAR SDK 1.0.6 remains the only inner
packet parser through `parseMemoryChannel`; no second checksum or distance/
angle parser was introduced.

The TCP path retains byte-stream framing: arbitrary `recv()` chunks are fed to
the S3RD assembler, which resynchronizes on the four-byte magic, waits for the
26-byte little-endian header and declared payload, verifies the outer
CRC16-Modbus, and drains every valid sticky frame in FIFO order. Sequence
duplicates and old frames are still rejected; forward gaps, including modulo
uint32 wrap, are accepted and counted. A new TCP connection increments its
epoch, resets sequence state, and clears any incomplete mapper revolution.

The mapper now uses the validated YDLIDAR CT bit 0 (also checked against S3RD
flags bit 0) as the only revolution boundary. The first zero packet starts a
fixed-angle accumulation grid. The next zero packet publishes exactly the
preceding accumulated revolution, with uncovered bins left `+inf`, then starts
the next grid. A boundary absent for the configured 1000 ms timeout drops the
partial revolution and increments `scan_timeout`/`incomplete` diagnostics.
Diagnostics include `decoded_packets`, `accumulated_packets`,
`revolutions_published`, `valid_points`, `partial_revolutions_dropped`,
`coverage_ratio`, `frame_count`, `sequence_gap`, `scan_time`, `zero_timeout`,
and `incomplete`. The extractor also rejects payloads shorter than the
10-byte YDLIDAR packet header before accessing the CT byte, even if a caller
supplies a looser minimum-payload parameter.

Offline verification after the final source change:

```text
docker compose build ros2-dev                         PASS
colcon build --symlink-install                        PASS
colcon test                                            PASS (6 targets)
colcon test-result --verbose                           36 tests, 0 errors,
                                                       0 failures, 0 skipped
git diff --check                                       PASS
```

The golden S3RD replay covers a split first frame followed by sticky frames
and confirms that no scan is published until the next zero packet. Mapper tests
cover missing-angle infinity bins, coverage by angle interval, zero-boundary
timeout, timestamps/frame id, sequence gaps, and connection reset. These are
offline fixtures; live S3 timing, real zero-boundary cadence, Wi-Fi reconnect,
and RViz rendering still require Windows/S3 hardware validation.
