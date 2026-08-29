# Bridge design

The bridge has four isolated responsibilities:

1. `TcpChunkAssembler` accepts arbitrary TCP chunks and repeatedly invokes
   `S3FrameExtractor` until the buffered bytes contain no complete frame. The
   extractor validates the experimental `S3RD` envelope, accepts only flags
   inside the configured mask, and requires flags bit 0 to match the payload CT
   bit 0. The assembler bounds memory and preserves split/sticky/multi-frame
   reads in FIFO order.
2. `TcpServerTransport` accepts one LAN TCP client at a time, uses a fixed
   recv buffer, emits jump frames without waiting for missing sequence numbers,
   and returns to accept after disconnect. `ReplayTransport` supplies an
   offline payload; `UnconfiguredTransport` remains the safe default.
3. `OfficialDecoder` supplies a read-only in-memory `ChannelDevice` to the
   official SDK's existing parser functions. It does not implement a second
   YDLIDAR checksum or point algorithm.
4. `ScanMapper` applies the official triangle node units (`distance_q2 / 4000`
   metres and Q6 angle degrees) to a fixed ROS scan grid. A CT/flags bit-0
   zero-position packet starts a revolution; the following zero-position packet
   seals and publishes the prior revolution, then starts the next. Invalid,
   out-of-range, and uncovered bins remain infinity (configurable to NaN).

The node drops stale, duplicate, and out-of-order frames; it counts and accepts
forward jumps, including a forward jump across the uint32 wrap transition. A
zero-position packet never publishes on its own. The first boundary only
starts accumulation; each later boundary publishes the preceding complete
revolution. If a boundary is absent for `zero_packet_timeout_ms`, the partial
revolution is dropped and the mapper waits for a new boundary. Connection epoch
changes also drop a partial revolution. Decoder failures are counted. A
`diagnostic_msgs/msg/DiagnosticArray` reports connection, packet validation,
sequence gaps, `coverage_ratio`, scan duration, zero-boundary timeouts, stale,
and publish-rate state. None of those paths emits a control command. The only
sensor ROS output is `sensor_msgs/msg/LaserScan` on `/scan` with
`SensorDataQoS`.
