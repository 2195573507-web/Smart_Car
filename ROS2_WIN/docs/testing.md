# Offline test evidence

The C++ tests exercise the experimental `S3RD` envelope, including split,
sticky/multi-frame, byte-wise magic resynchronization, CRC/version/length/type/
flags/identity rejection, CT/flags consistency, duplicate/old/forward-gap/
uint32-wrap sequence handling, a short-payload guard before CT access, and a
loopback TCP reconnect.

The official-decoder tests construct a deterministic triangle packet using the
checksum and field layout implemented by the vendored SDK. They assert decoded
raw q2 distances and reject a checksum mutation. Mapper tests verify that the
first CT/flags zero packet only starts a revolution, the next zero packet alone
releases the preceding complete `LaserScan`, missing/invalid bins remain
infinity, coverage is angle-interval based, and a zero-boundary timeout drops
rather than publishes a partial scan. They also verify the timestamp,
`laser_frame`, per-revolution gap count, and a connection reset that prevents
cross-epoch mixing.

Transport tests verify replay and unconfigured behavior. The TCP test opens
only loopback sockets and never opens serial devices or sends radar commands.

The protocol tests cover ordinary flags `0x0000`, zero-position flags `0x0001`,
CT/flags mismatch, unknown flags, CRC errors, and device/stream identity errors.
A golden S3RD replay deliberately splits the first outer frame and then sends
sticky frames; it is published only on the next zero packet. The Python sender
under `testdata/` covers legal no-intensity/intensity frames, half/sticky reads,
bad CRC/version/length, duplicate/jump sequences, and disconnect/reconnect.
These are deterministic fixtures, not real-device captures.

The telemetry decoder tests use the canonical `Common/SRP` encoder to
construct complete `0x14`, `0x11`, and `0x10` frames. They verify SRPv4
priority, `STREAM_DATA`, fixed payload sizes, DualAHRS schema, IMU sensor IDs,
and finite floating-point values. Legacy `0x210` is intentionally reported as
missing source freshness; these tests do not claim `/odom`, TF, SLAM, or live
device acceptance.
