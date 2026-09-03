# Offline S3 replay data

`s3_replay_sender.py` is a deterministic TCP sender for the experimental
`S3RD` envelope. It generates both official YDLIDAR triangle payload shapes:
two-byte distance samples without intensity and three-byte quality/distance
samples with intensity. The scenarios cover split reads, sticky/multi-frame
reads, bad CRC/version/length, duplicate and jump sequences, and disconnect /
reconnect.

Example (with the bridge configured as `transport:=tcp`):

```bash
python3 /ws/testdata/s3_replay_sender.py --scenario sticky
python3 /ws/testdata/s3_replay_sender.py --scenario bad_crc
python3 /ws/testdata/s3_replay_sender.py --scenario reconnect
```

The sender defaults to `device_id=1` and `stream_id=1`, matching
`config/bridge.yaml`. Use `--device-id` and `--stream-id` when exercising
identity-rejection cases or a different approved fixture configuration.

These bytes are test fixtures, not captures from a real S3 or YDLIDAR device.
