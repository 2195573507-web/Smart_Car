# Findings: S3 Chassis Speed Link Repair

- App/S3 already validates and forwards App `0x2D` as SRP `0x06` with a
  16-byte payload.
- CM7 `s3_service_on_frame()` currently handles only SRP `0x02`; SRP `0x06`
  is classified as motion but falls through to invalid-parameter ACK.
- `Application/Chassis` has only README files; no runtime or kinematics source
  is currently compiled.
- MotorBoard target speeds are zero-initialized and only update through the
  target API. `$MSPD:` is parsed feedback; PID output is sent as `$pwm:`.
- Confirmed track width: `CHASSIS_TRACK_WIDTH_MM = 193.0f`.
