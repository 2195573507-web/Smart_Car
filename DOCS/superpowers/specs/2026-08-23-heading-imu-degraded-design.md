# Heading and IMU Degraded-Startup Design

## Scope

The CM7 startup gate will tolerate a BMI323 transient without unlocking a
moving actuator prematurely. Normal operation remains BMI323-backed DualAHRS;
after three bounded calibration/precheck failures, LSM303 becomes the active
legacy AHRS source and the system publishes an explicit degraded state through
existing status/error fields and logs. No BLE or SCBP payload layout changes.

## Startup and Calibration

After both init workers and the self-test window, the manager waits 2000 ms
before calibration admission. During the warmup, the BMI producer forwards 20
raw gyro samples to a precheck. A sample is bad when any axis exceeds 300 LSB;
five bad samples reject the attempt. Missing samples also reject the attempt at
the short precheck deadline. A zero-sample window is treated as a definitive
disconnect so the six-second handoff target is not consumed by retries; a
partial but unreasonable window follows the three-attempt policy. Rejected
attempts are separated by 2000 ms and the shared retry counter is capped at
three. Static-window motion failures use the same retry helper. The third
failure enters `IMU_DEGRADED` instead of the terminal error state.

An explicit BMI323 initialization failure, or an init timeout after LSM303 has
completed, requests the same degraded finalization immediately after a valid LSM
sample is available; it does not wait for a BMI self-test that cannot succeed.
The existing zero-PWM `BOOT_READY` transaction is still admitted before the
degraded `CAL_EVENT`, so the S3 calibration lock and BLE/SCBP contract remain
unchanged.

The degraded path finalizes LSM-only acquisition, synthesizes a conservative
LSM calibration/leveling result from a valid stationary LSM sample, and marks
the boot lifecycle ready with a separate degraded flag. The existing IMU
cal-status payload carries the error byte; a log line names `LSM303` as the
active source. The startup coordinator accepts normal or degraded readiness and
only then creates the motor task.

## Heading Control

The controller owns a wrapped `cur_yaw` accumulator. A public IMU-update entry
integrates `gyro_z * dt` on every valid sensor update, independent of the
heading lock. The first valid sample anchors the accumulator; later lock or
unlock transitions never overwrite it. The control step latches target yaw only
on a straight command and computes error/integral/differential output only while
locked. Degraded LSM-only operation may use its legacy magnetic yaw when no BMI
gyro is available; this limitation is logged and remains hardware-unverified.

## Motor Polarity

No motor source change is planned. The current M2 negative MSPD value is
consistent with the documented RF encoder inversion because the sign is applied
once before PID and not again in `$pwm`. Physical direction still requires a
lifted-wheel, per-channel UART/PWM and encoder capture.

## Verification

The source-level acceptance checklist is:

| Criterion | Source behavior | Evidence still required |
| --- | --- | --- |
| Cold boot <= 5 s, normal or degraded `locked_flag=1` | Degraded admission is bounded by the warmup/precheck path; normal admission retains the existing `6000 ms` static window | Timestamped cold-boot log; the 5 s target conflicts with the retained 6 s calibration contract and needs an explicit timing decision |
| 10 s straight run produces correction | `cur_yaw` integrates independently and locked control computes differential output | Flashed image with `[HEADING]` trace while driving |
| 30 deg manual yaw disturbance | Wrapped error is calculated against the latched target and correction is applied with the existing sign convention | Bench/vehicle yaw disturbance capture |
| BMI323 absent/invalid switches within 6 s | Missing/invalid precheck is bounded and finalizes LSM303 degraded mode; `degraded` and log state are published | Physical disconnect test with boot timestamps and `locked_flag` trace |

Host assertions should cover precheck thresholds, retry/degraded transitions, yaw
wrap, and lock transition continuity. The CM7 target is clean-built below;
sensor values, UART/BLE delivery, motor direction, and vehicle behavior remain
hardware acceptance items.
