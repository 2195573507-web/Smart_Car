# Heading and IMU Startup Fix

## Goal

Prevent BMI323 startup transients from deadlocking attitude startup, provide a
bounded LSM303-only degraded mode, and keep heading yaw integration alive while
the heading lock is not active. Preserve BLE/SCBP payloads and wheel PID gains.

## Phases

- [x] Phase 1: Read project rules, architecture docs, source, and dirty-worktree boundaries.
- [x] Phase 2: Record design and evidence for startup, calibration, heading, and motor polarity.
- [x] Phase 3: Implement warmup, raw gyro precheck, bounded retries, and LSM-only fallback.
- [x] Phase 4: Decouple heading yaw integration from lock gating and preserve yaw across lock transitions.
- [x] Phase 5: Run host/static checks and CM7 build; record hardware acceptance gaps.

## Verification Targets

- Cold boot: ready or degraded lock within the documented bounded startup path.
- Straight command: nonzero heading correction after a deliberate yaw disturbance.
- BMI323 unavailable: LSM303 fallback is announced and actuator gate remains safe.
- Motor M2: preserve one-time encoder sign correction and direct PWM sign handling.
