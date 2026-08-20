# LSM303 Body-Frame Sign Alignment

## Goal

Align the LSM303 sensor axes with the vehicle Body Frame before calibration and
attitude estimation. The LSM303 accelerometer and magnetometer X/Y components
are negated at the sensor-manager boundary; Z is preserved. BMI323 Primary axis
definitions remain unchanged.

## Ownership And Data Flow

```text
LSM303 raw read
    -> imu_manager Body-Frame map (X=-X, Y=-Y, Z=Z)
    -> raw snapshot / static calibration / Leveling
    -> legacy filter + attitude and DualAHRS Redundant estimator
```

The map is applied once at `imu_update_lsm303()`. Downstream code must consume
the mapped values without applying a second accelerometer sign change. Existing
magnetometer yaw convention handling is reconciled so the mapped magnetic
vector is used consistently by both legacy attitude and DualAHRS.

## DualAHRS Coherence

Redundant Roll and Pitch are solved from the mapped LSM303 gravity vector. Any
final Euler correction or zero-reference output is followed immediately by
`euler_to_quaternion()` so the redundant quaternion in the 80-byte payload
represents the same Roll/Pitch/Yaw values. `delta_rad` is computed from the
final primary and redundant output attitudes, after zero reference and all
sign corrections.

## Compatibility And Risks

- The 80-byte schema-2 payload, field offsets, message IDs, and S3/App relay
  remain unchanged.
- Calibration bias values remain in the mapped Body Frame because the map is
  applied before the static window accumulates samples.
- A wrong physical axis assumption would invert the observed LSM303 motion;
  source/build checks cannot establish the sensor's installed orientation.
- No GPIO, IOC, bus timing, task cadence, or BMI323 behavior changes.

## Verification

1. Static source checks confirm exactly one LSM303 X/Y sign map and no duplicate
   downstream accelerometer inversion.
2. Host-level deterministic checks exercise mapped gravity vectors and verify
   Redundant Euler/quaternion consistency plus delta calculation inputs.
3. CM7 configure/build verifies compilation and link closure only.
4. Device acceptance remains separate: left/right roll and up/down pitch must
   move Primary and Redundant in the same direction, and a static tilted pose
   must hold all `diff_deg` values within `+/-1.0` degrees.
