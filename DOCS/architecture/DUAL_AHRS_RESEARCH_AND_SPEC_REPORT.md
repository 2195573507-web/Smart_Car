# Dual-AHRS Research and System Architecture Specification

Status: RESEARCH AND DESIGN PROPOSAL (2026-08-18)  
Scope: Smart_Car STM32H757 CM7, ESP32-S3, and macOS SwiftUI App.  
Evidence rule: CONFIRMED means visible in the current workspace. PLANNED means a proposed design. UNVERIFIED requires device, electrical-bus, bench, or integration evidence. This is a read-only research deliverable; it does not claim a flash, sensor, UART, BLE, radar, or vehicle acceptance result.

## 0. Executive Summary and Decision

The recommended target is a two-rate, heterogeneous Dual-AHRS system. The primary attitude estimator uses BMI323 gyro and accelerometer data at a high acquisition ODR, a PWM-aware anti-alias and vibration-rejection pipeline, and LSM303 magnetometer measurements as a low-rate absolute yaw reference. The redundant estimator uses only LSM303 accelerometer plus magnetometer at 20-50 Hz as a strongly damped static gravity and field reference. It has no gyro integration, so it cannot accumulate gyro drift. It is deliberately not a high-bandwidth substitute for the primary estimator.

The recommended primary algorithm is VQF 9D at a 200 Hz fusion boundary, with a bounded Mahony 9D implementation retained as a C99 replay baseline and possible simple fallback. Start with BMI323 at 400 Hz and decimate to 200 Hz if timing evidence is absent. Promote to 800 Hz acquisition and 200 Hz fusion only after measured ODR, SPI/DMA service time, task worst-case execution time, and overflow counters demonstrate adequate margin. An MEKF is a later option, not the first integration target.

The existing Smart_Car protocol is a hard compatibility boundary. Current SCBP-V3 MSG_ID 0x0201 is a 30-byte single-attitude payload. A dual-attitude layout is a PLANNED schema-gated extension, never a silent replacement of the current 30 bytes. The current workspace also has separate STM-S3 and App BLE envelopes; S3 must re-envelope validated payload bytes and must not reinterpret units or quaternion order.

## 1. Project Background and System Topology

### 1.1 Confirmed current platform facts

| Area | CONFIRMED workspace fact | Dual-AHRS consequence |
| --- | --- | --- |
| System ownership | STM32 owns deterministic sensing, calibration, attitude, local safety, actuation and final motion authority. ESP32-S3 owns gateway, BLE GATT, STM UART2, radar UART1 GPIO44 and radar PWM GPIO4. macOS owns operator UI. | Fusion, estimator health and any safety fallback remain on STM32. S3 and App are display/transport domains. |
| BMI323 | Active middleware source supports configured 100, 200, 400 and 800 Hz ODR; SPI1 PA5/PA6/PA7 plus CS PC4; a 512-entry ring is present. | High-rate acquisition groundwork exists, but physical timing and online sensor behavior remain UNVERIFIED. |
| Current BMI use | BMI data is limited to lifecycle, calibration, vibration, diagnostic and telemetry paths; legacy attitude does not use it. | Implement a separate primary estimator module first. Do not alter current LSM303 attitude semantics in place. |
| LSM303 | Active path uses I2C4, accel address 0x19 and mag address 0x1E. It supplies current calibrated/filter/attitude data. | Preserve it as an independently timed redundant channel while the primary path is introduced. |
| Current filter | Existing LSM303 filter is median-of-5 followed by EMA. PWM profiles currently select EMA alpha values. | Reuse profile ownership, but add explicit primary notch, low-pass, decimation and confidence scheduling. |
| Current attitude | Existing attitude code derives Euler roll/pitch/yaw from filtered LSM303 data. Quaternion fields exist in the public state but the present algorithm does not populate a quaternion solution. | A true quaternion primary estimator is new capability, not a tuning change. |
| Current protocol | SCBP-V3 ATTITUDE 0x0201 is 30 bytes: radian Euler values, degree Euler values, timestamp, source and status. | Mixed 30-byte/dual payload endpoints must be protected by explicit length and schema checks. |
| Gateway and App | S3 has a source bridge and App BLE parser, FFE2 is telemetry notify. Current end-to-end relay and physical delivery are not accepted. | Transport parsing and UI must be separately tested before any runtime conclusion. |

### 1.2 Target data flow

~~~mermaid
flowchart LR
  BMI[BMI323 SPI1
100 to 800 Hz] --> ALIGN[Timestamp, bias,
axis alignment]
  ALIGN --> PRE[Rate-domain LPF plus
PWM-selected notch]
  PRE --> DECIM[Anti-aliased decimator
400 or 800 to 200 Hz]
  DECIM --> PAHRS[Primary AHRS
VQF 9D or Mahony 9D]
  LSM[LSM303 I2C4
accel plus mag] --> CAL[Hard/soft iron,
axis alignment]
  CAL --> RAHRS[Redundant AHRS
damped tilt compass]
  PAHRS --> HEALTH[Health and
wrapped delta attitude]
  RAHRS --> HEALTH
  HEALTH --> STM[STM32 SCBP-V3]
  STM --> S3[ESP32-S3 bridge]
  S3 --> BLE[BLE FFE2 Notify]
  BLE --> APP[macOS App
primary, redundant, delta]
~~~

### 1.3 Control, fault and time ownership

Every BMI sample and every LSM sample needs a monotonic STM timestamp. The primary estimator consumes ordered BMI samples; it must not use the current diagnostic behavior that keeps only the newest high-rate sample and discards preceding samples. The redundant solution uses its own LSM timestamp and is compared only after temporal alignment. Radar PWM is an estimator input for profile selection, not proof that the radar motor is turning. A primary failure must become an explicit local STM status and policy decision; it must not silently change algorithms or give BLE/App safety authority.

## 2. Open-Source Research and Algorithm Comparison

### 2.1 Audited open-source references

| Project | Public source | Architecture lesson used here |
| --- | --- | --- |
| Betaflight | https://github.com/betaflight/betaflight | src/main/sensors/gyro.c separates fast acquisition from filtered/downsampled values and accumulates samples; src/main/common/filter.c supplies PT2/PT3, biquad and notch primitives. Adopt the sample-domain separation, not arbitrary flight-controller coefficients. |
| PX4 ECL and EKF2 | https://github.com/PX4/PX4-ECL and https://github.com/PX4/PX4-Autopilot | EKF2 ingests timestamped delta-angle/delta-velocity IMU data, tracks gyro/accel/mag bias states, gates innovations and publishes estimator health. It is the reference for observability and fault reporting. |
| ArduPilot | https://github.com/ArduPilot/ardupilot | AP_InertialSensor and HarmonicNotchFilter demonstrate per-sensor filtering and dynamic/harmonic notch scheduling. The relevant lesson is calibrated frequency scheduling plus measurable diagnostics. |
| VQF | https://github.com/dlaidig/vqf | VQF exposes separate updateGyr, updateAcc and updateMag calls, 6D/9D quaternions, rest detection, gyro bias estimation and magnetic-disturbance rejection. It maps well to unequal BMI and LSM data rates. |
| Mahony AHRS | https://github.com/PaulStoffregen/MahonyAHRS | Small deterministic PI feedback implementation based on gravity/magnetic vector residuals. It is suitable for an independently replayable C99 baseline. |
| Madgwick AHRS | https://github.com/arduino-libraries/MadgwickAHRS | Gradient-descent correction with a compact state and low memory. It is a useful baseline but needs external bias and disturbance policies. |
| Fusion | https://github.com/xioTechnologies/Fusion | C reference that combines quaternion feedback with accelerometer/magnetometer rejection and recovery timers. It provides a concrete rejection-state design reference. |
| Bosch BMI323 Sensor API | https://github.com/boschsensortec/BMI323_SensorAPI | Official register and bus-callback reference; use to verify local ODR, reset, data-ready and FIFO settings against the exact hardware revision. |
| ST LSM303AGR driver | https://github.com/STMicroelectronics/lsm303agr-pid | Official family API showing ODR and magnetometer offset support. It must not be assumed to describe every LSM303-compatible board variant. |

### 2.2 Selection matrix for STM32H757 CM7

| Algorithm | Arithmetic and determinism | Online gyro-bias behavior | Mag disturbance behavior | Integration risk | Decision |
| --- | --- | --- | --- | --- | --- |
| Mahony 9D | Very small fixed state; vector products, normalization and PI. Excellent bounded WCET. | Integral error feedback when trusted. | Requires explicit norm/direction gate and recovery policy around the core. | Low RAM/Flash; convention/sign errors are the main risk. | Required replay baseline and viable simple fallback. |
| Madgwick 9D | Small-to-medium; gradient vector normalization each update. | No complete native bias state in the common form. | Requires external gate. Beta is motion/noise dependent. | Low memory, but coefficient tuning is less physically transparent. | Baseline comparator, not final target. |
| VQF 9D | Medium; fixed-size quaternion/filter operations. At 200 Hz it should be feasible on CM7, but WCET must be measured. | Rest and motion bias estimation are first-class. | Native magnetic-disturbance detection/rejection and 6D/9D outputs. | More parameters/code than Mahony; wrapper must prevent dynamic allocation. | Recommended primary target. |
| MEKF | Largest cost; nominal state, covariance propagation, Jacobians and innovation updates. | Model-based bias state with covariance. | Innovation consistency tests and sensor selection are strong. | Highest RAM, numerical, timing and tuning burden. | Phase-3 option only after replay evidence shows VQF is insufficient. |

### 2.3 Final algorithm decision

Use VQF 9D at the 200 Hz output boundary. Use the BMI gyro and accelerometer at their actual timing; supply the LSM magnetometer only when a fresh calibrated sample is available. The VQF wrapper must retain VQF 6D output for magnetic-rejection diagnostics. Implement Mahony 9D with the same calibration, filter and gating contract for golden replay comparison. Do not start with an MEKF because current Smart_Car has no proven high-rate timing budget, covariance telemetry, innovation test plumbing, or hardware sensor acceptance.

## 3. BMI323 ODR Tuning and High-Rate Filtering Pipeline

### 3.1 BMI323 and LSM303 physical boundaries

BMI323 combines a 3-axis accelerometer and a 3-axis gyroscope. Higher ODR reduces the sample interval and gives more guard band for anti-alias/notch processing, but raises ISR/DMA wake frequency, bus transactions, buffer pressure and CPU work. Noise density, bandwidth, current and exact FIFO/data-ready behavior depend on range, bandwidth and the exact Bosch configuration; do not invent a noise figure from another revision. Confirm against the installed part datasheet and an on-board measurement.

LSM303 is an accelerometer/magnetometer family rather than a gyro source. Its accelerometer gives an absolute gravity direction only when specific force is dominated by gravity. Its magnetometer gives an absolute field direction only after hard-iron/soft-iron correction and only when local magnetic interference is controlled. The local source names an LSM303-compatible device, so the exact WHO_AM_I, ODR, sensitivity, axis order and variant-specific registers must be verified before importing a particular LSM303 family datasheet value.

### 3.2 ODR and transport budgeting

For a conservative 16-byte SPI transfer per simultaneous raw sample, including command/dummy/status allowance, wire traffic is R_spi = 8 times 16 times f_s. This is 12.8, 25.6, 51.2 and 102.4 kbit/s at 100, 200, 400 and 800 Hz. This arithmetic is not a real-time acceptance result: chip-select gaps, driver transactions, DMA descriptors, RTOS activation, retry paths and shared-bus use must be added from measurement.

| Configured ODR | Period | 16-byte nominal wire traffic | 512 sample ring duration | Recommended role |
| ---: | ---: | ---: | ---: | --- |
| 100 Hz | 10 ms | 12.8 kbit/s | 5.12 s | Low-load baseline; too little anti-alias margin for strong vibration. |
| 200 Hz | 5 ms | 25.6 kbit/s | 2.56 s | Direct-fusion baseline for a slow platform. |
| 400 Hz | 2.5 ms | 51.2 kbit/s | 1.28 s | Initial recommended acquisition rate for 200 Hz fusion. |
| 800 Hz | 1.25 ms | 102.4 kbit/s | 0.64 s | Maximum requested dynamic margin; enable only with timing and overrun evidence. |

The ring capacity provides only temporary delay tolerance. It is not a license to let data become stale. For each test record configured ODR, measured ODR, timestamp jitter, maximum service latency, ring overflow, lock-contention drops, SPI status and task stack high-water mark.

### 3.3 Recommended processing sequence

1. Acquire every BMI sample in timestamp order through data-ready plus DMA, or a bounded polling/FIFO strategy proven not to lose samples.
2. Convert raw values to m/s2 and rad/s, apply bias and body-frame alignment.
3. Apply PWM-profile selected notches to vibration-dominated axes, then an anti-alias low-pass at the high sample rate.
4. Decimate only after anti-alias filtering. For 800 to 200 Hz use M = 4; for 400 to 200 Hz use M = 2.
5. Run quaternion propagation at each 200 Hz output. Run accelerometer correction at the same output rate. Run magnetometer correction only for fresh 20-50 Hz LSM samples.
6. Publish age, profile ID, cutoff, notch ID, decimator count, measured ODR and health flags with the state.

### 3.4 Second-order low-pass and notch equations

For input x[n] and output y[n], a normalized biquad is:

~~~text
y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
~~~

Let f_s be sample rate in Hz, f_c be cutoff in Hz, w0 = 2*pi*f_c/f_s, c = cos(w0), alpha = sin(w0)/(2*Q). For a second-order Butterworth low-pass use Q = 1/sqrt(2):

~~~text
b0 = (1-c)/2, b1 = 1-c, b2 = (1-c)/2
a0 = 1+alpha, a1 = -2*c, a2 = 1-alpha
~~~

Divide b0, b1, b2, a1 and a2 by a0 before storage. For a notch centered at f0, set w0 = 2*pi*f0/f_s and use:

~~~text
b0 = 1, b1 = -2*c, b2 = 1
a0 = 1+alpha, a1 = -2*c, a2 = 1-alpha
~~~

Q determines notch width. Measure f0 and Q from the calibrated PWM sweep; do not use a guessed universal resonance. A dual biquad notch means two independently identified modes or a fundamental plus a measured harmonic, not two copies of an assumed frequency.

A boxcar decimator is z[k] = (1/M) times the sum of y[k*M+i] for i from 0 through M-1. It has a sinc response and may be used only after an adequate anti-alias low-pass. A polyphase FIR is a later replacement if a measured phase/ripple target demands it.

### 3.5 ODR choice rule

Start at 400 Hz acquisition and 200 Hz fusion. Move to 800 Hz acquisition only if all four conditions pass: measured rate at least 90 percent of configured rate, zero unaccounted ring/FIFO overflow, bounded transfer/task WCET below the 1.25 ms period with margin, and a replay or sweep demonstrates lower vibration error without unacceptable phase lag. Direct 200 Hz fusion remains the low-complexity comparator.

## 4. Sensor Calibration Injection and Body Frame Alignment

### 4.1 Units, static bias and calibration ordering

Use SI units throughout estimator code: acceleration a in m/s2, angular rate w in rad/s, magnetic field m in uT, time t in seconds and angles in radians. Let raw engineering-unit values be a_raw, w_raw and m_raw. Apply static offsets before fusion:

~~~text
a_c = R_align_a times (a_raw - b_a)
w_c = R_align_g times (w_raw - b_g)
m_c = S_m times (m_raw - b_h)
~~~

Here b_a, b_g and b_h are accelerometer, gyro and hard-iron offsets in their native engineering units. R_align maps each sensor coordinate system to the approved body coordinate system. S_m is the magnetometer soft-iron transform. During static calibration the existing contract keeps radar PWM at zero. A stationary accelerometer has norm close to g = 9.80665 m/s2 and a stationary gyro has norm close to zero; finite sample quality and variance limits decide whether an offset is accepted.

### 4.2 Alignment matrix

For a vector v_s measured in sensor coordinates, use v_b = R_align times v_s. R_align must be orthonormal, R_align transpose times R_align = I, with determinant +1 for a right-handed rotation. Obtain it from measured board mounting, then validate signs by six known static gravity placements and positive right-hand rotations. Do not derive axis order from display orientation or a presumed sensor breakout layout.

### 4.3 Hard-iron and soft-iron ellipsoid model

The minimum hard-iron model is a translated sphere: norm(m_raw - b_h) squared = B squared. The full model maps the measured ellipsoid to a sphere:

~~~text
m_c = S_m times (m_raw - b_h)
~~~

Fit an ellipsoid center b_h and positive-definite quadratic Q such that (m_raw - b_h) transpose times Q times (m_raw - b_h) is approximately B squared. Factor Q using Cholesky or eigendecomposition to obtain S_m satisfying S_m transpose times S_m = Q, then scale to a local field reference B. Reject a fit if orientation coverage is too small, any eigenvalue is non-positive, condition number is excessive, or residual RMS exceeds the acceptance threshold. Store sensor ID, body-frame version, calibration temperature/range, fit residual and creation timestamp alongside the parameters.

## 5. Radar PWM Adaptive Vibration Rejection

### 5.1 Profile model and dynamic cutoff

For each tested PWM point p_i, store PWM percent, RMS for x/y/z/total in m/s2, dominant spectral peaks f0, optional harmonic peaks, Q, valid sample count, rejected sample count and a quality bit. Interpolate only between qualified neighboring entries. A table is safer than a high-order polynomial when a motor has narrow resonances.

Let p = PWM/100, r(p) be interpolated total RMS, r_ref be the zero-PWM RMS, and clamp(x,l,h) bound x between l and h. The proposed initial policy is:

~~~text
f_cutoff(p) = clamp(f_max - k_p*p - k_r*max(0, r(p)-r_ref), f_min, f_max)
~~~

f_min, f_max, k_p and k_r are tunable calibration parameters. For a 200 Hz output set f_max no higher than 0.40 times 200 Hz unless measured response justifies another bound. Use f_min at or above the maneuver bandwidth needed by the vehicle. On a profile miss use f_default, disable unverified notches and raise PROFILE_MISSING; do not extrapolate a resonance curve.

### 5.2 Dynamic gravity confidence

Let e_a = absolute value of norm(a_c)/g minus 1. Define PWM-dependent gravity tolerance T(p) = T0 + (T100-T0)*p and a smooth confidence:

~~~text
K_a(p,e_a) = clamp(exp(-((max(0, e_a-T(p))/sigma) squared)), K_min, 1)
~~~

K_a scales accelerometer correction only, never gyro propagation. Add a hard gate K_a = 0 when e_a exceeds E_hard(p), an accelerometer sample is invalid, or the sample age exceeds the maximum. T0, T100, sigma, K_min and E_hard are not facts; derive them from replay and bench data.

The magnetic confidence is independent of K_a. With B_ref as reference magnitude, set K_m to zero when absolute value of norm(m_c)-B_ref divided by B_ref exceeds tau_B, when horizontal field direction residual exceeds tau_dir, or when the magnetometer is stale. Restore K_m over a recovery interval rather than stepping yaw correction immediately to full weight.

### 5.3 Adaptive filter safety properties

PWM is not a sensor of vibration and cannot prove physical rotation. The policy must be bounded for invalid/missing PWM, invalid RMS, non-finite data and abrupt profile changes. Coefficient changes should reset a filter to a coherent state or cross-fade over a bounded time. The fast sensor path must use static buffers, no heap allocation, non-blocking diagnostics and explicit ownership of DMA/interrupt context. BMI failure must not block LSM303, calibration or the existing 10 ms service cadence.

## 6. Primary 9-DOF AHRS Mathematical Model

### 6.1 Quaternion convention and propagation

Use scalar-first unit quaternion q = [q0,q1,q2,q3], with norm(q) = 1, and body angular rate w = [wx,wy,wz] in rad/s after bias correction. The exact Earth-to-body versus body-to-Earth convention is a project decision; all rotation and App serialization code must use one convention. For q dot = 0.5 times q tensor-product [0,w], the component equations are:

~~~text
q0_dot = -0.5*(q1*wx + q2*wy + q3*wz)
q1_dot =  0.5*(q0*wx + q2*wz - q3*wy)
q2_dot =  0.5*(q0*wy - q1*wz + q3*wx)
q3_dot =  0.5*(q0*wz + q1*wy - q2*wx)
~~~

For dt seconds, a robust discrete update uses dtheta = w*dt and d = norm(dtheta). If d is above epsilon, dq = [cos(d/2), dtheta/d times sin(d/2)]; otherwise dq = [1, 0.5*dtheta]. Compute q_next = normalize(q tensor-product dq). Reject or clamp impossible dt and raise DT_INVALID rather than injecting a huge quaternion step.

### 6.2 Gravity and magnetic vector residuals

Let a_hat = a_c/norm(a_c) and let g_B(q) be the expected unit gravity vector rotated into the body frame from the current q. The gravity residual is:

~~~text
e_a = a_hat cross g_B(q)
~~~

Let m_hat = m_c/norm(m_c), and let h_B(q) be the expected local magnetic unit vector in body coordinates after declination convention is applied. The magnetic residual is:

~~~text
e_m = m_hat cross h_B(q)
~~~

The correction angular rate is w_fb = Kp_a times K_a times e_a plus Kp_m times K_m times e_m. Propagate with w_used = w_raw - b_g + w_fb. Cross-product order determines feedback sign; validate it by a known positive physical rotation test and a static recovery test.

### 6.3 Mahony anti-windup bias loop

A gated integral loop is:

~~~text
e = K_a*e_a + K_m*e_m
b_g_dot = -Ki_g*e
b_g_next = clamp_componentwise(b_g + b_g_dot*dt, -b_limit, b_limit)
~~~

Integrate only if rest/motion policy and at least one correction confidence are valid. If a component of b_g is at its positive or negative bound and b_g_dot points further outward, set that component of b_g_dot to zero. Freeze bias integration when both K_a and K_m are too low, gyro is saturated, or a timing/magnetic fault is active. Bias limits are in rad/s and must be justified by stationary data.

### 6.4 VQF and MEKF relation

The VQF wrapper should call gyro updates at each decimated BMI sample, acceleration updates at the primary correction cadence, and magnetometer updates only when a fresh LSM sample passes its gate. Preserve VQF 6D output alongside 9D output; divergence between them is a useful magnetic-health signal.

For a later MEKF, keep nominal state q and b_g, and represent small attitude error by delta-theta. A minimal error state is delta-x = [delta-theta, delta-b_g] transpose. With gyro white noise n_g and bias random walk n_b, linearized propagation has delta-theta_dot approximately equal to negative skew(w) times delta-theta minus delta-b_g minus n_g, and delta-b_g_dot equals n_b. A vector observation residual becomes r = z_hat cross z_pred with a Jacobian H. Innovation covariance is S = H times P times H transpose plus R; reject the measurement if r transpose times S inverse times r exceeds a chosen chi-square gate. This explains why MEKF brings higher state/covariance/tuning burden.

## 7. Redundant 6-DOF Static Tilt Compass Model

### 7.1 Robust filtering

For each calibrated LSM axis, take x_med[k] as median(x[k-4] through x[k]) then apply a strong one-pole IIR:

~~~text
x_f[k] = alpha*x_f[k-1] + (1-alpha)*x_med[k], 0 < alpha < 1
~~~

Use alpha from a PWM profile or a separately bounded redundant-channel policy. The median removes isolated spikes; the IIR adds damping. A missing/old LSM sample sets REDUNDANT_STALE; do not publish the same old value as a new measurement.

### 7.2 Roll and pitch derivation

For calibrated body acceleration ax, ay, az and the stated convention, static gravity provides:

~~~text
roll phi = atan2(ay, az)
pitch theta = atan2(-ax, sqrt(ay squared + az squared))
~~~

The denominator is the horizontal gravity projection. If it is less than epsilon or any value is non-finite, report TILT_SINGULAR instead of returning NaN. Dynamic acceleration means these are apparent tilt estimates, which is why this channel is a low-rate reference.

### 7.3 Tilt-compensated heading

For corrected magnetometer mx,my,mz, rotate field into the horizontal plane after roll phi and pitch theta:

~~~text
X_h = mx*cos(theta) + mz*sin(theta)
Y_h = mx*sin(phi)*sin(theta) + my*cos(phi) - mz*sin(phi)*cos(theta)
yaw psi = wrap_pi(atan2(-Y_h, X_h) + magnetic_declination_rad)
~~~

This form is convention dependent. Validate signs, axis order, declination sign and quaternion convention against a known compass bearing and multi-axis orientation data. If sqrt(X_h squared + Y_h squared) is below epsilon, heading is invalid because no reliable horizontal field remains.

### 7.4 Primary versus redundant comparison

Use wrapped differences delta_roll = wrap_pi(roll_primary-roll_redundant), and likewise delta_pitch and delta_yaw. Compare only samples within a defined time-alignment window and only when both valid bits are set. Suggested initial UI diagnostics, not safety limits, are warning above 5 deg sustained 500 ms and critical above 15 deg sustained 200 ms. Motion-state-specific thresholds must be calibrated because a damped no-gyro reference intentionally lags fast maneuvers.

## 8. SCBP-V3 Dual-Attitude Contract and End-to-End Data Flow

### 8.1 Existing contract to preserve

The CONFIRMED STM-S3 SCBP-V3 envelope is AA 55 followed by VER, PRIORITY, SRC, DST, MSG_ID little endian, SEQ, FLAGS, LEN little endian, payload and CRC16-MODBUS little endian. CRC covers VER through payload. Existing ATTITUDE is MSG_ID 0x0201 with exactly 30 payload bytes. The independent App BLE envelope is AA, 01, TYPE, LEN little endian, payload, CRC16-MODBUS, 55. Do not merge the envelopes by name or by their shared CRC.

### 8.2 Proposed schema-gated 0x0201 dual layout

This table is PLANNED and incompatible with current 30-byte decoders. The safer deployment is a new MSG_ID such as 0x0205 DUAL_ATTITUDE. If the project explicitly retains 0x0201, every endpoint must gate on LEN=80 and schema=2, preserve legacy LEN=30 decoding, and reject every other length.

| Offset | Field | Width | Unit / meaning |
| ---: | --- | ---: | --- |
| 0 | schema | u8 | 2 for this dual payload |
| 1 | flags | u8 | bit0 primary valid, bit1 redundant valid, bit2 mag valid, bit3 bias converged, bit4 accel gated, bit5 mag gated, bit6 stale, bit7 fault |
| 2 | reserved | u16 LE | zero |
| 4 | timestamp_ms | u32 LE | monotonic STM output time |
| 8 | sample_seq | u32 LE | primary output sequence |
| 12 | primary roll, pitch, yaw | 3 x f32 LE | radians |
| 24 | primary quaternion w,x,y,z | 4 x f32 LE | scalar-first unit quaternion |
| 40 | redundant roll, pitch, yaw | 3 x f32 LE | radians |
| 52 | redundant quaternion w,x,y,z | 4 x f32 LE | scalar-first unit quaternion derived from the tilt solution |
| 68 | delta roll, pitch, yaw | 3 x f32 LE | wrapped primary minus redundant radians |
| 80 | end | - | exact payload length |

Do not transmit duplicate degrees in the new dual payload unless a bandwidth/compatibility requirement proves it necessary. The App can derive degrees from radians. NaN, infinity, a non-unit quaternion beyond a declared tolerance, unknown schema, bad length and timestamp regression are decoder rejects.

### 8.3 S3 forwarding and BLE notify mapping

The S3 bridge validates SCBP-V3 version, priority/flags, source/destination, message ID, length, schema, CRC and sequence status. It then rebuilds only the App BLE envelope and forwards the 80 payload bytes unchanged to FFE2 notify. FFE3 remains the bounded log path. Counters required at the bridge are dual_len_reject, dual_schema_reject, dual_crc_reject, dual_seq_gap, dual_duplicate, dual_notify_drop and dual_ble_not_ready. These counters prove only parser/bridge behavior until UART and BLE capture prove physical delivery.

## 9. macOS App Presentation and Diagnostic Design

### 9.1 Data model and update cadence

Extend the existing telemetry model with PrimaryAttitude, RedundantAttitude and DualAttitudeHealth. The BLE receive pipeline continues decoding on its serial queue and delivers immutable state through MainActor. Store last source timestamp, host receipt time, sequence and validity for both estimators. A 20-50 Hz SwiftUI presentation timer may publish the latest state; a 3D model may interpolate valid time-ordered primary quaternions with shortest-arc slerp. It must not extrapolate indefinitely.

### 9.2 UI surfaces

| Surface | Required state | Behavior |
| --- | --- | --- |
| Primary 3D attitude | Primary quaternion, RPY, age, rate, K_a/K_m gate and bias state | High-frequency interpolation from valid timestamped quaternions. Freeze and mark stale on timeout. |
| Redundant instrument | Damped roll/pitch, tilt-compensated yaw, field norm, age | Visually separate static reference; do not imply it follows fast dynamics. |
| Delta monitor | Wrapped roll/pitch/yaw deltas, persistence and severity | Show warning/critical/stale separately from motor safety. |
| Engineering diagnostics | ODR config/measured, profile ID, RMS, cutoff/notch, drops, CRC, seq gap, latency | Developer-only scanable source for tuning and fault triage. |

Suggested initial display policy is stale above 500 ms for diagnostics and a hard telemetry timeout above 3 s consistent with the present App convention. App thresholds are not motor-control thresholds. The App shall display raw source-validity bits instead of inventing a healthy state from a recent UI update.

### 9.3 Swift pseudo-code

~~~swift
struct DualAttitude: Equatable {
    let timestampMs: UInt32
    let sequence: UInt32
    let flags: UInt8
    let primary: QuaternionAndEuler
    let redundant: QuaternionAndEuler
    let deltaRad: SIMD3<Float>
}

func ingestDual(_ bytes: [UInt8], receivedAt: Date) throws {
    guard bytes.count == 80, bytes[0] == 2 else { throw DecodeError.invalidPayload }
    let sample = try decodeLittleEndianDual(bytes)
    guard sample.primary.quaternion.isFiniteUnit, sample.redundant.quaternion.isFiniteUnit else {
        throw DecodeError.invalidPayload
    }
    guard sample.timestampMs >= lastTimestampOrWrapAwareComparison else {
        throw DecodeError.timestampRegression
    }
    pendingDual = sample
    pendingReceivedAt = receivedAt
}
~~~

## 10. Phased Implementation Plan and C99 Pseudo-code

### Phase A: offline evidence and replay

1. Record raw BMI data at all four configured ODRs with timestamps, bus errors, ring/FIFO state and task timing.
2. Sweep radar PWM with synchronized BMI and LSM data. Compute per-axis RMS, power spectral density, peak frequencies, Q and sample quality.
3. Perform static and multi-orientation LSM magnetometer collection; fit and cross-validate hard/soft-iron calibration.
4. Build a host replay harness comparing Mahony, Madgwick, VQF and a reduced MEKF with identical units, axis mapping and gates. Report RMS attitude difference, latency, convergence, reject time and CPU time.

### Phase B: STM32 primary telemetry-only integration

Create a new planned module STM32H757/Middleware/Attitude/DualAHRS. Keep existing attitude.c, LSM path, calibration state machine, UART contract and motion authority behavior unchanged. The new module receives ordered samples and publishes a latest-value snapshot but does not become a control source.

~~~c
typedef struct {
    float q[4];
    float rpy[3];
    float gyro_bias[3];
    uint32_t timestamp_ms;
    uint32_t sequence;
    uint8_t valid;
    uint8_t accel_gated;
    uint8_t mag_gated;
    uint8_t bias_converged;
} primary_state_t;

void primary_update(const bmi_sample_t *sample, const lsm_mag_t *mag, float pwm) {
    float dt = checked_dt(sample->timestamp_us, state.last_us);
    vec3 a = align_and_bias_accel(sample->accel);
    vec3 w = align_and_bias_gyro(sample->gyro);
    const profile_t profile = profile_lookup(pwm, sample->timestamp_us);
    a = notch_then_lpf(a, profile);
    w = notch_then_lpf(w, profile);
    if (decimator_push(&state.decim, a, w, dt)) {
        float ka = gravity_confidence(state.decim.accel, profile);
        float km = mag_confidence(mag);
        vqf_update_gyr(&state.vqf, state.decim.gyro);
        vqf_update_acc_gated(&state.vqf, state.decim.accel, ka);
        if (mag_is_fresh_and_valid(mag)) vqf_update_mag_gated(&state.vqf, corrected_mag(mag), km);
        primary_publish(&state, ka, km);
    }
}
~~~

### Phase C: redundant channel and health contract

~~~c
void redundant_update(vec3 a_raw, vec3 m_raw, uint64_t timestamp_us) {
    vec3 a = median5_iir(lsm_align_accel(a_raw), &red.accel_filter);
    vec3 m = median5_iir(soft_iron_correct(m_raw), &red.mag_filter);
    red.roll = atan2f(a.y, a.z);
    red.pitch = atan2f(-a.x, hypotf(a.y, a.z));
    float xh = m.x*cosf(red.pitch) + m.z*sinf(red.pitch);
    float yh = m.x*sinf(red.roll)*sinf(red.pitch) + m.y*cosf(red.roll) - m.z*sinf(red.roll)*cosf(red.pitch);
    red.yaw = wrap_pi(atan2f(-yh, xh) + declination_rad);
    red.valid = finite_and_non_singular(a, m, xh, yh);
    redundant_publish(&red, timestamp_us);
}
~~~

### Phase D: protocol, S3 and App rollout

1. Add C golden vectors for SCBP-V3 fragmenting, schema 2 and legacy 30-byte attitude decoding.
2. Add S3 bridge tests for wrong length/schema, CRC failure, sequence gap/duplicate, notify-not-ready and exact unchanged payload re-enveloping.
3. Add Swift parser tests for 80-byte decoding, little-endian floats, non-finite values, quaternion validation, wrapping and stale timestamps.
4. Enable the new payload only after every endpoint advertises accepted schema/version. Otherwise retain 30-byte ATTITUDE and report dual capability unavailable.

### Phase E: controlled bench acceptance

Acceptance order is host math/replay, static build, firmware timing, electrical sensor identity, UART capture, BLE capture, stationary calibration, PWM sweep, controlled motion bench and only then a safety-reviewed vehicle test. Build success, logs or source symbols alone do not prove the next layer.

## 11. Verification Matrix, Risks and Non-goals

| Check | Required method | Passing evidence | Present status |
| --- | --- | --- | --- |
| BMI configured versus measured ODR | timestamped raw capture | >=90 percent rate with bounded jitter and accounted gaps | UNVERIFIED |
| SPI/DMA/RTOS budget | logic analyzer plus task counters | bounded service time, no overflow, adequate stack margin | UNVERIFIED |
| Filter/notch response | sine/chirp plus PWM bench sweep | measured gain, phase and notch depth match selected coefficients | UNVERIFIED |
| Bias convergence | stationary/rest-motion replay and bench | bounded bias, no wind-up, repeatable restart | UNVERIFIED |
| LSM mag calibration | orientation-rich cross-validation | positive-definite fit, accepted residual and norm stability | UNVERIFIED |
| Dynamic gates | acceleration/PWM replay | accel/mag correction rejects contamination and recovers smoothly | UNVERIFIED |
| Wire contract | C/S3/Swift golden vectors | exact length, endian, CRC and schema behavior | PLANNED |
| BLE relay | UART and BLE capture | notification payload matches validated V3 payload | UNVERIFIED |
| App UI | staged app runtime test | independent panes, freshness and no main-thread blocking | UNVERIFIED |
| Safety | fault injection and controlled bench | STM local policy stays authoritative | UNVERIFIED |

Principal risks are: treating configured ODR as measured ODR; anti-aliasing failure caused by newest-only ring consumption; radar/motor magnetic interference; quaternion convention mismatch across C/Swift; mixed 30-byte and 80-byte decoder acceptance; unbounded bias integration; and incorrectly promoting build or log evidence to physical/vehicle acceptance.

Non-goals of this report are changing existing source, CubeMX, hardware, SCBP-V3 bytes, BLE UUIDs, calibration semantics or safety ownership. Those require a separately authorized implementation task.

## 12. Project Source Map and Research Traceability

Workspace material inspected for current facts: README.md; .codex/BOOT.md; .codex/RULES.md; PROJECT_STATUS.md; docs/architecture/system.md; docs/architecture/communication.md; docs/imu/imu-pipeline.md; docs/imu/bmi323.md; docs/imu/lsm303.md; docs/imu/filter.md; docs/imu/attitude.md; docs/imu/calibration.md; DOCS/protocol/SCBP_V3_REFERENCE.md; STM32H757/Middleware/Sensor/BMI323; STM32H757/Middleware/Sensor/imu_manager.c; STM32H757/Middleware/Filter/imu_filter.c; STM32H757/Middleware/Attitude/attitude.c; ESPS3/components/smartcar_protocol; ESPS3/components/smartcar_service; ESPS3/components/s3_ble; IOS_APP/SmartCar_Control_MAC Model/VehicleState.swift, Model/SmartCarProtocol.swift and Stores/TelemetryStore.swift.

No existing source, build configuration, device image or Git history was modified to create this report.

