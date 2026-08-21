# Smart Car PID Online Tuning Design

Status: **READY FOR USER REVIEW**

Date: 2026-08-21

## 1. Scope and Ownership

This change adds one runtime configuration transaction and a macOS operator
surface. It does not change GPIO, UART ownership, motor-board protocol bytes,
the SCBP frame envelope, or the existing wheel-speed command semantics.

Implementation file boundary:

| Area | Files in scope |
| --- | --- |
| Shared protocol | `Common/SCBP_CAN/include/scbp_protocol_defs.h`, `Common/SCBP_CAN/scbp_wire.c`, `Common/SCBP_CAN/tests/test_scbp_can.c` |
| CM7 PID/API | `STM32H757/CM7/Core/Inc/pid_controller.h`, `STM32H757/CM7/Core/Src/pid_controller.c`, `STM32H757/Middleware/MotorBoard/motor_board_task.h`, `STM32H757/Middleware/MotorBoard/motor_board_task.c`, `STM32H757/Middleware/Communication/Services/s3_service.c` |
| S3 bridge | `ESPS3/components/smartcar_protocol/include/app_parser.h`, `ESPS3/components/smartcar_service/command_bridge.c` |
| macOS App | `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/Model/SmartCarProtocol.swift`, `BLE/BLEManager.swift`, `ViewModels/SmartCarViewModel.swift`, `UI/WheelSpeedControlCard.swift`, new `UI/PIDTuningCard.swift`, and focused state/model tests if the package test target is introduced |
| Canonical docs | `docs/protocol/protocol.md`, `docs/protocol/app-ble-protocol-v1.md`, `docs/protocol/stm32-s3-command-reference.md`, affected App/CM7/S3 module notes |

Ownership remains layered:

| Layer | Responsibility |
| --- | --- |
| macOS App | Validate/edit parameters, display four-wheel telemetry, send App BLE `TYPE 0x1D` on Apply |
| ESP32-S3 | Validate App frame, translate it into SCBP-CAN `0x111`, correlate ACK, return App ACK |
| STM32H757 S3 service | Validate SCBP source/destination/length/values, invoke MotorBoard update, send fast ACK |
| MotorBoard task | Own the four PID/Ramp instances and apply one coherent runtime update |
| Shared SCBP layer | Define ID/length and explicit float32 little-endian wire helpers |

The PID configuration is global: one tuple is applied to all M1 through M4.

## 2. Protocol Design

### 2.1 SCBP-CAN command

Add:

```c
#define SCBP_MSG_ID_PID_PARAMS_CMD UINT16_C(0x111)
#define SCBP_PAYLOAD_PID_PARAMS_SIZE UINT16_C(16)
```

Direction is S3 to STM32H757. The frame uses realtime priority and
`SCBP_CAN_FLAG_ACK_REQUIRED`. Payload fields are fixed and have no padding:

| Offset | Field | Encoding | Units |
| ---: | --- | --- | --- |
| 0 | `kp` | f32 LE | dimensionless |
| 4 | `ki` | f32 LE | 1/s-equivalent controller gain |
| 8 | `kd` | f32 LE | controller gain |
| 12 | `max_accel` | f32 LE | mm/s^2 |

The existing `scbp_wire_write/read_f32_le` helpers and four-element array
helpers remain the only serialization path. No packed C struct is used for
the command payload.

### 2.2 App BLE command

Add App frame type `0x1D` named `PID_PARAMS_CMD`, with the same 16-byte payload
and field order. The App frame remains `AA 01 TYPE LEN payload CRC 55`; S3
re-envelopes the validated payload into a new SCBP-CAN frame.

The App ACK remains type `0x06` with `{acknowledged_type_u8, result_u8}`.
`Apply` reports success only after the S3 receives an STM ACK with status
`SCBP_FAST_RESP_OK`. Invalid App data or an STM rejection maps to the existing
rejected result.

## 3. Firmware Design

### 3.1 PID API

Extend `pid_controller.h/.c` with:

```c
void PID_Update_Gains(PID_Controller_t *pid, float kp, float ki, float kd);
void Ramp_Update_Max_Accel(Ramp_Profile_t *ramp, float max_accel);
```

The existing lowercase `pid_controller_t` remains source-compatible through a
typedef alias. The update functions reject null or non-finite input by leaving
the instance unchanged; range admission is performed by the higher-level
MotorBoard update function.

### 3.2 MotorBoard ownership API

Add a public function in `motor_board_task.h/.c`:

```c
bool motor_board_update_pid_params(float kp, float ki, float kd,
                                   float max_accel);
```

It validates all four values against the agreed ranges, enters the same short
critical section used by target updates, updates all four PID controllers and
ramps, and exits only after the complete tuple is committed. It returns false
without changing any wheel when validation fails. It does not reset integrals,
targets, or current ramp positions; a tuning update takes effect on the next
control cycle without a motion discontinuity.

### 3.3 STM service dispatch

Add a `SCBP_MSG_ID_PID_PARAMS_CMD` branch in `s3_service_on_frame()`:

1. Require a non-null payload and exact length 16.
2. Decode four explicit f32 LE values.
3. Require finite values and call `motor_board_update_pid_params()`.
4. Reply with fast ACK OK only after the MotorBoard API returns true; otherwise
   reply with `INVALID_PARAM` and do not alter runtime parameters.
5. Emit bounded diagnostic text:
   `[PID_CONFIG] Updated: Kp=..., Ki=..., Kd=..., Accel=...`.

The branch preserves existing source, destination, bus-off, retry, and ACK
correlation behavior.

## 4. ESP32-S3 Gateway Design

In `command_bridge.c`, extend the App parser dispatch for type `0x1D`:

1. Require exactly 16 bytes.
2. Decode with `scbp_wire_read_f32_array_le`-equivalent scalar checks and
   enforce the shared ranges before enqueueing a transaction.
3. Call `scbp_link_send()` with realtime priority, STM destination, message
   `SCBP_MSG_ID_PID_PARAMS_CMD`, `ACK_REQUIRED`, and the unchanged 16-byte
   payload.
4. Use a callback context identifying App type `0x1D`; return App ACK only
   after the SCBP transaction completes.

The bridge remains bounded by the existing service task and four pending SCBP
transaction slots. It does not allocate per Apply request and does not retry
the App frame independently of the SCBP link manager.

## 5. macOS App Design

### 5.1 Protocol and ViewModel

- Add `FrameType.pidParams = 0x1D` and a BLEManager method that encodes four
  float32 values explicitly as little-endian bytes.
- Add a `PIDTuningState` owned by `SmartCarViewModel` with the four defaults,
  range/step metadata, editable text values, validation state, and last Apply
  result.
- `Apply` is disabled while disconnected or while any field is invalid. It
  sends exactly one frame and does not send on slider/stepper changes.
- `Defaults` restores the four local fields; it does not transmit.
- Numeric fields accept keyboard entry and stepper changes, format on commit,
  and reject non-finite, malformed, or out-of-range values.

### 5.2 Wheel target controls

`WheelSpeedControlCard` contains:

1. A clearly labeled “All wheels” slider that calls `setAllWheelTargets` and
   writes the same target to M1..M4.
2. Four individual target sliders that call `setWheelTarget` and preserve the
   existing 50 ms coalescing and 100 ms heartbeat behavior.
3. A four-lane chart below the controls. Each lane is a fixed-height drawing
   region within a 240..280 pt overall chart area, shares the time axis, and
   displays actual speed plus a semi-transparent horizontal dashed baseline at
   the current target value. Target history is not added to the wire telemetry
   contract in this change.

The chart uses fixed colors: M1 orange, M2 blue, M3 green, M4 purple. Each
lane dynamically derives its own Y range with a minimum floor so low-speed
changes remain visible; the range includes target and actual values with
stable padding and finite-value fallback. Four lanes avoid high-speed wheels
compressing low-speed wheels into a flat line.

### 5.3 PIDTuningCard

The card is collapsible and uses numeric input plus stepper controls:

| Field | Range | Step | Default |
| --- | ---: | ---: | ---: |
| Kp | 0.0..4.0 | 0.05 | 1.10 |
| Ki | 0.0..0.3 | 0.005 | 0.06 |
| Kd | 0.0..0.1 | 0.002 | 0.00 |
| Accel | 200..2000 mm/s^2 | 50 | 800 |

The card shows connection state, validation errors, and the last Apply result
without claiming that the motor output or vehicle motion changed. It uses
standard SwiftUI controls and no additional dependency.

## 6. Error Handling and Safety

- Invalid lengths, non-finite floats, and out-of-range values are rejected at
  App, S3, and STM boundaries.
- A failed validation is atomic: no subset of wheels changes.
- SCBP ACK timeout/retry behavior remains the existing 500 ms and three retry
  policy; App receives rejected status after exhaustion.
- BLE disconnect and existing wheel watchdog behavior are unchanged. PID
  parameters are runtime-only and revert to compile-time defaults on reboot.
- A successful ACK proves parameter admission, not physical motor response.

## 7. Verification Plan

### Host protocol tests

Add golden-byte tests for PID payload encoding/decoding, exact 16-byte length,
non-finite/range rejection, and SCBP `0x111` ACK correlation. Preserve all
existing parser, CRC, retry, and bus-off tests.

### CM7 tests/build

- Unit-test the PID and ramp update functions, including null/non-finite input,
  atomic all-wheel update, and preservation of integral/current target state.
- Run the documented clean CM7 configure/build with warnings treated as errors
  for the changed sources.

### S3 build

- Build the ESP-IDF S3 bridge with the shared SCBP component and verify the App
  `0x1D` path compiles with no warnings.
- Exercise the App parser and bridge callback with valid, malformed, and
  out-of-range 16-byte payloads using host doubles where available.

### App build

- Run `swift build` for the macOS package.
- Add protocol/model tests for little-endian golden bytes, numeric validation,
  defaults, master-slider fan-out, and four-lane history rendering state.

### Evidence boundary

No build or host test is treated as proof of BLE delivery, UART electrical
integrity, flashed firmware behavior, encoder response, or vehicle safety.
