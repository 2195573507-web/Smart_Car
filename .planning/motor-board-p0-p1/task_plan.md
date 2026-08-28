# MotorBoard P0/P1 Implementation Plan

## Goal

Implement the approved MotorBoard P0/P1 reliability design without changing
the existing motor parameters, wheel mapping, encoder signs, S3 protocol, or
P2 scheduling/resource work.

## Phases

- [x] Phase 1: Review current code, official protocol, and user `$read_flash#` capture
- [x] Phase 2: Write and approve design specification
- [x] Phase 3: Add strict read_flash data model and protocol send/parse support
- [x] Phase 4: Add MotorBoard LOCKED/CONFIGURING/WAIT_FEEDBACK/READY/FAULT gating
- [x] Phase 5: Add 200 ms MSPD watchdog, recovery, and battery freshness
- [x] Phase 6: Add focused host tests or deterministic parser checks
- [x] Phase 7: Build canonical CM7 Debug target and inspect diff/evidence

## Fixed Constraints

- Motor: 520, encoder, magnetic line 11, gear ratio 30, wheel diameter 65 mm, 12 V.
- Motor-board deadzone: 1600, already measured and persisted.
- Order: M1=RR, M2=RF, M3=LR, M4=LF.
- Encoder sign: `{1, -1, 1, 1}` applied once to feedback only.
- UART: USART6 PC6/PC7, 115200 8N1.
- P0 MSPD timeout: 200 ms.
- Normal boot must not send `$MPID#` or `$flash_reset#`.
- P2 is explicitly out of scope.

## Verification Gates

- `read_flash:OK!` must not terminate the configuration collection.
- Complete matching fields must skip all persistent configuration writes.
- A missing/invalid field must not unlock motion.
- Nonzero target is accepted only after two valid, normally timed MSPD frames.
- MSPD timeout clears target/PID/Ramp, queues zero PWM, and does not restore the old target.
- Battery data expires instead of being reported indefinitely.
- Use `/Users/zhiqin/Projects/Smart_Car/STM32H757/CM7/build/Debug` for CM7 output.

## Completion Evidence

- The host protocol test accepts the user-captured multiline `read_flash`
  response, rejects incomplete/invalid snapshots, and verifies `$deadzone:1600#`.
- `cmake --build build/Debug --target Smart_Car_H757_CM7 -j2` completed with
  no compiler diagnostics; the generated ELF remains under the canonical
  `build/Debug` directory.
- USART6 capture and vehicle testing remain required to prove the board's real
  ACK wording, no-write matching path, feedback timing, and physical stopping.
