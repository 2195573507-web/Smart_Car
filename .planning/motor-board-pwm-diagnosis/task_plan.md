# MotorBoard PWM Diagnosis

## Goal

Correct the MotorBoard logical-to-physical four-wheel boundary if confirmed by
the board guide, add bounded 200 ms PID/PWM diagnostics, and complete a CM7
clean build without disturbing unrelated worktree changes.

## Phases

- [x] Analyze existing MotorBoard task, PID, protocol, and hardware mapping.
- [x] Implement explicit feedback/PWM mapping and throttled diagnostics.
- [x] Review the diff and run static checks.
- [x] Run the CM7 clean build and record evidence.
- [x] Report source findings separately from hardware acceptance.

## Design Boundary

The SCBP and chassis contract remains logical `RR,RF,LR,LF`. The MotorBoard
UART boundary is treated as physical `M1=LF,M2=LR,M3=RF,M4=RR`, based on the
four-way board guide. Feedback is mapped to logical order before sign correction
and PID; PID output is mapped back to physical order before `SendPwm()`.

## Verification

`cmake --preset Debug` and
`cmake --build build/Debug --target Smart_Car_H757_CM7 --clean-first -j2`
completed successfully on 2026-08-22. `git diff --check` is clean. Hardware
direction, encoder signs, and vehicle yaw behavior remain unverified.

The final link reported FLASH `184396 B / 1 MB` (17.59%), RAM `60032 B /
128 KB` (45.80%), and no compiler warnings or errors.
