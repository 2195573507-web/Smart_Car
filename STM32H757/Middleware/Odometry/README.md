# Odometry Middleware

`chassis_odometry.c` is the allocation-free estimator used by the CM7
chassis-state publisher. It consumes corrected MotorBoard MSPD values in the
fixed order `[RR, RF, LR, LF]`, plus fresh Primary DualAHRS yaw and the MSPD
arrival timestamp. `Application/Chassis/chassis_state_payload.c` supplies the
RTOS-free 24-byte serializer and the repeated-sequence consume gate shared by
the task and host tests.

The estimator averages the four wheel speeds to obtain center velocity,
integrates source-time displacement, projects it with Primary yaw, and retains
`x_mm`, `y_mm`, `yaw_rad`, and absolute distance in metres. The first sample,
the first sample after explicit invalidation, and any interval over 200 ms do
not integrate distance.

The module has no RTOS, HAL, transport, logging, or allocation dependency.
An invalid or over-age timestamp interval clears the integration anchor; the
next fresh sample is anchored without integrating across the gap.
`Application/Chassis/chassis_state_task.c` owns scheduling and freshness checks,
while the payload helper owns SRP serialization and publication input. Source/build verification does not
establish physical MSPD scale, yaw convention, slip performance, or map
quality.
