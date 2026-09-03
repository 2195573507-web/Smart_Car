# Odometry Middleware

`chassis_odometry.c` is the allocation-free estimator used by the CM7
chassis-state publisher. It consumes corrected MotorBoard MSPD values in the
fixed order `[RR, RF, LR, LF]`, plus fresh Primary DualAHRS yaw and the MSPD
arrival timestamp.

The estimator averages the four wheel speeds to obtain center velocity,
integrates source-time displacement, projects it with Primary yaw, and retains
`x_mm`, `y_mm`, `yaw_rad`, and absolute distance in metres. The first sample,
the first sample after explicit invalidation, and any interval over 200 ms do
not integrate distance.

The module has no RTOS, HAL, transport, logging, or allocation dependency.
`Application/Chassis/chassis_state_task.c` owns scheduling, freshness checks,
SRP payload serialization, and publication. Source/build verification does not
establish physical MSPD scale, yaw convention, slip performance, or map
quality.
