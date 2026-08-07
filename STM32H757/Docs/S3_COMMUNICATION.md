# S3 Communication Boundary

`Middleware/Communication/S3_Link/` is the only STM32-side boundary for the
ESP32-S3 link. It defines interfaces only; transport framing, serialization,
checksums, retries, and scheduling are not implemented in this initialization.

## S3 to STM32

- Speed control
- Direction control
- Mode selection
- Parameter settings

## STM32 to S3

- Motor status
- Encoder speed
- Odometry status
- IMU status
- System status

The link carries control and state information, not external sensing data.
Wi-Fi, phone App, ROS2 gateway logic, and external sensing remain in the
ESP32-S3 project.
