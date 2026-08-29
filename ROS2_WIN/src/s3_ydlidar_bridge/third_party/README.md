# Third-party source inventory

`ydlidar_sdk/` is a scoped extraction from the official YDLidar-SDK archive.
The original `LICENSE.txt` and source copyright headers are retained. The
project-only change is the documented `YDlidarDriver::parseMemoryChannel`
entry point; it only supplies a caller-owned read channel and invokes the
existing official parser methods.
Only parser dependencies are included: unreferenced SDK samples, filters,
math helpers, and passive-socket sources are intentionally omitted.

The official ROS2 driver archive was audited at
`资料/EAI X3&X3 Pro激光雷达/ROS2教程/源码/ydlidar_ros2_driver-master.tar.xz`
(package.xml version 1.0.1, MIT). Its archive `LICENSE.txt` is empty, so no
runtime driver source is copied. The bridge deliberately does not use its
serial-opening node or start/stop/DTR controls.
