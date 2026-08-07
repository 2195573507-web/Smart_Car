# STM32H757 Architecture

STM32H757 is the real-time motion controller. It owns four motor outputs,
encoder acquisition, IMU acquisition, attitude and odometry estimates,
chassis control, safety policy, and the command/state boundary to ESP32-S3.

The ESP32-S3 owns lidar acquisition, phone connectivity, Wi-Fi, ROS2 gateway
functions, and high-level tasks. No lidar driver, parser, or lidar state is
part of this STM32H757 tree.

## Dependency Direction

```text
CubeMX HAL/Core -> BSP -> Drivers -> Middleware -> Application
                                  \-> System services used by all layers
```

Application modules do not call HAL directly. Drivers consume BSP interfaces;
Middleware does not depend on Application. System services provide execution,
logging, watchdog, and memory policy without owning vehicle behavior.

## Dual-Core Baseline

CM7 is the primary real-time control candidate. CM4 remains an available
execution domain for future partitioning of non-critical work or shared
services. Core placement, HSEM/IPC ownership, and shared-memory contracts are
deferred until the runtime task model is authorized.
