# STM32H757 Code Structure

| Layer | Path | Naming | Responsibility |
| --- | --- | --- | --- |
| BSP | `BSP/<bus>` | `bsp_xxx` | Board and peripheral adaptation boundary |
| Drivers | `Drivers/<device>` | `drv_xxx` or device name | Device-level motor, encoder, and IMU abstractions |
| Middleware | `Middleware/<domain>` | `middleware_xxx` | Filtering, attitude, odometry, control, and S3 link contracts |
| Application | `Application/<domain>` | `app_xxx` | Chassis, motion, remote, and safety orchestration |
| System | `System/<service>` | `sys_xxx` | Tasks, logging, watchdog, and memory policy |
| Configuration | `Config` | project configuration | Board, build, and ownership settings |

The existing `Core/`, `CM4/`, `CM7/`, HAL, startup, linker, IOC, and CMake
files remain CubeMX/project infrastructure and are not replaced by this
scaffold.

Every new module has a README. Functional implementation must add tests and an
explicit owner before changing the placeholder interfaces.
