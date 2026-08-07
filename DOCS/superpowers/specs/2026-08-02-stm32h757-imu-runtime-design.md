# STM32H757 IMU Runtime Log Integration Design

## Scope and constraints

This change integrates the existing BMI323, LSM303, and IMU Manager into the
CM7 runtime for board validation. The CH340 debug adapter is connected to
USART1: PA9 is USART1_TX, PA10 is USART1_RX, and the line format is 115200
8N1. USART6 remains reserved for GPS. No new UART instance is introduced.

BMI323 and LSM303 register transactions, scaling, identity checks, and bus
drivers remain unchanged. Motion, Chassis, Safety, CubeMX SPI/I2C resources,
and attitude-fusion behavior are outside this change.

## Runtime architecture

The CM7 startup sequence owns hardware initialization, debug UART readiness,
sensor initialization, task creation, and scheduler start:

1. Initialize HAL, clocks, GPIO, SPI/I2C, and USART1.
2. Print `SMART CAR H757 BOOT` and `USART1 DEBUG READY` through the existing
   BSP log entry point.
3. Print `IMU INIT START`, call `imu_init()`, and print per-device success or
   the returned failure reason. A failure never stops startup.
4. Create `imu_task` and `imu_debug_task`, print `IMU TASK START`, and start
   the CM7 FreeRTOS scheduler.

The existing BSP UART abstraction is the only application logging transport.
`BSP_UART_USART1` resolves to the generated `huart1` handle. No task writes
directly to HAL UART APIs.

## Tasks and data flow

`imu_task` runs at 100 Hz using `vTaskDelayUntil()`. It calls the manager's
update iteration only and never prints. It maintains the manager's update
counter and last successful sample timestamp through the manager statistics
interface. If the manager is not ready or an update fails, the task continues
running and retries initialization at a bounded one-second interval.

`imu_debug_task` runs at 1 Hz. It takes one coherent `IMU_Data_t` snapshot via
`imu_get_data()`, reads the device readiness flags, and formats one bounded
status block. It sends the block through `uart_log_write()` after the 100 Hz
task's update window, so diagnostic output cannot run from the sampling task.
If no coherent sample is available, the task prints zeroed values and the
current readiness/error state rather than blocking.

The status block uses this fixed field order:

```text
========== IMU STATUS ==========
BMI323 online: OK/FAIL
ax: <value>
ay: <value>
az: <value>
gx: <value>
gy: <value>
gz: <value>

LSM303 online: OK/FAIL
mx: <value>
my: <value>
mz: <value>

update_count: <value>
===============================
```

The manager exposes read-only statistics for `update_count` and
`last_update_ms`; the statistics are updated only when a complete sample is
published. The existing coherent snapshot mechanism remains the ownership
boundary between the sampling and debug tasks.

## Error handling and recovery

Startup records a non-OK `imu_init()` result, including the numeric BSP status,
and immediately continues to task creation. `imu_task` does not busy-loop: it
keeps its 10 ms cadence and attempts `imu_init()` no more than once per second
while the manager is not ready. A successful retry emits the same per-device
success state through the 1 Hz debug block and resumes normal publication. The
100 Hz task keeps retries silent; a failed retry leaves both tasks alive for a
later recovery while the debug task reports the returned status.

The debug task reports each sensor's current readiness independently. UART
transmit failures do not terminate either task. No watchdog, global delay, or
priority change is used to mask a sensor or serial failure.

## FreeRTOS integration

The CM7 target receives the project-local FreeRTOS kernel sources and the ARM
Cortex-M7 GCC port under the RTOS integration boundary, plus a CM7
`FreeRTOSConfig.h`. The build includes the kernel, heap implementation, and
port without changing sensor drivers. `main()` creates the two static-priority
tasks before `vTaskStartScheduler()`; task priorities keep `imu_task` above
`imu_debug_task`, while the latter remains at normal application priority.

## Verification

Static verification checks that USART1 is configured on PA9/PA10 at 115200 8N1,
USART6 is not used by the debug logger, both task entry points are created, and
the fixed log labels are present. Build verification runs a clean CM7 CMake
configure followed by Ninja and confirms
`STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf` exists. Hardware acceptance
is separate: flash the CM7 image, connect CH340 at 115200 8N1 with crossed TX/RX
and common ground, reset the board, and capture the startup and 1 Hz status
blocks. A build does not prove sensor identity or live SPI/I2C traffic.
