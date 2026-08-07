# IMU Driver Guide

This document defines the STM32H757 IMU driver boundary for the BMI323
inertial sensor and the LSM303 accelerometer/magnetometer. The implementation
belongs under `Drivers/IMU`; the shared sample service belongs under
`Middleware/Sensor`. Device transactions use the BSP interfaces, while the
CM7 startup path remains responsible for CubeMX peripheral setup. The
FreeRTOS task is the only periodic bus owner; consumers use the manager
snapshot API.

## Devices

### BMI323

BMI323 supplies the accelerometer, gyroscope, and temperature sample used by
the unified IMU service. The driver startup sequence is:

1. initialize the SPI and GPIO BSP interfaces;
2. read and validate `CHIP_ID` (`0x00`, nominal BMI323 value `0x43`);
3. issue the BMI323 software reset and allow the device reset time to elapse;
4. configure the accelerometer and gyroscope ranges/output data rates;
5. enable data updates before the first sample is published.

The current driver configures +/-4 g acceleration and +/-2000 dps gyro output
and converts samples to SI units. Consumers must use the converted
floating-point values returned by the driver rather than interpreting raw
register codes. `bmi323_get_chip_id()` exposes the validated ID for startup
diagnostics.

### LSM303

LSM303 supplies the magnetic-field sample (the manager keeps BMI323 as the
unified acceleration source). The driver reads and validates the
accelerometer `WHO_AM_I` (`0x33`) and the three-byte magnetometer identity
(`0x48, 0x34, 0x33`) before enabling both data paths. LSM303 modules exist in
several compatible variants; the board BOM/module must determine whether
these LSM303DLHC-compatible IDs and addresses apply. Do not treat a read of
`0x00` or `0xff` as a valid identity. `lsm303_get_accel_id()` and
`lsm303_get_mag_id()` expose the validated values.

## Bus and pin configuration

The CubeMX IOC and BSP own peripheral initialization. The driver only calls
the corresponding BSP functions after generated system/peripheral startup.

| Device | Bus | Signals | Chip select/address |
| --- | --- | --- | --- |
| BMI323 | SPI1 | SCK `PA5`, MISO/SDO `PA6`, MOSI/SDI `PA7` | CS `PC4` (`BSP_GPIO_BMI323_CS`) |
| BMI323 | GPIO | interrupt input `PB2` | `BSP_GPIO_BMI323_INT1` |
| LSM303 | I2C4 | SCL `PD12`, SDA `PD13` | 7-bit address supplied by the selected module |
| LSM303 | GPIO | interrupt input `PC2` | board net; no BSP live interrupt enum is currently recorded |

SPI transactions use the BSP's SPI1 mode-0, 8-bit interface and software CS.
BMI323 reads account for the command and dummy bytes required by its SPI
protocol. I2C transactions pass 7-bit addresses to the BSP; address shifting
to the HAL representation is handled inside `bsp_i2c_*`. Bus calls are
blocking and must propagate non-`BSP_STATUS_OK` results to the caller.

## Public API

### BMI323 driver

Header: `Drivers/IMU/BMI323/bmi323.h`

```c
typedef struct {
    float x;
    float y;
    float z;
} Vector3f;

bsp_status_t bmi323_init(void);
bsp_status_t bmi323_get_chip_id(uint8_t *chip_id);
bsp_status_t bmi323_read_acc(Vector3f *acc);
bsp_status_t bmi323_read_gyro(Vector3f *gyro);
bsp_status_t bmi323_read_temperature(float *temperature);
uint8_t bmi323_is_ready(void);
```

`bmi323_init()` performs identity verification, reset, sensor configuration,
and data-update enablement. The read functions return converted SI samples:
acceleration in `m/s^2`, angular rate in `rad/s`, and temperature in degrees
Celsius. A failed transaction leaves the caller's output unchanged where the
implementation can guarantee that behavior.

### LSM303 driver

Header: `Drivers/IMU/LSM303/lsm303.h`

```c
bsp_status_t lsm303_init(void);
bsp_status_t lsm303_get_accel_id(uint8_t *id);
bsp_status_t lsm303_get_mag_id(uint8_t id[3]);
bsp_status_t lsm303_read_acc(Vector3f *acc);
bsp_status_t lsm303_read_mag(Vector3f *mag);
uint8_t lsm303_is_ready(void);
```

The LSM303 acceleration output is in `m/s^2`; the magnetic output is in
microtesla (`uT`). The selected LSM303 variant, addresses, scales, and
identity values remain in the driver layer.

### Unified sensor manager

Headers: `Middleware/Sensor/imu_manager.h` and `imu_manager.c`

```c
typedef struct {
    Vector3f acc;
    Vector3f gyro;
    Vector3f mag;
    float temperature;
    uint32_t timestamp;
} IMU_Data_t;

bsp_status_t imu_init(void);
bsp_status_t imu_update(void);
bsp_status_t imu_get_data(IMU_Data_t *data);
```

`imu_update()` is the single publication point for a coherent sample. The
manager's timestamp is the monotonic millisecond value returned by
`timer_get_ms()`, captured for the published sample. `imu_get_data()` uses a
sequence counter to provide a consistent snapshot when called from another
task. The manager owns the synchronization mechanism and returns
`BSP_STATUS_TIMEOUT` if a stable snapshot cannot be obtained.

## 100 Hz task and logging

The sensor task owns periodic sampling and runs on CM7 at 100 Hz:

```c
for (;;) {
    (void)imu_update();
    vTaskDelay(pdMS_TO_TICKS(10));
}
```

Only this task should perform periodic BMI323/LSM303 reads. Other tasks read
the latest `IMU_Data_t` through `imu_get_data()` and must not access the buses
or driver state directly. `imu_task_step()` is provided for a scheduler or
test harness that owns the loop. Initialization and update failures should
be reported without spinning in a tight retry loop.

Startup diagnostics use the `System/Log` boundary (`uart_log_write` or the
project's logger wrapper), not direct `printf`. The expected field order is:

```text
BMI323 ID: <value>
ACC: <x> <y> <z>
GYRO: <x> <y> <z>

LSM303 ID: <value(s)>
ACC: <x> <y> <z>
MAG: <x> <y> <z>
```

The logger must not run from an ISR. Keep messages bounded and serialize
output through the System/Log owner if more than one task can emit them.

## Test and acceptance procedure

### Compile-time checks

From the STM32H757 project, configure and build the CM7 target with the
project's existing CMake preset. Confirm that the new BMI323, LSM303, and
`Middleware/Sensor` sources are part of that target. CM4 is outside this
driver scope. A successful link only proves source/API integration; it does
not prove sensor identity or live bus traffic.

### Board checks

With the target board and both sensors connected:

1. boot CM7 and capture the startup log;
2. verify the BMI323 `CHIP_ID` is the expected value for the fitted part;
3. verify every configured LSM303 `WHO_AM_I` value matches the fitted variant;
4. observe non-constant acceleration, gyro, and magnetic samples while moving
   the board;
5. confirm timestamp progression and approximately 10 ms update intervals;
6. leave the 100 Hz task running long enough to check for bus errors, data
   races, and task starvation.

The existing BSP compile-only smoke test proves API/link compatibility only.
It does not prove sensor identity, live SPI/I2C communication, data refresh,
or long-duration FreeRTOS behavior. Those claims require recorded hardware
evidence. No attitude fusion, EKF, PID, or motor-control behavior is part of
this driver acceptance.

## Current implementation status

The source boundary is implemented, but board-level acceptance remains
pending. `CM7/CMakeLists.txt` lists the two device-driver sources and
`Middleware/Sensor/imu_manager.c`; CubeMX IOC files are not changed by this
work. The CM7 ELF build is verified, but until a board log records valid IDs
and changing samples, report the driver as **build-verified /
hardware-unverified**.

## Ownership and change boundary

The protected BSP, Application, Motion, Chassis, Safety, and CubeMX IOC files
remain unchanged by this work. Sensor register definitions, scaling,
transaction sequencing, and synchronization belong to `Drivers/IMU` and
`Middleware/Sensor`; higher layers consume only the manager API.
