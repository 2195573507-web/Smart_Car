# STM32H757 IMU Initialization Diagnostics

## Scope

Add startup-only diagnostics for the existing BMI323 SPI and LSM303 I2C
driver paths. The change is limited to the IMU drivers and their manager
routing. USART1, the SmartCar Logger implementation, FreeRTOS, chassis and
motion code, and the CubeMX IOC remain unchanged. No flash or monitor action
is part of the work.

## Design

Each device keeps its existing no-log initialization API and gains an explicit
diagnostic initialization entry point. `imu_init()` calls the diagnostic
entries synchronously after the existing UART logger is ready. `imu_recover()`
calls the no-log entries, so recovery from the periodic task never emits bus
diagnostics.

BMI323 continues to use SPI1 with PA5 SCK, PA6 MISO, PA7 MOSI, PC4 software
chip select, and PB2 as the input. The bus remains SPI mode 0, 8-bit,
MSB-first. Its prescaler changes from 16 to 32, giving approximately 7.5 MHz
from the configured 240 MHz SPI kernel clock. The diagnostic entry logs SPI
initialization status, CS assertion/deassertion status, the raw CHIP_ID read,
and the returned status through `uart_log_write()`.

LSM303 continues to use the generated I2C4 handle on PD12 SCL and PD13 SDA,
with 7-bit addresses passed through the BSP. Its diagnostic entry logs I2C
initialization, probes every address from `0x00` through `0x7F`, prints each
acknowledged address, and reads/logs the accelerometer identity plus the
three-byte magnetometer identity. A missing accelerometer does not prevent the
magnetometer diagnostic attempt; initialization still returns the existing
failure status when either identity is invalid.

## Error handling

Diagnostic logging is best-effort and never changes a returned `bsp_status_t`.
The existing manager-level initialization failure remains available after the
raw bus diagnostics. No diagnostic code is reachable from the 100 Hz sample
loop except through the no-log recovery path.

## Verification

Run the requested CM7 configure, clean, and Ninja build commands. Build success
proves source and API integration only; live WHO_AM_I values remain hardware
evidence to be captured from the board after a later authorized flash/monitor
run.
