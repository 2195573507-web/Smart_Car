# STM32H757 CubeMX and Build Validation

## Scope

This procedure validates only the STM32H757 base project at `STM32H757/Smart_Car_H757.ioc`. It excludes flashing, debugging on a board, motor movement, encoder stimuli, sensor identity checks, GPS reception, and ESP32 communication.

## Required IOC Baseline

- Device: `STM32H757XIH6`.
- Clock intent: CM7 480 MHz and CM4 240 MHz.
- Enabled categories: GPIO, BMI323 SPI, LSM303 I2C, and the valid GPS UART. The requested STM communication UART remains blocked until its `PD3`/`PD4` hardware route is changed or an authorized interface alternative is supplied.
- No PWM output pin is supplied, so do not enable an unconnected PWM output merely to satisfy a timer reservation.
- Explicitly avoid unrelated peripheral enablement.
- Configure only GPIO direction and basic peripheral resources; do not add task/application logic.

## Verification Procedure

1. Open `Smart_Car_H757.ioc` in a compatible STM32CubeMX release and confirm successful parse without an IOC recovery dialog.
2. Review the Pinout and Configuration view. Resolve every red pin conflict and verify the required pins in [STM32_PIN_MAP.md](STM32_PIN_MAP.md).
3. Record `PD3`/`PD4` as a blocking communication-route conflict rather than enabling a false UART TX/RX pair. Confirm the authorized choice for `PA13` versus SWDIO before generation.
4. Verify the Clock Configuration view reports the intended CM7 480 MHz and CM4 240 MHz targets.
5. Generate the selected base project without overwriting hand-maintained application work outside the authorized initialization scope.
6. Perform a clean build using the generated project/toolchain and record the exact command/tool version and separate CM7/CM4 results.
7. Preserve tool warnings and failures. A failed or omitted step must remain reported as such.

## Acceptance Matrix

| Check | Passing evidence | Not established by pass |
| --- | --- | --- |
| IOC opens | CubeMX parse succeeds | Generated source and hardware operation |
| Pinout clean | CubeMX reports no pin conflict | Schematic or physical wire correctness |
| Clock view | CubeMX clock tree shows target values | Measured clocks, CM4 boot, timing behavior |
| Generation | Project generator completes | Successful compile/link or target boot |
| Build | Compiler and linker exit successfully | Flash, peripheral traffic, or vehicle behavior |

## Reporting Template

Record: CubeMX version, generator/toolchain, IOC parse result, pin-conflict result, clock-view result, CM7 build result, CM4 build result, warnings, and remaining runtime validation. Do not convert an unavailable local CubeMX installation or a missing compiler into a passing result.
