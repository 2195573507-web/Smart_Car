# PA2 to GPIO18 Level Test

Status: forced test firmware. This test isolates one physical direction and
does not initialize UART, SRP, DMA, BLE, radar, motor, or application services.

## Purpose

| Endpoint | Pin | Mode | Behavior |
| --- | --- | --- | --- |
| STM32H757 CM7 | PA2 | GPIO push-pull output, no pull | Starts low, then toggles every 500 ms |
| ESP32-S3 | GPIO18 | GPIO input, no pull, no interrupt | Logs the sampled level every 100 ms |

Connect STM PA2 directly to S3 GPIO18 and connect STM GND to S3 GND. Leave
the normal UART roles disabled by using only these forced test images. Do not
connect another active output to either end of the net; power down or isolate
the radar/other external device if it shares the GPIO18 net.

## Expected Result

The S3 USB Serial/JTAG console prints `GPIO18=1` for about five 100 ms samples,
then `GPIO18=0` for about five samples, repeating at 1 Hz. A stable alternating
sequence demonstrates that PA2 transitions are reaching GPIO18 and that the
boards have a logic-level reference sufficient for this signal.

Software logs alone cannot certify physical continuity or common-ground
resistance with 100 percent certainty. For an electrical sign-off, observe PA2
and GPIO18 simultaneously with a logic analyzer or oscilloscope and measure
the ground connection with a multimeter. A missing or unstable alternation can
be caused by an open wire, missing ground, another driver, incorrect pin route,
or a power-level fault.

## Build

```sh
export PATH="/opt/ST/STM32CubeCLT_1.22.0/GNU-tools-for-STM32/bin:$PATH"
cmake -S STM32H757/CM7 -B STM32H757/CM7/build/Debug \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/STM32H757/gcc-arm-none-eabi.cmake"
cmake --build STM32H757/CM7/build/Debug \
  --target clean
cmake --build STM32H757/CM7/build/Debug \
  --target Smart_Car_H757_CM7 -j2

source /Users/zhiqin/.espressif/v5.5.4/esp-idf/export.sh
cd ESPS3
idf.py -B build-gpio18-level-forced fullclean
idf.py -B build-gpio18-level-forced \
  -D SDKCONFIG=build-gpio18-level-forced/sdkconfig \
  -D SDKCONFIG_DEFAULTS=sdkconfig.defaults build
```

## Flash

Program STM32 CM7 at `0x08000000` with
`STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf`.

From `ESPS3/`, flash the complete S3 bundle with:

```sh
idf.py -B build-gpio18-level-forced -p PORT flash
```

Use only matching artifacts from the GPIO level-test build directories. No
device is flashed by the build procedure.
