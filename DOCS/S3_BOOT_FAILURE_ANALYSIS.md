# ESP32-S3 Boot Failure Analysis and Fix

Date: 2026-08-01
Project: `S3-radartest/ESPS3`
Application: `s3_radar_minimal`
Framework: ESP-IDF v5.5.4
Target: ESP32-S3 N32R16 (32 MB OPI flash, 16 MB Octal PSRAM)

## Result

The pre-fix image was built for a different memory configuration than the connected
board. The old configuration selected DIO flash, declared a 2 MB flash device, and
left PSRAM disabled. The target identifies itself through eFuse as an Octal flash
device and has 32 MB flash plus 16 MB Octal PSRAM. That mismatch explains why the ROM
printed `Octal Flash Mode Enabled` but no IDF `cpu_start` or `app_main` messages
followed.

The configuration is now explicit in both `sdkconfig.defaults` and the generated
`sdkconfig`. The final image was rebuilt, flashed, and monitored on
`/dev/cu.usbmodem5B901601171`. The board reached `main_task: Calling app_main()`,
initialized Octal PSRAM with a passing memory test, and continued into the existing
RADAR UART/PWM code. No radar, UART, PWM, BLE, or other business logic was changed.

## Evidence Before the Fix

The previous generated configuration (`S3-radartest/ESPS3/sdkconfig.old`) contained:

| Setting | Previous value | Why it was wrong for N32R16 |
| --- | --- | --- |
| `CONFIG_ESPTOOLPY_FLASHMODE` | `"dio"` | The board uses the eFuse-selected Octal/OPI path. |
| `CONFIG_ESPTOOLPY_FLASHSIZE` | `"2MB"` | The connected flash is 32 MB. |
| `CONFIG_SPIRAM_MODE` | Disabled (`CONFIG_SPIRAM` unset) | The board has 16 MB Octal PSRAM and the image must initialize it. |
| `CONFIG_SPIRAM_TYPE` | No setting | PSRAM type was not selected because PSRAM was disabled. |
| `CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ` | `160` | This value was already suitable and was retained. |
| `CONFIG_BOOTLOADER_LOG_LEVEL` | `3` (`INFO`) | Too little detail for partition/MSPI diagnosis. |

The old image and bootloader were internally valid; this was not evidence of a
truncated write. The old `build/flash_args` explicitly used
`--flash_mode dio --flash_size 2MB`, which did not describe the target hardware.

## Hardware Identification

Read-only `esptool` queries on the connected board reported:

| Query | Result |
| --- | --- |
| Chip | ESP32-S3 QFN56, revision v0.2 |
| Crystal | 40 MHz |
| Embedded PSRAM | 16 MB, AP_1v8 |
| Detected flash size | 32 MB |
| Flash eFuse mode | Octal (8 data lines) |
| Flash device | MXIC, OPI runtime mode |

This matches an ESP32-S3 N32R16 board. The hardware result is independent of the
project configuration and is the reason the project must select OPI/Octal settings.

## Current Configuration

The following settings are now present in `S3-radartest/ESPS3/sdkconfig.defaults` and
resolved in the generated `sdkconfig`:

| Requested check | Current value | Interpretation |
| --- | --- | --- |
| `CONFIG_ESPTOOLPY_FLASHMODE` | `"dout"` | ESP-IDF intentionally emits the DOUT image header/tool mode when `CONFIG_ESPTOOLPY_FLASHMODE_OPI=y` is selected; this is the expected OPI boot image encoding for this IDF version. |
| OPI selector | `CONFIG_ESPTOOLPY_OCT_FLASH=y`, `CONFIG_ESPTOOLPY_FLASHMODE_OPI=y` | Enables the Octal/OPI flash boot path. |
| Flash size | `CONFIG_ESPTOOLPY_FLASHSIZE="32MB"` | Matches the detected 32 MB device. |
| `CONFIG_SPIRAM_MODE` | `CONFIG_SPIRAM_MODE_OCT=y` | Selects Octal PSRAM. |
| `CONFIG_SPIRAM_TYPE` | No literal string key; `CONFIG_SPIRAM_TYPE_AUTO=y` | ESP-IDF's Kconfig expresses the requested AUTO type as a selector rather than a `CONFIG_SPIRAM_TYPE=` value. |
| PSRAM enable/init | `CONFIG_SPIRAM=y`, `CONFIG_SPIRAM_BOOT_INIT=y` | Initializes PSRAM before the application starts. |
| PSRAM validation | `CONFIG_SPIRAM_MEMTEST=y`; `CONFIG_SPIRAM_IGNORE_NOTFOUND` unset | Initialization/test failures remain visible and fail closed. |
| PSRAM speed | `CONFIG_SPIRAM_SPEED_80M=y` | Matches the successful runtime timing. |
| `CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ` | `160` | Runtime log confirms 160 MHz. |
| Bootloader diagnostics | `CONFIG_BOOTLOADER_LOG_LEVEL_DEBUG=y` (`4`) | Emits flash header, partition, image, and OTA-selection details. |

The generated flash arguments are:

```text
--flash_mode dout --flash_freq 80m --flash_size 32MB
```

The runtime driver confirms the actual interface with:

```text
spi_flash: detected chip: mxic (opi)
spi_flash: flash io: opi_dtr
```

Therefore `CONFIG_ESPTOOLPY_FLASHMODE="dout"` is not a fallback to the old DIO
configuration; the OPI selectors and the runtime `opi_dtr` line are the authoritative
checks for this ESP-IDF release.

## Startup Chain

The observed chain is:

```text
ESP32-S3 ROM
  -> load second-stage bootloader from 0x00000000
  -> bootloader call_start_cpu0() at 0x403c8994
  -> read and verify partition table at 0x00008000
  -> read OTA data at 0x00029000 and select ota_0
  -> validate/map app image at 0x00030000
  -> app call_start_cpu0() at 0x403755b4
  -> MSPI flash timing and Octal PSRAM initialization
  -> PSRAM memory test
  -> FreeRTOS main_task
  -> main_task calls app_main()
```

### ROM and Bootloader

The ROM starts in the board's eFuse-selected Octal mode and loads the bootloader.
The final monitor session showed:

```text
ESP-ROM:esp32s3-20210327
Octal Flash Mode Enabled
entry 0x403c8994
I (...) boot: ESP-IDF v5.5.4 2nd stage bootloader
I (...) boot.esp32s3: SPI Flash Size : 32MB
```

The bootloader map resolves `call_start_cpu0` to `0x403c8994`. This proves ROM-to-
bootloader transfer completed; the original stop was later than the ROM banner, not a
failure to enter the second-stage image.

The historical symptom used `entry 0x403c8920`, while the current clean build and
monitor use `entry 0x403c8994`. If `0x403c8920` reappears after this repair, the board
is running an older bootloader (or a different build/offset); check that the current
bootloader was written at `0x00000000` and that the reset actually followed the new
flash operation.

### Partition Table and OTA Selection

The custom partition table is at `0x8000` and its MD5 verifies. The final bootloader
printed all entries and selected the first OTA slot after the freshly flashed
`otadata` was empty:

```text
partition table verified, 9 entries
No factory image, trying OTA 0
Loaded app from partition at offset 0x30000
```

The current table is:

| Label | Type | Offset | Size |
| --- | --- | ---: | ---: |
| `nvs` | data | `0x9000` | `0x20000` (128 KiB) |
| `otadata` | data | `0x29000` | `0x2000` (8 KiB) |
| `phy_init` | data | `0x2b000` | `0x1000` (4 KiB) |
| `ota_0` | app | `0x30000` | `0x700000` (7 MiB) |
| `ota_1` | app | `0x730000` | `0x700000` (7 MiB) |
| `storage` | data | `0xe30000` | `0x400000` (4 MiB) |
| `coredump` | data | `0x1230000` | `0x40000` (256 KiB) |
| `home_ai` | data | `0x1270000` | `0x200000` (2 MiB) |

There is no factory app in this table. Any diagnostic referring to a factory image at
`0x10000` or a 1 MiB app partition describes a different temporary project and does
not apply to this `ESPS3` build.

### App Image and `call_start_cpu0`

`esptool image_info` and the linker map agree:

| Artifact | Result |
| --- | --- |
| `build/bootloader/bootloader.bin` | 24,848 bytes, 3 segments, checksum and SHA-256 valid, entry `0x403c8994` |
| `build/s3_radar_minimal.bin` | 203,760 bytes, 6 segments, checksum and SHA-256 valid, entry `0x403755b4` |
| App map | `call_start_cpu0=0x403755b4`, `app_main=0x42006bc8`, `main_task=0x42013c60` |
| Size check | App `0x31bf0`; smallest OTA partition `0x700000`; `0x6ce410` (97%) free |

The bootloader printed the app image header, mapped all six segments, calculated the
same image hash, and transferred control to `0x403755b4`. There is no linker overflow,
invalid entry point, partition overlap, or image checksum anomaly.

The ESP-IDF size check also passed for the bootloader and app. The current minimal
image uses the available IRAM budget without an overflow; future additions of
IRAM-resident code should rerun the size check before flashing.

## PSRAM Failure Path

On ESP32-S3, the application entry function in `esp_system/port/cpu_start.c` performs
MSPI setup and PSRAM initialization before the FreeRTOS `main_task` can log
`Calling app_main()`. With PSRAM enabled and `CONFIG_SPIRAM_IGNORE_NOTFOUND` unset,
chip detection, timing setup, or the configured memory test can stop startup before
application code. This is the only part of the path that can plausibly explain a
silent stop between the bootloader `entry` line and `cpu_start`/`app_main` without
involving radar or UART code.

The repaired monitor session proved that this path now completes:

```text
I (...) octal_psram: vendor id    : 0x0d (AP)
I (...) octal_psram: density      : 0x05 (128 Mbit)
I (...) esp_psram: Found 16MB PSRAM device
I (...) esp_psram: Speed: 80MHz
I (...) esp_psram: SPI SRAM memory test OK
I (...) cpu_start: Pro cpu start user code
I (...) cpu_start: cpu freq: 160000000 Hz
I (...) main_task: Calling app_main()
```

This rules out a current PSRAM initialization or memory-test failure. It does not
prove that every possible timing margin is qualified across temperature or repeated
power cycles, but it proves the connected board completed the pre-`app_main` PSRAM
stage with this image.

## Runtime Acceptance

The final TTY session used:

```bash
source /Users/zhiqin/.espressif/v5.5.4/esp-idf/export.sh
idf.py -B build fullclean
idf.py -B build build
idf.py -B build -p /dev/cu.usbmodem5B901601171 flash monitor
```

Flash output showed `Hash of data verified` for bootloader, app, partition table, and
OTA data. The monitor then showed, in order:

```text
Octal Flash Mode Enabled
SPI Flash Size : 32MB
partition table verified
Loaded app from partition at offset 0x30000
start: 0x403755b4
Found 16MB PSRAM device
SPI SRAM memory test OK
flash io: opi_dtr
cpu freq: 160000000 Hz
main_task: Calling app_main()
RADAR: UART1 ready TX=GPIO18 RX=GPIO44 baud=115200 rx_buffer=4096
RADAR: M_CTR PWM ready GPIO=4 frequency=10000Hz duty=50%
RADAR: RADAR_SYSTEM running uart=yes pwm=yes
```

The repeated `RADAR_UART_RX len=0` messages mean no radar frame was present on the
input during this session; they are post-`app_main` application behavior, not a boot
failure. BLE was not exercised by this minimal radar image, but the memory/interface
configuration no longer prevents future UART/BLE development.

## Files Changed and Scope

- `S3-radartest/ESPS3/sdkconfig.defaults`: made the N32R16 OPI/Octal/32 MB/16 MB
  configuration reproducible and enabled bootloader DEBUG diagnostics.
- `S3-radartest/ESPS3/sdkconfig`: synchronized generated Kconfig values.
- `DOCS/S3_BOOT_FAILURE_ANALYSIS.md`: this analysis and evidence record.

No business logic was changed. The radar parser, UART, PWM, and future BLE integration
remain outside this repair.

## Limits and Follow-up

- The flash and monitor evidence is one successful connected-board boot after a full
  image rewrite; it is not a long-duration or temperature/power-cycle qualification.
- Build and image validation prove the generated artifacts. The monitor session is the
  separate hardware proof for flash mode, partition loading, PSRAM, and `app_main`.
- Keep `sdkconfig.defaults` as the source of truth. A future `menuconfig` or clean
  configuration must preserve OPI/Octal, 32 MB flash, 16 MB PSRAM, and the 160 MHz CPU
  setting for ESP32-S3 N32R16.
