# STM32H757 Flash Source Report

## Scope

This is a read-only filesystem and build-artifact audit. No C/C++ source,
CMake configuration, protocol, S3/App file, or programmer command was changed
or executed. No device was flashed.

## 1. Current CM7 ELF

The project documents the canonical CM7 artifact at
`STM32H757/CM7/README.md:12-14`, and the top-level CMake guard names the same
file at `STM32H757/CMakeLists.txt:3-6`:

| Image | Timestamp | Size |
| --- | --- | ---: |
| `/Users/zhiqin/Projects/Smart_Car/STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf` | 2026-08-09 09:57:46 | 2,773,344 bytes |

BMI323 source/object comparison:

| File | Timestamp | Interpretation |
| --- | --- | --- |
| `/Users/zhiqin/Projects/Smart_Car/STM32H757/Middleware/Sensor/BMI323/bmi323.c` | 2026-08-09 09:56:35 | source changed before the build |
| `/Users/zhiqin/Projects/Smart_Car/STM32H757/CM7/build/Debug/CMakeFiles/Smart_Car_H757_CM7.dir/Users/zhiqin/Projects/Smart_Car/STM32H757/Middleware/Sensor/BMI323/bmi323.c.obj` | 2026-08-09 09:57:46 | object rebuilt after source |
| `/Users/zhiqin/Projects/Smart_Car/STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf` | 2026-08-09 09:57:46 | ELF linked with that object |

The ELF contains the current split BMI323 strings:

```text
[BMI323][CS] %s
[BMI323][DEBUG]
[BMI323][ERROR]
[BMI323][INIT]
[BMI323][SPI]
[BMI323][SPI_CONFIG]
[BMI323][SPI_TRACE]
[BMI323][WHOAMI]
```

This proves local source-to-object-to-ELF consistency. It does not prove that
this ELF is the image programmed into the board.

## 2. STM32H757 image files found

The full repository scan found 252 files ending in `.elf`, `.hex`, or `.bin`;
most are ESP-IDF/CMake intermediates or unrelated tools. After excluding
`CMakeFiles`, ESP-IDF internals, bootloader/partition intermediates, and model
files, 62 application-like artifacts remain across all projects.

The only STM32H757 application image candidates are:

| Full path | Timestamp | Size | Role |
| --- | --- | ---: | --- |
| `/Users/zhiqin/Projects/Smart_Car/STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf` | 2026-08-09 09:57:46 | 2,773,344 bytes | CM7 application |
| `/Users/zhiqin/Projects/Smart_Car/STM32H757/CM4/build/Debug/Smart_Car_H757_CM4.elf` | 2026-08-03 00:06:48 | 1,964,636 bytes | CM4 application |

No STM32H757 `.hex` or `.bin` application image was found. The `.bin` files
under `STM32H757/CM4/build/Debug/CMakeFiles/...` and
`STM32H757/CM7/build/Debug/CMakeFiles/...` are compiler ABI test objects, not
flash images.

## 3. Flash scripts and commands

Search scope included STM32H757 scripts, JSON/YAML task files, Markdown, and
project tools. No STM32 programming command was found for:

```text
STM32_Programmer_CLI
st-flash
openocd
JLink
J-Link
JLinkExe
```

The only STM32-related post-build/toolchain references are:

- `STM32H757/gcc-arm-none-eabi.cmake:15-20`: sets `objcopy` and `.elf`
  suffixes; it does not invoke a programmer or generate a flash artifact.
- `STM32H757/starm-clang.cmake:14-19`: equivalent toolchain settings.

An `openocd` process-cleanup command exists at
`ESPS3/.vscode/tasks.json:7`; it is S3 tooling, not an STM32H757 flash command.

## 4. Possible old or alternate firmware sources

Within `STM32H757`, no directories named `build_old`, `Debug_old`, `Release`,
`backup`, or `cmake-build*` were found. There are, however, three build roots:

```text
/Users/zhiqin/Projects/Smart_Car/STM32H757/CM4/build
/Users/zhiqin/Projects/Smart_Car/STM32H757/CM7/build
/Users/zhiqin/Projects/Smart_Car/STM32H757/build
```

The top-level `/STM32H757/build/Debug` contains only CMake cache/configuration
state and no ELF/HEX/BIN output. It is not a usable flash image directory.

The repository also contains archived S3/radar images under
`S3-radartest/archive/builds` and `S3-radartest/分支项目`; those are unrelated to
STM32H757 and cannot be used to identify the STM32 image.

## 5. Root-cause judgment

- **Confirmed:** the current local CM7 Debug ELF is the newest STM32H757
  application artifact and was linked after the BMI323 source object was rebuilt.
- **Confirmed:** there is no checked-in STM32 flash script or recorded command
  selecting an image.
- **Not provable from this workspace:** which file, if any, was actually
  programmed into the connected STM32H757. There is no device readback,
  programmer log, or flash-history record.

The most likely explanation for runtime logs that do not match this ELF is an
older or different image on the board, but that remains a hypothesis until the
target flash is read back or a programmer log identifies the input file.
