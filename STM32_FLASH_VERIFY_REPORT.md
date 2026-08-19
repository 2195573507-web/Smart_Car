# STM32H757 Flash Verify Report

## Scope and safety

The source tree and BMI323 implementation were not modified. No download,
erase, write, reset, or run command was executed. The only target operations
performed were probe enumeration, SWD connection/status, and flash readback.

## 1. ST-Link status

`STM32_Programmer_CLI` is installed at:

```text
/opt/ST/STM32CubeCLT_1.22.0/STM32CubeProgrammer/bin/STM32_Programmer_CLI
```

Version reported by the tool: `STM32CubeProgrammer v2.23.0`.

`STM32_Programmer_CLI -l` found:

| Field | Value |
| --- | --- |
| ST-Link serial | `37FF71064E573436BB1C1B43` |
| ST-Link firmware | `V2J37S7` |
| Access port | `4` |
| DFU | no device |
| J-Link | no probe |

## 2. SWD and chip state

`STM32_Programmer_CLI -c port=SWD` succeeded:

| Field | Value |
| --- | --- |
| Voltage | `3.21 V` |
| SWD frequency | `4000 KHz` |
| Connect mode | `Normal` |
| Reset mode | `Software reset` |
| Device ID | `0x450` |
| Revision | `Rev V` |
| Device | `STM32H7xx` |
| NVM size | `2 MBytes` |
| CPU | `Cortex-M7/M4` |
| Bootloader version | `0x91` |

`STM32_Programmer_CLI -c port=SWD -score` reported `Core is halted`. This
confirms SWD access and identifies the chip, but it does not prove that the
application is currently executing.

## 3. Temporary programming command

The command to program and verify the canonical CM7 ELF is:

```sh
/opt/ST/STM32CubeCLT_1.22.0/STM32CubeProgrammer/bin/STM32_Programmer_CLI \
  -c port=SWD \
  -d /Users/zhiqin/Projects/Smart_Car/STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf \
  -v -rst
```

This command was recorded only and was **not executed**. It would write the
target and reset it, so it requires an explicit programming decision.

## 4. ELF and target flash consistency

Canonical ELF:

```text
/Users/zhiqin/Projects/Smart_Car/STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf
```

The ELF has a CM7 flash load range starting at `0x08000000` and ending at
`0x08017FD0`, a total of 98,256 bytes. The ELF was converted to a temporary
binary, then the same range was read from the connected target using:

```sh
STM32_Programmer_CLI -c port=SWD \
  -u 0x08000000 98256 /tmp/stm32_flash_verify.53dHvU/STM32H757_CM7_flash.bin
```

Readback result:

| Check | Result |
| --- | --- |
| Readback size | `98,256 bytes` |
| ELF binary SHA-256 | `d369c88f43b3c91b2d29fa3a7b6e8f67eb9a61be903447a828eb559f5a68e400` |
| Target readback SHA-256 | `d369c88f43b3c91b2d29fa3a7b6e8f67eb9a61be903447a828eb559f5a68e400` |
| Byte comparison | `cmp_exit=0` |

The target flash contents in this CM7 range are byte-for-byte identical to the
current local Debug ELF payload. This is stronger than a timestamp comparison,
but it still does not establish that the halted core has reached `main()`.

## 5. BMI323 startup log verification

The current ELF contains these BMI323 startup strings:

```text
[BMI323][WHOAMI]
[BMI323][SPI]
```

No runtime UART/BLE capture was performed. Because `-score` reports the core as
halted, the required startup records cannot be confirmed from this session.
The remaining runtime verification is to run/reset the target under the normal
startup path and capture the existing USART2/SC_TYPE_LOG-to-BLE log stream:

```text
[BMI323][WHOAMI]
[BMI323][SPI]
```

## Final judgment

- **Flash source:** confirmed. The connected target's CM7 flash matches the
  current `Smart_Car_H757_CM7.elf` exactly.
- **ST-Link/SWD:** confirmed and readable.
- **CPU execution:** not confirmed; current core state is halted.
- **BMI323 runtime logs:** not confirmed; static ELF strings are present, but no
  startup capture was made.
