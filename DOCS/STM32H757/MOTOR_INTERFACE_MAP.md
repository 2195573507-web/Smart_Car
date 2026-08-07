# STM32H757 Motor and Encoder Interface Map

## Scope and Evidence

This document records the resource-allocation result for the frozen Smart_Car
extension-board netlist. It does not rename PCB nets, alter the extension-board
design, add motor-control or PID code, or establish electrical, waveform, or
vehicle behavior. The mappings below are static alternate-function and IOC
planning evidence only.

The applicable MCU is `STM32H757XIH6`. The existing protected interfaces remain
unchanged: SPI1 on `PA5`/`PA6`/`PA7`, I2C4 on `PD12`/`PD13`, USART2 on
`PA2`/`PA3`, and SWD on `PA13`/`PA14`.

## Motor Control

The frozen interface uses PWM plus a GPIO direction input for each motor-driver
channel. `AIN1` and `BIN1` are the PWM candidates; `AIN2` and `BIN2` remain
direction GPIO outputs. The four PWM candidates are the complete contiguous
channel set of TIM3, so they can share one PWM time base.

| Motor channel | PWM net | MCU pin | Timer alternate function | Direction net | MCU pin | Direction mode |
| --- | --- | --- | --- | --- | --- | --- |
| AT1 A | `AT1_AIN1` | `PC6` | `TIM3_CH1` | `AT1_AIN2` | `PC5` | GPIO output |
| AT1 B | `AT1_BIN1` | `PC7` | `TIM3_CH2` | `AT1_BIN2` | `PC1` | GPIO output |
| AT2 A | `AT2_AIN1` | `PC8` | `TIM3_CH3` | `AT2_AIN2` | `PB14` | GPIO output |
| AT2 B | `AT2_BIN1` | `PC9` | `TIM3_CH4` | `AT2_BIN2` | `PB15` | GPIO output |

### PWM Allocation Result

`TIM3_CH1` through `TIM3_CH4` satisfy the four required PWM outputs without
using SWD, SPI1, I2C4, or USART2 pins. `PC8` exposes FMC alternatives, but
selecting `TIM3_CH3` does not require FMC and must remain exclusive of any
future FMC selection. This is a resource allocation only: output polarity,
safe startup level, frequency, duty cycle, dead time, motor-driver enable, and
motor behavior require later authorized work and hardware validation.

## Encoder Inputs

| Wheel | Phase A net and pin | Phase B net and pin | Pin alternate functions found | TIM Encoder Mode result |
| --- | --- | --- | --- | --- |
| RF | `RF_INT1` -> `PA8` | `RF_INT2` -> `PA9` | `TIM1_CH1` / `TIM1_CH2` | Supported by TIM1 |
| LF | `LF_INT1` -> `PA10` | `LF_INT2` -> `PA13` | `TIM1_CH3` / no TIM signal on PA13 | Not supported |
| RB | `RB_INT1` -> `PA15` | `RB_INT2` -> `PB3` | `TIM2_CH1` / `TIM2_CH2` | Supported by TIM2 |
| LB | `LB_INT1` -> `PB8` | `LB_INT2` -> `PB9` | `TIM4_CH3` / `TIM4_CH4` | Not supported |

TIM Encoder Mode consumes timer inputs TI1 and TI2, represented by timer
channels 1 and 2. Consequently, the RF pair can use TIM1 and the RB pair can
use TIM2. LF cannot use TIM1 Encoder Mode because `PA10` is only TIM1_CH3 and
`PA13` exposes only `DEBUG_JTMS-SWDIO` or GPIO. LB cannot use TIM4 Encoder
Mode because `PB8`/`PB9` expose TIM4_CH3/CH4 rather than the required CH1/CH2
pair. The frozen netlist therefore supports two, not four, hardware TIM
quadrature encoder interfaces.

`PA13` must remain SWDIO to preserve the required SWD connection on
`PA13`/`PA14`; it must not be reassigned to `LF_INT2`. `PA15` and `PB3` carry
JTAG/trace alternate functions, respectively. They do not conflict with the
required Serial-Wire SWD pair, but a future full-JTAG or SWO trace selection
would conflict with the RB TIM2 assignment.

## Recommended CubeMX Resource Selection

| Resource | Selection | Purpose | Status |
| --- | --- | --- | --- |
| TIM3 | PWM generation on CH1-CH4 | Four motor PWM outputs | Recommended |
| TIM1 | Encoder Mode on CH1/CH2 | RF quadrature encoder | Recommended |
| TIM2 | Encoder Mode on CH1/CH2 | RB quadrature encoder | Recommended |
| LF encoder | Keep frozen nets as GPIO inputs | No valid timer pair on PA10/PA13; PA13 is SWDIO | Resource insufficient |
| LB encoder | Keep frozen nets as GPIO inputs | TIM4 channels are CH3/CH4, not Encoder Mode TI1/TI2 | Resource insufficient |

No PCB or net-name change is authorized or made here. The two unsupported
encoder pairs are recorded as resource constraints only; this document does
not select a workaround or hardware redesign.

## Follow-up Constraints

1. Keep SPI1 (`PA5`, `PA6`, `PA7`), I2C4 (`PD12`, `PD13`), USART2 (`PA2`,
   `PA3`), and SWD (`PA13`, `PA14`) unchanged when editing the IOC.
2. CubeMX must report a clean pinout before generation. The generated CM7 and
   CM4 projects must then be clean-built independently with the configured ARM
   GCC toolchain.
3. IOC parsing, code generation, and compilation do not prove PCB wiring,
   logic-level compatibility, PWM waveform, encoder polarity/counting, motor
   response, or safety. No flashing or hardware test is part of this stage.
4. The LF and LB resource shortfalls need an explicitly authorized hardware or
   interface decision before four-wheel TIM Encoder Mode can be claimed.

## Static Sources

- `STM32H757/Smart_Car_H757.ioc`: current net labels and protected interface
  assignments.
- STM32CubeMX `STM32H757XIHx.xml`: package alternate-function inventory used
  for the TIM1, TIM2, TIM3, and TIM4 mappings.
- `STM32H7xx_HAL_Driver`: Encoder Mode API reserves timer channels 1 and 2.
