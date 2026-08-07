# STM32H757 Hardware Resource Audit

## Scope And Evidence

This independent audit reviews the frozen Smart_Car expansion-board net list
against the final `STM32H757/Smart_Car_H757.ioc` configuration for
`STM32H757XIH6` / TFBGA240. It does not change the PCB, net names, IOC, or
application code.

Static evidence reviewed:

- `Smart_Car_H757.ioc`, SHA-256
  `9860d7a8a91691cf6d1b58fb53bc88f42d19407cb2a3dc8adbbe8291d55f054f`.
- STM32CubeMX 6.18.0 device database:
  `STM32H757XIHx.xml`.
- The required pin records in `STM32_PIN_MAP.md`.

## Occupied Resources

| Resource | Final IOC assignment | Audit result |
| --- | --- | --- |
| SWD | `PA13` = `DEBUG_JTMS-SWDIO`, `PA14` = `DEBUG_JTCK-SWCLK` | Preserved in the IOC. |
| SPI1 | `PA5` SCK, `PA6` MISO, `PA7` MOSI | Preserved; no selected motor or encoder AF shares these pins. |
| I2C4 | `PD12` SCL, `PD13` SDA | Preserved; no selected motor or encoder AF shares these pins. |
| USART2 | `PA2` TX, `PA3` RX | Preserved; no selected motor or encoder AF shares these pins. |
| Base timer | `TIM6` | Remains enabled and independent of the motor/encoder selections. |
| PWM timer | `TIM3` channels 1 through 4 | Four connected PWM candidates are selected. |
| Encoder timers | `TIM1` and `TIM2`, TI1/TI2 encoder mode | Supports RF and RB only. |

## PWM Resource Table

| Motor driver input | MCU pin | Final IOC signal | Timer channel | Result |
| --- | --- | --- | --- | --- |
| `AT1_AIN1` | `PC6` | `S_TIM3_CH1` | `TIM3_CH1` | Pass |
| `AT1_BIN1` | `PC7` | `S_TIM3_CH2` | `TIM3_CH2` | Pass |
| `AT2_AIN1` | `PC8` | `S_TIM3_CH3` | `TIM3_CH3` | Pass |
| `AT2_BIN1` | `PC9` | `S_TIM3_CH4` | `TIM3_CH4` | Pass |
| `AT1_AIN2` | `PC5` | `GPIO_Output` | Direction GPIO | Pass |
| `AT1_BIN2` | `PC1` | `GPIO_Output` | Direction GPIO | Pass |
| `AT2_AIN2` | `PB14` | `GPIO_Output` | Direction GPIO | Pass |
| `AT2_BIN2` | `PB15` | `GPIO_Output` | Direction GPIO | Pass |

`PC6` through `PC9` each expose the matching `TIM3_CH1` through
`TIM3_CH4` alternate function in the STM32H757XIH6 database. The selected
`TIM3` channels do not take a fixed SPI1, I2C4, USART2, or SWD pin. `PC8`
also has FMC and trace alternatives in the device database, but neither is
selected in this IOC.

## Encoder Resource Table

Timer encoder mode requires a single timer's TI1 and TI2 inputs, which map to
that timer's channels 1 and 2. Channels 3 and 4 cannot be substituted for a
TI1/TI2 quadrature pair.

| Wheel | Net 1 / pin | Net 2 / pin | Available AF facts | Final IOC result | Encoder-mode result |
| --- | --- | --- | --- | --- | --- |
| RF | `RF_INT1` / `PA8` | `RF_INT2` / `PA9` | `TIM1_CH1` + `TIM1_CH2` | `TIM1` TI12 encoder mode; both pins selected | Pass |
| LF | `LF_INT1` / `PA10` | `LF_INT2` / `PA13` | `PA10` has `TIM1_CH3`; `PA13` has only debug/GPIO, no timer channel | `PA10` remains GPIO input; `PA13` remains SWDIO | Fail |
| RB | `RB_INT1` / `PA15` | `RB_INT2` / `PB3` | `TIM2_CH1` + `TIM2_CH2` | `TIM2` TI12 encoder mode; both pins selected | Pass |
| LB | `LB_INT1` / `PB8` | `LB_INT2` / `PB9` | `TIM4_CH3` + `TIM4_CH4`; also separate `TIM16_CH1` / `TIM17_CH1` alternatives | Both pins remain GPIO inputs | Fail |

The frozen net list therefore provides two valid timer-encoder pairs, not the
four required for four-wheel quadrature acquisition. No false timer mapping is
configured for LF or LB.

## Conflict List

| Priority | Finding | Effect | Current treatment |
| --- | --- | --- | --- |
| P0 | LF encoder pins are not a TI1/TI2 pair: `PA10` is only `TIM1_CH3`, while `PA13` has no timer AF. | LF cannot use TIM encoder mode on its frozen nets. | Keep `PA10` as input and retain `PA13` for SWD. |
| P0 | LB encoder pins are `TIM4_CH3` / `TIM4_CH4`, not the TI1/TI2 pair required by timer encoder mode. | LB cannot use TIM encoder mode on its frozen nets. | Keep both as inputs; do not enable a misleading encoder timer. |
| P1 | `LF_INT2` is wired to `PA13`, the SWDIO pin. | Selecting LF input would remove SWD; retaining SWD leaves this encoder net unavailable to firmware. | IOC preserves `PA13` as `DEBUG_JTMS-SWDIO`. |
| P2 | RB uses `PA15` (JTDI) and `PB3` (JTDO/TRACESWO). | Full JTAG and SWO trace are unavailable on these pins when TIM2 is selected. SWD on `PA13`/`PA14` remains selected. | Accepted for the RB encoder pair; validate debug access on hardware later. |
| P2 | `PC8` has optional FMC/trace alternatives. | A later FMC or trace configuration could conflict with `TIM3_CH3`. | No current conflict because neither alternative is selected. |

## Final Recommendation

1. Keep the final PWM allocation: `TIM3_CH1..CH4` on `PC6..PC9`, with
   `PC5`, `PC1`, `PB14`, and `PB15` as the corresponding direction GPIO.
2. Keep RF on `TIM1` TI1/TI2 (`PA8`/`PA9`) and RB on `TIM2` TI1/TI2
   (`PA15`/`PB3`).
3. Do not claim a four-wheel Timer Encoder Mode configuration. LF and LB
   cannot meet that requirement with the frozen nets, and the IOC correctly
   avoids inventing an alternate route.
4. A four-wheel hardware-timer encoder design requires an authorized PCB/net
   revision. This audit does not propose or apply one.

## Validation Limits

This report establishes only static IOC and alternate-function consistency.
CubeMX parse/pin-conflict checking, CM7/CM4 generation and clean builds, and
hardware validation are separate evidence layers. Even a successful build
would not prove SWD electrical access, PWM waveforms, encoder polarity,
pull-up/down requirements, counter direction, motor-driver operation, or
vehicle behavior. No hardware was flashed or exercised for this audit.
