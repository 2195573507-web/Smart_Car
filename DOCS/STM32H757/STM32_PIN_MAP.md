# STM32H757 Pin Map

## Evidence and Naming Rules

This table records the required Smart_Car hardware net names and MCU package pins supplied for initialization. A hardware net name such as `GPS_TX` names the external source signal; its MCU alternate function can therefore be an RX function. An IOC/CubeMX check is still required to confirm the selected alternate function and any electrical settings.

No signal in this document is proof of board wiring, voltage compatibility, interrupt polarity, signal integrity, or peripheral operation.

## Encoder Inputs

| Channel | Hardware net | MCU pin | Initialization classification |
| --- | --- | --- | --- |
| RF | `RF_INT1` | `PA8` | Encoder input net |
| RF | `RF_INT2` | `PA9` | Encoder input net |
| LF | `LF_INT1` | `PA10` | Encoder input net |
| LF | `LF_INT2` | `PA13` | Encoder input net |
| RB | `RB_INT1` | `PA15` | Encoder input net |
| RB | `RB_INT2` | `PB3` | Encoder input net |
| LB | `LB_INT1` | `PB8` | Encoder input net |
| LB | `LB_INT2` | `PB9` | Encoder input net |

This initialization records these as input nets only. Timer encoder-mode assignment, pull settings, filtering, edge behavior, and counter ownership are not selected by the provided baseline and require a dedicated hardware/control task. These pins must not all be configured as EXTI inputs: `PA8`/`PB8` share EXTI line 8, `PA9`/`PB9` share line 9, and `PA15`/`PB15` share line 15. `PA13` also consumes the default SWDIO debug function when used as `LF_INT2`.

## Motor-Control GPIO

| Driver channel | Hardware net | MCU pin | Required direction |
| --- | --- | --- | --- |
| AT1 A | `AT1_AIN1` | `PC6` | GPIO output |
| AT1 A | `AT1_AIN2` | `PC5` | GPIO output |
| AT1 B | `AT1_BIN1` | `PC7` | GPIO output |
| AT1 B | `AT1_BIN2` | `PC1` | GPIO output |
| AT2 A | `AT2_AIN1` | `PC8` | GPIO output |
| AT2 A | `AT2_AIN2` | `PB14` | GPIO output |
| AT2 B | `AT2_BIN1` | `PC9` | GPIO output |
| AT2 B | `AT2_BIN2` | `PB15` | GPIO output |

GPIO output selection reserves direction-control lines only. It does not assert a safe startup level, driver enable state, PWM relationship, or motor behavior.

## BMI323 SPI and Interrupt

| Hardware net | MCU pin | Intended function |
| --- | --- | --- |
| `BMI_CS` | `PC4` | GPIO output, SPI chip select |
| `BMI_SCK` | `PA5` | SPI clock |
| `BMI_SDI` | `PA7` | SPI controller output / device data input |
| `BMI_SDO` | `PA6` | SPI controller input / device data output |
| `BMI_INT1` | `PB2` | GPIO/EXTI input as supported by IOC |

## LSM303 I2C

| Hardware net | MCU pin | Intended function |
| --- | --- | --- |
| `LSM_SCL` | `PD12` | I2C clock |
| `LSM_SDA` | `PD13` | I2C data |

## Communication and GPS

| Interface | Hardware net | MCU pin | Signal-direction note |
| --- | --- | --- | --- |
| STM32 communication | `STM_TX` | `PD3` | Required net recorded; supports USART2 CTS/NSS, not UART TX/RX |
| STM32 communication | `STM_RX` | `PD4` | Required net recorded; supports USART2 DE/RTS, not UART TX/RX |
| GPS | `GPS_TX` | `PA3` | GPS transmitter is expected to connect to an MCU RX function |
| GPS | `GPS_RX` | `PA2` | GPS receiver is expected to connect to an MCU TX function |

## IOC Cross-Check Requirements

1. Every pin above must occur once in the final IOC pin list with the same user label.
2. SPI must use the `PA5`/`PA6`/`PA7` set, I2C must use `PD12`/`PD13`, and GPS must use `PA2`/`PA3` with direction resolved against the external labels.
3. `PD3`/`PD4` cannot meet the requested STM UART TX/RX connection on `STM32H757XIH6`; CubeMX must not be used to claim a false UART configuration. Resolve the hardware route before enabling the STM communication UART.
4. `PA13` use for `LF_INT2` must be explicitly accepted as a loss of default SWDIO access, or the hardware/debug route must change.
5. Timer PWM channels must be placed only on connected hardware pins. No unconnected timer peripheral should be enabled merely to reserve a capability.
