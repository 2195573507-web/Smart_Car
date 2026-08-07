# ESP32-S3 YDLIDAR X3PRO UART Raw Data Test

## Scope

This image is a minimal ESP-IDF 5.5.4 test for receiving raw bytes from a
YDLIDAR X3PRO over UART. The first phase proves stable byte reception and
preserves the input stream for protocol work. It does not implement a complete
point cloud, target tracking, networking, or ROS integration.

The project remains an ESP32-S3 image. Existing flash, PSRAM, partition, and
console settings are retained; this document does not authorize changing
`sdkconfig`, flash settings, or PSRAM settings.

## Retained Board Configuration

| Item | Required value |
| --- | --- |
| Target | `esp32s3` |
| ESP-IDF | `v5.5.4` |
| Flash | `32MB` |
| PSRAM | `16MB` |
| Console | Existing project console configuration |

These values describe the existing board configuration. A successful build is
not evidence that the connected board has the stated memory devices or that
the radar wiring is correct.

## X3PRO UART and Motor Wiring

| Signal | ESP32-S3 assignment | X3PRO connection |
| --- | --- | --- |
| UART controller | `UART1` | X3PRO serial link |
| MCU RX | `GPIO44` | X3PRO TX output -> S3 UART RX |
| MCU TX | `GPIO18` | X3PRO RX |
| UART format | `115200`, `8N1`, no parity | X3PRO UART data link |
| UART driver RX buffer | `4096` bytes | ESP-IDF driver buffer |
| M_CTR motor PWM | `GPIO4` | X3PRO motor control input |
| PWM frequency | `10000Hz` | LEDC output |
| PWM duty | `70%` | Motor start duty |

`radar_pwm_init()` configures LEDC on GPIO4 and applies the 10kHz, 70% duty
output to start the scanner motor. After a 3000 ms motor spin-up delay,
`radar_gpio_monitor_init()` configures GPIO44 as an input with both pulls
disabled and starts `radar_gpio_monitor_task`. The monitor reads
`gpio_get_level(GPIO_NUM_44)` every 100 ms and logs
`RADAR_GPIO44_LEVEL=<0|1>`. `radar_uart_init()` then configures UART1 and starts
the continuous receive task. UART0 remains the application console.

The required `app_main` startup order is:

```text
radar_pwm_init();
delay 3000ms;
radar_gpio_monitor_init();
radar_uart_init();
```

The GPIO monitor is observation-only. It must not change the UART pin
multiplexing, parser, `sdkconfig`, flash settings, or PSRAM settings.

## X3PRO Protocol Basis

The parser is based on the binary scan-packet structure described by the
YDLIDAR X3PRO data manual. The stream uses the little-endian sync value
`0x55AA`, represented on the wire as `AA 55`. A candidate packet contains the
command/type byte (`CT`), a sample-count/length byte (`LSN`), start and end
angle fields (`FSA`/`LSA`), sample data, and a checksum field (`CS`). The total
packet size is derived from the protocol length/sample-count field and the
selected sample encoding.

The exact X3PRO firmware/manual revision and checksum variant must be confirmed
with a captured device stream before interpreting point fields. The first
phase therefore keeps the checksum hook and point decoder as interfaces. It
does not invent `angle`, `distance`, or `quality` values.

The parser intentionally has no legacy fixed-header or fixed-length contract.
Candidate boundaries come only from X3PRO protocol metadata.

## Parser and Receive Flow

```text
X3PRO TX output
  -> GPIO44 / S3 UART1 RX
  -> ESP-IDF UART driver buffer (4096 bytes)
  -> radar_uart.c read buffer
  -> radar_parser.c ring buffer
  -> frame-header search and length-derived boundary
  -> optional checksum callback
  -> raw-frame callback
```

`radar_parser.c` provides the framing boundary only:

- A ring buffer accepts data split across any number of UART reads.
- The parser searches for the X3PRO sync/header and resynchronizes when noise
  or an unrecognized byte precedes it.
- The frame length comes from the protocol field; it is never a fixed constant.
- A checksum/CRC validator callback is reserved for the confirmed manual
  variant.
- Incomplete or unrecognized data remains available for continued parsing;
  subsequent UART reads can complete the candidate frame.
- `radar_parser_parse_measurement()` remains a decode interface and returns no
  fabricated measurement until the exact point encoding is confirmed.

The GPIO monitor and UART task run concurrently. Every 100 ms the monitor
emits one level record, including when the UART is idle. The UART task retains a
length record for every read, including zero-length reads. When bytes arrive,
it logs the raw chunk and feeds it to the parser:

```text
RADAR_GPIO44_LEVEL=1
RADAR_UART_RX len=0
RADAR_GPIO44_LEVEL=1
RADAR_UART_RX len=12
RADAR_HEX:
AA 55 ...
```

`RADAR_GPIO44_LEVEL=0` or `RADAR_GPIO44_LEVEL=1` is an electrical-level
observation only; it does not by itself prove valid UART framing. A non-zero
`RADAR_UART_RX len` followed by `RADAR_HEX:` bytes proves that the UART driver
received bytes. The raw bytes still require the parser/protocol evidence below
before any decoded measurement claim.

When a frame boundary is recognized, the callback logs:

```text
RADAR_FRAME:
angle=<decoded value or unavailable>
distance=<decoded value or unavailable>
quality=<decoded value or unavailable>
```

`unavailable` is the expected value while the point-field variant is still
unconfirmed. It is preferable to an invented value because this phase is a
UART/protocol capture test, not a completed point-cloud decoder.

## Source Layout

```text
  main/
    main.c
    radar/
      radar_uart.c
      radar_uart.h
      radar_parser.c
      radar_parser.h
```

`radar_uart.c` owns UART1, the 4096-byte driver buffer, continuous reads, raw
logging, the GPIO44 level monitor, and the GPIO4 LEDC motor output.
`radar_parser.c` owns buffering and protocol framing. `main.c` starts PWM,
waits 3000 ms, starts the GPIO44 monitor, then starts UART1 and reports system
state while the worker tasks run.

## Build and Evidence Boundaries

Use the existing ESP-IDF v5.5.4 environment from this directory:

```sh
idf.py fullclean
idf.py build
```

`idf.py fullclean` and `idf.py build` prove only that the current source can be
configured, compiled, and linked with the retained project configuration. They
do not prove boot, flash, PSRAM stability, GPIO electrical behavior, motor
rotation, UART signal integrity, or X3PRO protocol compatibility.

Device acceptance requires a separately captured monitor session from the
actual ESP32-S3 board and attached X3PRO. That evidence should show successful
UART1/PWM initialization, a repeated 100 ms `RADAR_GPIO44_LEVEL` sequence,
repeated `RADAR_UART_RX len=...` records without UART errors or resets, and
captured `RADAR_HEX:` packet bytes suitable for confirming the manual revision,
checksum, sample encoding, angle units, distance units, and quality semantics.
An idle session with stable GPIO44 levels and only `len=0` records establishes
the observed electrical state but does not establish that the X3PRO is
transmitting. No point-cloud or decoded-field claim is valid without device
evidence.
