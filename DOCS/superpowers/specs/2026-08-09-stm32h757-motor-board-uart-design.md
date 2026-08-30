# STM32H757 MotorBoard UART Design

## Status

Approved design for implementation planning. The design is limited to the
first-stage UART transport and raw communication diagnostics. No motor-board
business protocol is defined by the current workspace.

## Scope and Exclusions

The change adds `STM32H757/Middleware/MotorBoard` and configures the confirmed
board UART route:

| Board net | STM32H757 pin | Function | Configuration |
| --- | --- | --- | --- |
| `BOARD_TX` | `PC7` | board transmitter -> MCU receiver | `USART6_RX` |
| `BOARD_RX` | `PC6` | MCU transmitter -> board receiver | `USART6_TX` |

The UART is 115200 baud, 8 data bits, no parity, 1 stop bit, with hardware
flow control disabled. Existing USART1 logging, USART2 STM-S3 transport, IMU,
LSM303, the SRPv4 STM-S3 link, BLE, S3 firmware, and calibration code remain
unchanged.

The former TIM3 CH1/CH2 local PWM output on PC6/PC7 is stopped. Its generated
initialization and historical pin mapping remain in source comments/history
until the implementation confirms there are no runtime consumers. No local
PWM, speed loop, encoder loop, or vehicle control is added.

## Components

### `motor_board_uart.c`

Owns the `USART6` HAL transport boundary. It provides bounded blocking TX for
task context, a fixed-size RX cache, idle/timeout handling, and raw receive
statistics. The UART handle remains owned by the generated STM32 startup/MSP
layer; the MotorBoard module only references the initialized handle.

### `motor_board_protocol.c`

Owns protocol framing/decoding hooks. Because no board manual or source frame
definition is present, the encoder and parser return an explicit unsupported
result and never emit guessed command bytes. Raw diagnostic bytes are kept in
the UART layer and are not treated as business commands or status frames.

### `motor_board.c`

Owns the public lifecycle/API boundary and the FreeRTOS diagnostic task. It
creates the test task only after the existing `SYSTEM READY` boot point. The
task periodically emits a documented non-protocol diagnostic pattern, drains
the bounded RX cache, and logs every action with the `[MOTOR_BOARD]` prefix.
`motor_board_set_pwm` and `motor_board_read_status` remain source-compatible
interfaces but return `false` with a protocol-missing diagnostic until a
verified board protocol is supplied.

### `motor_board.h`

Exports initialization, raw UART send, the requested PWM/status API, and test
task start. It does not expose the USART handle or generated GPIO symbols.

## Startup and Data Flow

```text
main()
  -> MX_USART6_UART_Init()
  -> motor_board_init()
  -> boot_log("SYSTEM", "READY")
  -> motor_board_test_task_start()
  -> vTaskStartScheduler()

motor_board_test_task
  -> motor_board_uart_send_raw(diagnostic pattern)
  -> motor_board_uart_receive_until_idle(timeout)
  -> raw RX log / counters
```

The task uses a finite delay and finite HAL timeouts, so a disconnected board
cannot block the scheduler indefinitely. No callback is added to the existing
USART1/USART2 paths and no SRPv4 frame is passed through this module.

## TIM3 Resource Retirement

Before editing, the implementation will search all CM7 sources for
`MX_TIM3_Init`, `HAL_TIM_PWM_Start`, `HAL_TIM_PWM_Stop`, `pwm_set_duty`, and
`bsp_pwm_*`. The active startup call and PC6/PC7 post-init mapping will be
disabled. The old generated TIM3 setup will remain as a clearly marked
historical block or source record rather than being silently deleted. The
PWM BSP will not drive a timer after the route is retired; any remaining test
API returns a non-operational status.

## Error Handling and Logging

All module logs use `[MOTOR_BOARD]` and distinguish initialization, TX, RX,
timeout, protocol-missing, and task-creation failures. Transport failures are
non-fatal to the existing boot and IMU tasks. RX overflow and HAL error counts
remain observable in RAM and are included in the integration report.

## Protocol Evidence Boundary

`MOTOR_BOARD_PROTOCOL_ANALYSIS.md` will record:

- confirmed: board net mapping, USART6 route, and 115200 8N1 configuration;
- unconfirmed: frame header, length, command identifier, four-motor field
  encoding, checksum, response/status format, and timing/ack semantics;
- required follow-up: the motor-board manual or captured, identified frames.

No PWM command or status parser is claimed until those fields are supplied and
reviewed.

## Verification

1. Static searches confirm the requested pins and that no protected IMU,
   SRPv4, S3, or BLE files changed.
2. IOC/source checks confirm PC6=`USART6_TX`, PC7=`USART6_RX`, 115200 8N1,
   and no active TIM3 PC6/PC7 post-init.
3. `STM32H757/CM7` Debug clean build validates compile/link integration.
4. No flash, serial monitor, logic analyzer, motor movement, or electrical
   acceptance is claimed. Those remain hardware verification items.
