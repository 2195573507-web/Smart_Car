# SCBP Audit Findings

This file records source evidence while the audit proceeds. Entries must retain
exact file/function references and distinguish confirmed facts from inference.

## Frame and transport

- `STM32H757/Middleware/Communication/SmartCar_Frame/sc_frame.h` and the matching
  S3 `components/smartcar_protocol/include/frame.h` define SCBP-V3: `AA 55`,
  version `0x01`, 14-byte overhead, 128-byte maximum payload, node IDs STM32
  `0x01` and S3 `0x02`, flags, message IDs, and parser error codes.
- `sc_frame.c` on both targets implements CRC16-MODBUS with initial `0xFFFF`,
  reflected polynomial `0xA001`; encode/decode covers bytes `VER` through the
  payload and stores CRC little-endian. The sequence counter is an 8-bit
  per-process transmitter counter with natural wrap.
- STM `UART_Link/uart_link.c` configures USART2, 115200, 8 data bits, 1 stop,
  no parity, no hardware flow control; PA2/PA3 are generated AF7 USART2 pins.
  STM RX uses a 512-byte ring and 128-byte `HAL_UARTEx_ReceiveToIdle` chunks;
  TX mutex/HAL timeout is 20 ms.
- S3 `components/stm_uart/stm_uart.c` configures UART2 on GPIO17/18 with the
  same 115200 8N1/no-flow settings. It uses a 4096-byte driver RX/TX buffer,
  4096-byte newest-byte storage ring, 256-byte service read buffer, and 100 ms
  read/TX wait limits.
- Both C parsers hold `SC_FRAME_MAX_SIZE` (142 bytes), accept fragmented input,
  bound LEN at 128, validate header/version/priority/flags/length/CRC, and seek
  the latest possible `AA` after an error. Per-source sequence states are FIRST,
  IN_ORDER, GAP, DUPLICATE, or OUT_OF_ORDER; diagnostics do not suppress a
  syntactically valid callback.

## Message and payload evidence

- Active STM producers are `imu_boot_manager.c` (0x0202, 0x0203, 0x0204,
  0x0205, 0x0206, 0x0208, 0x0401, 0x0007, 0x0302 ACK),
  `imu_runtime.c` (0x0200, 0x0201, 0x0207), and `bsp_uart.c` (0xF000).
- Active S3 sends are `command_bridge.c`/`radar_calibration_manager.c` for
  0x0002, 0x0005, 0x0006, 0x0301, 0x0302, and 0x0401; S3 receives/relays the
  STM telemetry IDs in the same bridge.
- `imu_boot_manager.c:597-633` serializes 0x0202 as
  `stage,u8 pwm,u32 sample_count,u32 sample_total,u8 error` (11 bytes) and
  0x0208 as phase/progress/error/flags/index/PWM plus two LE32 timestamps (16).
- `imu_runtime.c:77-137` serializes 0x0200 as 38 bytes and 0x0207 as a
  source-tagged 30-byte payload. The current 0x0200 producer writes sensor/status,
  accel xyz, reserved/zero gyro xyz, and mag xyz; S3 accepts historical 43-byte
  input but current STM code does not emit it.
- `imu_runtime.c:144-165` and `s3_service.c:189-239` implement 0x0201 legacy
  30-byte attitude plus schema=2 80-byte DualAHRS. `dual_ahrs.c:958-984`
  gives exact 80-byte offsets.
- `bsp_uart.c:186-218` defines 0xF000 payload as source, level, timestamp LE32,
  text length LE16, and up to 96 bytes of text (maximum payload 104 bytes).
- `command_bridge.c:275-335` validates and maps telemetry lengths before placing
  unchanged payload bytes into the separate App BLE envelope; it does not alter
  SCBP payload units or field order.

## Control, ACK, retry

- `scbp_message_flags()` marks control/calibration messages ACK_REQUIRED and
  stream telemetry LOG/IMU/RADAR messages STREAM_DATA. `0x0005` ACK payload is
  `{ack_msg_id LE16, ack_seq, result, error_code}` (5 bytes); matching requires
  ACK flag, expected source/destination, pending message ID, and exact request
  sequence. `0x0006` ERROR payload is `{source,error,msg_id LE16,seq}` (5).
- STM `imu_boot_manager.c` uses 500 ms event ACK deadlines and max retry 3 for
  CAL_EVENT notifications. S3 `radar_calibration_manager.c` uses 500 ms
  RADAR_PWM_READY ACK deadlines and max retry 3. No generic timer/retry engine
  exists in the frame codec; reserved control IDs have no current business
  handler or payload contract.
- STM link freshness warning is 2000 ms and radar-status staleness threshold is
  3000 ms (`s3_service.c`). S3 calibration event deadlines are 75 s for static,
  22 s per vibration level, and an 11 s not-before guard for repeated id=2;
  these are state-machine guards, not frame retransmission intervals.
