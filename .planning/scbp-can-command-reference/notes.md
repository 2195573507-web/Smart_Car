# Findings: STM32-S3 SCBP-CAN Command Reference

## Confirmed Source Facts

- Current STM32H757 <-> ESP32-S3 UART2 path is shared SCBP-CAN in `Common/SCBP_CAN/`.
- UART is 921600 baud, 8N1, no hardware flow control; source mapping is STM USART2 PA2/PA3 to S3 UART2 GPIO17/GPIO18.
- Frame is `5A A5 | CAN_ID_LE | FLAGS | LEN | HCS | SEQ | PAYLOAD | FCS_LE | 0D 0A`; HCS is CRC-8-ITU over CAN_ID/FLAGS/LEN and FCS is CRC16-MODBUS over payload only.
- Message field is 10 bits. Active constants are CAL_EVENT 0x001, ACK 0x005, ERROR 0x006, BOOT_READY 0x007, ATTITUDE 0x201, IMU_CAL_STATUS 0x202, IMU_TELEMETRY 0x207, RADAR_STATUS 0x301, RADAR_PWM_READY 0x302, and LOG 0x3F0.
- Shared link uses four pending slots, 500 ms ACK timeout, three retransmissions, exact `(ack_can_id, ack_seq)` correlation, REC/TEC thresholds, and bus-off recovery callbacks.
- STM emits BOOT_READY/CAL_EVENT/calibration status/IMU telemetry/DualAHRS/log; S3 receives and dispatches those messages. S3 emits RADAR_PWM_READY and RADAR_STATUS.
- Current STM `s3_service_on_frame()` handles RADAR_PWM_READY but has no RADAR_STATUS consumer branch. The reference documents this as a source boundary rather than claiming an STM status consumer.
- S3 reconstructs App BLE envelopes for selected messages. Raw SCBP-CAN bytes are not App BLE frames. SmartCar log notifications are a third, separate envelope.
- ATTITUDE is exactly 80 bytes, schema 2, little-endian fields, radians for Euler/delta values, quaternion order w/x/y/z; S3 validates length/schema/reserved bytes and preserves payload bytes into App type 0x11.
- STM runtime scheduling is 100 ms for one IMU_TELEMETRY frame per sensor and 50 ms for ATTITUDE when payload packing succeeds.

## Historical Exclusions

Historical V3 AA55 framing, SC_TYPE adapters, PING/PONG, 0x0200, 0x0208,
0x0401, 0xF000, legacy 30-byte ATTITUDE, and old bias/result records are not
active SCBP-CAN contracts.

## Verification To Run

- Markdown link and formatting checks.
- Compare every active ID, payload size, direction, and cadence in the new doc against `scbp_protocol_defs.h`, `s3_service.c`, `command_bridge.c`, `imu_boot_manager.c`, and `imu_runtime.c`.
- Confirm existing dirty files remain untouched.

## Verification Result

- Shared C11 host test passed with `-Wall -Wextra -Werror -pedantic`.
- Changed-document relative links passed a Ruby path check.
- `git diff --check` passed.
- BMI323 telemetry vector was corrected to `rad/s` after checking the active
  `bmi323_gyro_raw_to_rads()` conversion in `imu_manager.c`.
