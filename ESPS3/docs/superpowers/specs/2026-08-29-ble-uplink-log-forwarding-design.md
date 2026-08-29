# BLE Uplink Log Forwarding Design

## Scope

Forward the radar Wi-Fi/TCP uplink task's operational events through the
existing S3 BLE log characteristic. Keep the existing ESP-IDF console logs,
BLE UUIDs, S3RD packet format, radar UART path, and STM32 control path
unchanged.

## Data flow

`radar_uplink.c` continues to emit `ESP_LOG*` records for the console and also
emits a short `s3_log_*` record for these events:

- Wi-Fi disconnect and credential rotation
- Wi-Fi configuration failure
- TCP connection success
- TCP send failure
- uplink configuration or startup failure

The BLE component encodes these records using the existing SmartCar log frame
and sends them through the existing FFE3 notification path. If no BLE client
has enabled log notifications, the existing bounded pending queue is used.

## Real-time and failure behavior

BLE notification failure is observational only. It must not block the radar
UART task, delay the uplink retry loop beyond the existing scheduling, or
change STM32/UART2 behavior. Passwords are never included in BLE or console
messages.

## Verification

- Run `main/radar/tests/run_host_tests.sh`.
- Build with ESP-IDF 5.5.4 and the current uplink-enabled configuration.
- Review that every new BLE log call is short, bounded, and outside Wi-Fi event
  callbacks.
