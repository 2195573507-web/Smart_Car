# SCBP-V3 Migration Report (Historical)

Status: superseded by the SCBP-CAN migration.

This report records a previous V3 migration and must not be used as an active
protocol, UART baud-rate, message-ID, payload-width, or validation reference.
The current STM32H757 <-> ESP32-S3 implementation uses the shared SCBP-CAN
module in `Common/SCBP_CAN/` at 921600 baud. Consult
[protocol.md](protocol.md) and [stm32-s3-transport.md](stm32-s3-transport.md)
instead.

No historical build result in this report establishes present hardware or
end-to-end behavior.
