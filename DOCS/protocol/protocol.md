# SRP v4 Protocol Module

The active STM32H757 CM7 <-> ESP32-S3 UART2 contract is SRP v4. The shared
implementation is `Common/SRP/`; both firmware targets compile the same
codec, stream parser, wire helpers and link manager.

Read [`../SRP_v4_Spec.md`](../SRP_v4_Spec.md) for the wire contract,
[`../UART_Config_Guide.md`](../UART_Config_Guide.md) for DMA/IDLE transport,
and [`../Integration_Manual.md`](../Integration_Manual.md) before adding a
message. App BLE remains a separate envelope.

The SRP frame is:

```text
AA 55 | LEN_LE | HEADER_LE | PAYLOAD | CRC16-CCITT-FALSE_LE | 0D 0A
```

Header priority/type/sequence/flags are accessed through `SRP_HDR_*` macros;
no C bit-fields or serialized source/destination IDs are used. Payload lengths
and type assignments are in `Common/SRP/include/srp_registry.h`.

Host codec tests and CM7/ESP-IDF builds are source evidence. UART, DMA, BLE,
EMERGENCY latency, dynamic baud and long-duration CRC criteria require a
flashed bench setup and live captures.
