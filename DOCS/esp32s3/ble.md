# BLE Module

## Function

Expose the `SmartCar_S3` GATT service for App writes, telemetry notifications,
and separate structured log notifications.

## Source Location

`ESPS3/components/s3_ble/s3_ble.c` and `include/s3_ble.h`; command/session
consumer in `ESPS3/components/smartcar_service/command_bridge.c`; App
counterparts in `IOS-APP/Sources/SmartCarIOS/Core/BLE/BLEManager.swift` and
`SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/BLE/BLEManager.swift`.

## Entry Functions

S3 `s3_ble_init`, `s3_ble_notify_send`, `s3_ble_log_notify_send`,
`s3_ble_set_rx_callback`, `s3_ble_set_ready_callback`; App
`BLEManager.startScanning`, `connectToDevice`, and receive pipelines.

## Inputs

CoreBluetooth writes on FFE1, CCC writes for FFE2/FFE3, GATT connection/MTU
events.

## Outputs

FFE2 data notifications, FFE3 log notifications, callback bytes, readiness
state, and bounded pending early logs.

## Public Interfaces

`s3_ble_init`, `s3_ble_notify_send`, `s3_ble_log_notify_send`,
`s3_ble_set_rx_callback`, and `s3_ble_set_ready_callback`.

## Dependencies

ESP-IDF Bluedroid/GATT, FreeRTOS, `smartcar_log`, and the App CoreBluetooth
manager.

## Contract

Service `0000FFE0-0000-1000-8000-00805F9B34FB`; RX FFE1 write; TX FFE2 notify;
logger FFE3 notify. Device name is `SmartCar_S3`. App-BLE V1/V2 framing,
session, ACK, timeout, and mapping are specified in
[`DOCS/protocol/app-ble-protocol-v2.md`](../protocol/app-ble-protocol-v2.md).

## Current Status

GATT table, advertising, CCC handling, notification fragmentation, V2 session
admission, and App discovery identifiers are source-established. Live BLE
behavior is unverified.

## Known Issues

Notification readiness is not the same as telemetry relay readiness. Protocol
builds do not prove ATT MTU negotiation, packet delivery, or UART/motor stop
behavior.

## Modification Notes

Keep parser work off the GATT event path where possible. Preserve CCC readiness,
MTU chunking, pending-log bounds, and MainActor delivery on the App side.
