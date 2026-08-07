# BLE Module

## Function

Expose the `SmartCar_S3` GATT service for App writes, telemetry notifications,
and separate structured log notifications.

## Source Location

`ESPS3/components/s3_ble/s3_ble.c` and `include/s3_ble.h`; App counterpart in
`IOS_APP/SmartCar_Control_MAC/.../BLE/BLEManager.swift`.

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
logger FFE3 notify. Device name is `SmartCar_S3`.

## Current Status

GATT table, advertising, CCC handling, notification fragmentation, and App
discovery identifiers are source-established. Live BLE behavior is unverified.

## Known Issues

The RX callback API is not connected to a current command parser in the S3
source. Notification readiness is not the same as telemetry relay readiness.

## Modification Notes

Keep parser work off the GATT event path where possible. Preserve CCC readiness,
MTU chunking, pending-log bounds, and MainActor delivery on the App side.
