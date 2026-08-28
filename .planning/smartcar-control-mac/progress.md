# Progress: SmartCar_Control_MAC Alignment

## 2026-08-23

- Completed read-only comparison of historical macOS and current iOS sources.
- User approved the top-level `SmartCar_Control_MAC/` location.
- User selected sequential restoration followed by incremental iOS parity.
- Restored the historical package into `SmartCar_Control_MAC/` using the Git
  snapshot without restoring the deleted generic `IOS_APP/` tree.
- Original macOS baseline build passed.
- Ported iOS protocol, telemetry, BLE queue/reconnect, control mode, chassis
  speed, independent wheel, and motion heartbeat behavior.
- Added macOS lifecycle notifications for background/hide/terminate zeroing.
- Added mode and chassis target-speed controls to the original macOS wheel card.
