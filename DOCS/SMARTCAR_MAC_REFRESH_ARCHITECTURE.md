# SmartCar macOS App Refresh Architecture

## Current refresh triggers before the change

- Every valid BLE frame appended to `BLEManager.decodedMessages` emitted its
  `@Published` change.
- Every valid frame mutated `BLEManager.vehicleState`, emitting a second
  `@Published` change.
- `SmartCarViewModel` subscribed to both publishers and assigned its own
  published `vehicleState` and frame counter.
- `CalibrationViewModel` scanned the entire decoded-message history on every
  message-array emission.
- `SmartCarViewModel` ran a global 10 Hz/20 Hz timer and called
  `objectWillChange.send()`, causing unrelated controls, cards, and the debug
  console to re-evaluate.
- `DebugConsole` received the live decoded-message array, so each packet could
  rebuild the console list.

## New refresh triggers

- BLE parsing still runs for every notification and writes only the latest
  values into `TelemetryStore` pending slots.
- Attitude snapshots publish at 20 Hz.
- IMU snapshots publish at 5 Hz.
- Vehicle status snapshots publish at 1 Hz.
- Calibration snapshots publish immediately on calibration status/bias events.
- Debug counters and last-packet type publish at 1 Hz.
- The debug log publishes only when `refreshLogs()` is invoked by the
  “Refresh Logs” button. The source remains the bounded `decodedMessages` ring.

## Frequency comparison

| Surface | Before | After |
| --- | --- | --- |
| Attitude / whole VM | Per packet plus global 10/20 Hz timer | 20 Hz attitude store |
| IMU | Per packet plus global timer | 5 Hz IMU store |
| Calibration | Full-history scan per packet | Event-triggered calibration store |
| Vehicle status | Per packet plus global timer | 1 Hz status store |
| Debug counters | Per packet | 1 Hz coalesced metrics |
| Debug log | Per packet | Manual snapshot only |
