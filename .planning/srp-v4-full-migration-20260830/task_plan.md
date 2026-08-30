# SRPv4 Full Migration and Legacy Cleanup

## Goal

Against branch 3, make the active STM32H757 <-> ESP32-S3 UART2 protocol use
SRPv4 end to end, remove executable legacy SCBP-CAN sources and references,
and leave retained historical documents explicitly deprecated. Preserve the
independent App BLE, YDLIDAR, SmartCarLog, and experimental S3RD envelopes.

## Phases

1. **Baseline and scope** - confirm branch-3 contract, current diff, active
   source/build references, and protected protocol boundaries. (complete)
2. **Implementation review** - inspect the existing SRPv4 migration and repair
   compile/API/contract gaps without unrelated refactors. (complete)
3. **Build and host verification** - run SRP host tests, ESP-IDF build, CM7
   canonical Debug build, and ROS2 decoder tests or local compile checks.
   (complete)
4. **Legacy cleanup** - update active docs/indexes and mark retained historical
   SCBP material clearly deprecated; prove active source/build references are
   zero. (complete)
5. **Final evidence** - run diff/scan checks and report exact residuals,
   validation boundaries, and files changed. (complete)

## Invariants

- STM USART2 PA2/PA3 <-> S3 UART2 GPIO17/18 remains 921600 8N1.
- SRPv4 is the only active STM-S3 UART protocol.
- App BLE `AA 01 ... 55`, YDLIDAR `AA 55`, SmartCarLog, and S3RD remain
  separate contracts.
- CM7 retains attitude, BUS_OFF, timeout, emergency-stop, and MotorBoard
  motion authority.
- Wheel order remains `[RR, RF, LR, LF]`, track width `193.0 mm`.

## Errors

| Error | Attempts | Resolution |
|---|---:|---|
| CM7 CMake could not find `Application/Chassis` sources | 1 | Confirmed branch 4 omits files present on origin/3; restore the branch-3 chassis implementation required by the migrated SRP command path |
| CM7 build invoked with incorrect working directory | 1 | Corrected to `STM32H757/CM7`; no source impact |
