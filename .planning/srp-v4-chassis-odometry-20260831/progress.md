# Progress: SRP v4 Chassis Odometry

## Implemented

- Added chassis flag mask, payload offset assertions, and the shared Windows/
  firmware 36-byte golden SRP frame with CRC `0xc07f`.
- Added atomic MotorBoard MSPD snapshot metadata: corrected speeds, monotonic
  timestamp, sequence, and validity.
- Added an allocation-free odometry estimator and 50 ms low-priority publisher
  task, started only after MotorBoard readiness.
- Added strict CM7 payload validation before SRP send.
- Added S3 service forwarding, strict semantic validation, one static
  latest-only chassis slot, stats, and bounded-fair scheduling.
- Updated protocol, odometry, module-index, and design documentation.

## Verified

- Common/SRP host suite: PASS.
- CM7 odometry normal and ASAN/UBSAN host tests: PASS.
- S3 radar host suite: PASS.
- S3 chassis queue ASAN/UBSAN test: PASS.
- CM7 canonical Debug build: PASS, Flash 190648 B, RAM 62624 B.
- CM7 ELF SHA-256: `2d39f084eff77675e61305cae53df5fdb7da0b9304218daaafe61233b38702ed`.
- ESP-IDF 5.5.4 build/size: PASS, app binary `0x12ced0`, 83% app partition free.
- S3 BIN SHA-256: `1613caeb3a7e295e3940e52bf5de188607a9cbd04a4f11783b142be442185a5b`.
- Targeted `git diff --check`: PASS.

## Not Verified

- No firmware was flashed.
- No live SRP `0x15`, S3RD telemetry type 2, ROS `/odom`, TF, or map capture was
  performed.
- MSPD scale/sign, Primary-yaw convention, slip, and map quality require
  staged hardware acceptance.
