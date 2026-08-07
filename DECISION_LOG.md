# Smart_Car Decision Log

This is the durable record of accepted boundaries. A decision records what was
chosen and why; it does not turn an unverified plan into runtime evidence.

## 2026-08-07: Knowledge Layers

- **Problem:** central docs, module README files, nested repositories, and
  vendor material compete as possible Codex context.
- **Candidates:** move everything; add indexes only; authoritative layer plus
  history/reference layers.
- **Decision:** use `.codex/` for startup rules and stable memory, root status
  and decisions for current governance, `docs/` for canonical engineering
  pages, and explicit history/reference classifications for the rest.
- **Reason:** preserves ownership and provenance while preventing stale plans
  from masquerading as current truth.
- **Impact:** nested repositories and third-party docs stay in place and are
  indexed rather than silently promoted.

## 2026-08-03: Active IMU

- **Problem:** BMI323 startup diagnostics blocked the intended runtime path.
- **Candidates:** keep both sensors active; mask failures; use the existing
  LSM303 path and retain BMI323 for paused diagnostics.
- **Decision:** LSM303 is the current primary IMU path; BMI323 is paused.
- **Reason:** current `imu_manager.c` initializes LSM303 and explicitly logs
  `BMI323 SKIPPED`; prior evidence isolated BMI323 startup as a blocking
  boundary.
- **Impact:** calibration/filter/attitude documents must describe LSM303 as
  active and BMI323 as paused, without deleting its driver.

## 2026-07-29: Frozen Hardware Allocation

- **Problem:** motor PWM, encoder, sensor, debug, and transport functions
  compete for finite STM32 pins and timer channels.
- **Candidates:** repurpose frozen nets; claim unsupported timer mappings; keep
  the validated allocation and record limitations.
- **Decision:** retain TIM3 PC6..PC9 for PWM, RF TIM1 PA8/PA9 and RB TIM2
  PA15/PB3 as timer encoder pairs, I2C4 PD12/PD13 for LSM303, SPI1 PA5/PA6/PA7
  plus PC4 CS for BMI323, and SWD PA13/PA14.
- **Reason:** current IOC/generated source and resource audit show LF/LB are
  not valid timer TI1/TI2 pairs on their frozen nets.
- **Impact:** no document or code change may invent four-wheel timer encoder
  support or silently repurpose SWD/GPIO.

## 2026-08-03: STM UART Ownership

- **Problem:** logs and gateway transport use different UART paths.
- **Decision:** USART1 PA9/PA10 remains the STM32 CH340/debug log path;
  USART2 PA2/PA3 is the STM32-S3 transport path; S3 uses UART2 GPIO17/18.
- **Reason:** current generated MSP, `bsp_uart`, `uart_link.h`, and S3
  `stm_uart.h` establish separate ownership.
- **Impact:** never route SmartCar transport through the logger UART, and do not
  treat legacy PD3/PD4 IOC labels as current serial proof.

## 2026-08-03: Radar Ownership

- **Problem:** radar control and real-time motion could be conflated.
- **Decision:** S3 owns radar UART1/GPIO44 receive and GPIO4 PWM; STM32 retains
  vehicle motion authority.
- **Reason:** current `radar_uart.h` and `main.c` initialize these paths, while
  the system architecture assigns low-level actuation to STM32.
- **Impact:** radar readiness is not equivalent to motor readiness or physical
  radar rotation.

## 2026-08-04: BLE UUIDs

- **Problem:** App and gateway need a stable GATT discovery contract.
- **Decision:** device name `SmartCar_S3`; service FFE0; FFE1 write, FFE2 notify,
  FFE3 log notify.
- **Reason:** current `BLEManager.swift` and `s3_ble.c` agree.
- **Impact:** UUID changes require synchronized App/S3 updates and a live BLE
  capture; source agreement alone is not acceptance.

## 2026-07-30: ROS2 and Architecture Boundary

- **Problem:** later SLAM/navigation work could bypass the embedded control
  safety model.
- **Candidates:** run ROS2 as direct motor owner; place all sensing in ROS2; or
  retain STM32 final authority with S3 as gateway and ROS2 as future host.
- **Decision:** STM32 remains final motion authority, S3 retains radar/gateway
  ownership, and ROS2_WIN is a planned host-side autonomy/map domain.
- **Reason:** it preserves deterministic local safety and separates radar
  transport from host-level planning.
- **Impact:** ROS2 can produce only an explicitly admitted future intent; it
  does not own direct motor pins, S3 UART hardware, or safety stop behavior.

## 2026-08-07: Two Frame Envelopes Must Stay Distinct

- **Problem:** documents use “AA55” for incompatible byte layouts.
- **Decision:** document App BLE v1 (`AA | 01 | ... | 55`) and STM32-S3 source
  frames (`AA | 55 | 01 | ... | CRC`) as separate contracts until a verified
  bridge explicitly translates them.
- **Reason:** `SmartCarProtocol.swift` and both C `sc_frame.c` implementations
  have different offsets, tails, and type tables.
- **Impact:** no document may claim a single end-to-end protocol without naming
  the transport boundary and implementation evidence.

## Historical Decisions

Older hardware, A5/5A protocol, ROS2, and radar planning decisions are indexed
in [DOCUMENT_AUDIT_REPORT.md](DOCUMENT_AUDIT_REPORT.md) and
[docs/history/](docs/history/). They remain useful context but are not current
authority unless this log explicitly adopts them.
