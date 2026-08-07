# Smart_Car System Architecture

## Function

Define ownership, boundaries, and the current implementation state of the
Smart_Car system. This is the canonical overview; protocol bytes and module
details live in linked pages.

## Current Stage

Phase 1 source exists for STM32H757, ESP32-S3, and the macOS control App.
Phase 2 radar/ROS2/SLAM/navigation is reserved. The diagrams describe source
boundaries, not end-to-end acceptance.

## Hardware Relationship

```mermaid
flowchart LR
    STM["STM32H757\nCM7 / CM4"]
    LSM["LSM303\nI2C4"]
    BMI["BMI323\nSPI1, paused"]
    MOTOR["Motor drivers\nTIM3 PWM + GPIO direction"]
    ENC["Encoders\nRF/RB timer pairs"]
    S3["ESP32-S3\nUART2 + BLE + radar"]
    RADAR["YDLIDAR X3/X3 Pro\nUART1 RX GPIO44"]
    APP["macOS Control App\nCoreBluetooth"]
    ROS["ROS2_WIN\nplanned host"]
    LSM --> STM
    BMI -. retained .-> STM
    ENC --> STM
    STM --> MOTOR
    STM <-->|"USART2 PA2/PA3 <-> S3 GPIO17/18"| S3
    RADAR --> S3
    S3 <-->|"BLE FFE0 / FFE1-FFE3"| APP
    S3 -. planned Wi-Fi .-> ROS
```

## Software Relationship

```mermaid
flowchart TB
    subgraph STM32[STM32H757]
      HAL["Generated HAL/Core"] --> BSP["BSP"]
      BSP --> DRV["LSM303 / BMI323 / motor / encoder"]
      DRV --> IMU["IMU manager"]
      IMU --> CAL["Calibration and boot manager"]
      CAL --> FIL["Filter"]
      FIL --> ATT["Attitude"]
      ATT --> SVC["UART link and S3 service"]
      RTOS["FreeRTOS tasks"] --> IMU
      RTOS --> SVC
    end
    subgraph GATEWAY[ESP32-S3]
      SUART["stm_uart"] --> SP["STM-S3 frame parser"]
      SP --> SS["smartcar_service"]
      RAD["radar_uart / parser / control"] --> SS
      BLE["s3_ble GATT"]
      SS -. current relay gaps .-> BLE
    end
    subgraph APP["macOS SwiftUI"]
      CB["BLEManager"] --> PAR["SmartCarProtocol parser"]
      PAR --> STORE["TelemetryStore / DeviceLogStore"]
      STORE --> UI["Control / Developer views"]
    end
    SVC <--> SUART
    BLE <--> CB
```

## Data Flow

```mermaid
flowchart LR
    SENSOR["LSM303 sample"] --> CAL["calibration"] --> FILTER["filter"] --> ATT["attitude"]
    ATT --> STMFRAME["STM-S3 frame source"] --> UART["USART2/UART2"] --> S3FRAME["S3 parser"]
    S3FRAME -. relay not proven .-> BLE["BLE notify"] --> APP["App telemetry"]
    RADAR["Radar bytes"] --> S3RADAR["S3 radar parser"] --> STATE["radar state"]
    CONTROL["App control intent"] -. callback/parser not connected .-> S3CTRL["S3 command bridge"] -.-> STMCTRL["STM control authority"]
```

## Control Flow

```mermaid
sequenceDiagram
    participant A as App
    participant B as S3 BLE
    participant G as S3 gateway
    participant S as STM32
    A->>B: write control frame
    B-->>G: RX callback (registration currently unproven)
    G->>S: admitted STM-S3 frame (bridge incomplete)
    S->>S: validate locally and apply safety policy
    S-->>G: status/ACK/telemetry source frame
    G-->>B: notify relay (current source path incomplete)
    B-->>A: notification
```

## Ownership

| Domain | Owner | Must not be delegated |
| --- | --- | --- |
| Final motion and local stop | STM32H757 | App, BLE, ROS2, or radar gateway |
| Sensor sampling/calibration/attitude | STM32H757 | App display or S3 inference |
| STM-S3 transport and gateway state | ESP32-S3 | App direct hardware access |
| Radar UART/PWM/parsing | ESP32-S3 | STM32 real-time motor loop |
| Operator UX | macOS App | Safety authority |
| SLAM/navigation/autonomy | ROS2_WIN, future | Direct motor electrical path |

## Known Gaps

- `s3_ble_set_rx_callback()` is defined but no current registration call is
  visible; App command admission is therefore incomplete.
- `imu_bridge_handle()` is empty; telemetry relay is not proven.
- App and STM-S3 frame layouts differ and require an explicit translation or a
  confirmed contract before integration.
- Physical UART, BLE, radar, sensor, motor, and vehicle evidence is absent from
  this static documentation task.
