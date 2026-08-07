# Smart_Car Data Flow

## IMU Data

```mermaid
flowchart LR
    ACC["LSM303 accel"] --> RAW["imu_manager raw snapshot"]
    MAG["LSM303 mag"] --> RAW
    RAW --> CAL["imu_calibration_apply"]
    CAL --> FILTER["imu_filter_update"]
    FILTER --> ATT["attitude_update when ready"]
    RAW --> LOG["STM logger/status output"]
    ATT -. STM-S3 frame .-> UART["USART2/UART2"]
```

The manager updates accel and magnetometer paths independently, then publishes
a complete raw snapshot to calibration only when the cycle reports success.
Separate getter calls are not an atomic hardware snapshot.

## Control Data

```mermaid
flowchart LR
    UI["App control intent"] --> BLE["BLE write"] --> S3["S3 command admission"]
    S3 --> UART["STM-S3 UART frame"] --> STM["STM local validation"]
    STM --> SAFE["local safety / actuator owner"]
    SAFE --> ACK["ACK/status"]
```

The S3 admission and relay blocks are source gaps in the current checkout; the
diagram defines the intended ownership, not a completed route.

## Radar Data

```mermaid
flowchart LR
    SENSOR["X3/X3 Pro TX"] --> GPIO["S3 GPIO44"] --> UART1["S3 UART1"]
    UART1 --> PARSER["radar_parser"] --> RADARSTATE["radar measurements/state"]
    PWM["S3 GPIO4 LEDC PWM"] --> SENSOR
    RADARSTATE -. future gateway .-> ROS["ROS2 LaserScan / SLAM"]
```

Current S3 source reads and logs raw radar bytes. The exact model/checksum and
ROS2 integration remain unverified or planned.

## Status and Logs

- STM32 text logs use USART1 and the independent SmartCar Logger path.
- STM-S3 structured log frames use the C source frame and S3 log bridge.
- BLE log notifications use FFE3 and `SmartCarLogParser` on the App.
- App telemetry stores are MainActor-owned; BLE/parser work is isolated on the
  receive pipeline queue.
