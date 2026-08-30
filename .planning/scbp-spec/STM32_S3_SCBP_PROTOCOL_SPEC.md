# STM32H757 <-> ESP32-S3 SCBP 串口通信协议规格书

> 历史/已废弃规格书：本文记录 SRPv4 全量切换前的 SCBP 快照，仅供追溯。
> 当前 UART2 活动协议以 `Common/SRP`、`DOCS/SRP_v4_Spec.md` 和现行构建入口为准。

**审计日期：** 2026-08-19
**适用边界：** 当前工作区源码中的 STM32H757 CM7 与 ESP32-S3 UART2 链路。
**证据等级：** 源码/生成配置静态确认；未包含烧录、示波器、逻辑分析仪、UART 抓包或整链路运行验收。

## 0. 结论摘要

- 活动串口协议是 **SCBP-V3**，不是旧的单字节 TYPE 帧。
- 帧固定以 `AA 55` 起始，固定开销 14 字节，最大载荷 128 字节，长度字段为小端 `uint16`。
- 物理链路两端均为 `115200 baud, 8 data bits, 1 stop bit, no parity, no hardware flow control`。
- STM32 使用 `USART2`（PA2 TX、PA3 RX）；S3 使用 `UART2`（GPIO17 TX、GPIO18 RX）。方向为交叉连接：STM PA2 -> S3 GPIO18，S3 GPIO17 -> STM PA3。
- CRC 是 CRC16-MODBUS：初值 `0xFFFF`，反射多项式 `0xA001`，覆盖 `VER` 到最后一个载荷字节，不覆盖 `AA 55` 和 CRC 自身，CRC 小端发送。
- 当前业务帧主要是 STM32 遥测/校准状态 -> S3，以及 S3 雷达 PWM 调度 -> STM32。`VERSION/RESET/MOTOR_CONTROL/PWM_SET/PARAM_SET/RADAR_CONTROL/CAL_START` 在头文件和优先级/标志表中定义，但当前服务代码没有业务处理器或确定的载荷长度。

## 1. 源码依据与责任边界

| 责任 | STM32H757 | ESP32-S3 |
| --- | --- | --- |
| 帧编解码、CRC、序列诊断 | `STM32H757/Middleware/Communication/SmartCar_Frame/sc_frame.c/.h` | `ESPS3/components/smartcar_protocol/frame.c`, `parser.c`, `include/*.h` |
| 原始 UART | `Middleware/Communication/UART_Link/uart_link.c/.h` | `components/stm_uart/stm_uart.c`, `include/stm_uart.h` |
| 链路服务/业务分派 | `Middleware/Communication/Services/s3_service.c/.h` | `components/smartcar_service/command_bridge.c` |
| 校准状态与发送 | `Middleware/Calibration/imu_boot_manager.c/.h` | `components/smartcar_service/radar_calibration_manager.c/.h` |
| IMU/姿态载荷生产 | `Application/RTOS/imu_runtime.c`, `Attitude/DualAHRS/dual_ahrs.c` | 只校验并转发，不改 SCBP 载荷 |
| 日志载荷 | `BSP/UART/bsp_uart.c`, `Services/log_service.h` | `smartcar_service/log_bridge.c` |

S3 还把已验证的 SCBP 载荷放入独立的 App BLE envelope；该 BLE envelope 不是 STM-S3 SCBP，见第 8 节。

## 2. 物理层

### 2.1 串口参数

| 项目 | STM32 `uart_link.c` | S3 `stm_uart.c` | SCBP 要求 |
| --- | --- | --- | --- |
| 外设 | `USART2` | `UART_NUM_2` | 同一条 STM-S3 链路 |
| 波特率 | `115200` | `115200` | 115200 |
| 数据位 | `UART_WORDLENGTH_8B` | `UART_DATA_8_BITS` | 8 |
| 停止位 | 1 | 1 | 1 |
| 校验 | `UART_PARITY_NONE` | `UART_PARITY_DISABLE` | 无 |
| 流控 | `UART_HWCONTROL_NONE` | `UART_HW_FLOWCTRL_DISABLE` | 无硬件 RTS/CTS |
| STM 采样/时钟 | oversampling 16，prescaler 1 | ESP-IDF 默认 UART 时钟 | 不影响线协议 |

### 2.2 引脚与缓冲

- STM 生成的 AF7 USART2 为 PA2/PA3；IOC 标签分别为 `S3_UART_TX_TO_ESP32` 和 `S3_UART_RX_FROM_ESP32`。
- S3 `stm_uart.h` 固定 GPIO17 为 TX、GPIO18 为 RX。
- STM RX：`HAL_UARTEx_ReceiveToIdle` 每次最多读 128 字节，进入 512 字节环形缓冲；环满时丢弃最旧字节并增加 overflow/drop 计数。
- S3：ESP-IDF 驱动 RX/TX 缓冲各 4096 字节，服务层再使用 4096 字节存储环；任务一次最多取 256 字节。
- 编解码器最大帧为 `SC_FRAME_MAX_SIZE = 14 + 128 = 142` 字节；STM/S3 parser 的内部字节数组均为 142 字节。
- UART TX 不是协议重试：STM TX mutex/HAL 调用上限 20 ms；S3 `uart_write_bytes` 后 `uart_wait_tx_done` 上限 100 ms。

## 3. SCBP-V3 帧封装

### 3.1 线格式

```text
AA | 55 | VER | PRIORITY | SRC | DST | MSG_ID_L | MSG_ID_H |
SEQ | FLAGS | LEN_L | LEN_H | PAYLOAD[LEN] | CRC_L | CRC_H
```

| 偏移 | 字段 | 宽度 | 编码/含义 |
| ---: | --- | ---: | --- |
| 0 | SOF0 | 1 | 固定 `0xAA` |
| 1 | SOF1 | 1 | 固定 `0x55` |
| 2 | VER | 1 | 当前固定 `0x01` |
| 3 | PRIORITY | 1 | `0 emergency`, `1 realtime`, `2 normal`, `3 debug` |
| 4 | SRC | 1 | 发送节点 |
| 5 | DST | 1 | 接收节点或广播 |
| 6 | MSG_ID_L | 1 | 16 位消息 ID 低字节 |
| 7 | MSG_ID_H | 1 | 16 位消息 ID 高字节 |
| 8 | SEQ | 1 | 发送端 8 位计数器，`0..255` 回绕 |
| 9 | FLAGS | 1 | ACK/错误/重试/流数据/配置标志 |
| 10 | LEN_L | 1 | 载荷长度低字节 |
| 11 | LEN_H | 1 | 载荷长度高字节 |
| 12.. | PAYLOAD | LEN | 消息定义的二进制载荷 |
| 12+LEN | CRC_L | 1 | CRC16-MODBUS 低字节 |
| 13+LEN | CRC_H | 1 | CRC16-MODBUS 高字节 |

总长度为 `14 + LEN`。所有 `uint16/uint32` 和所有 IEEE-754 `float32` 均按小端字节序逐字段序列化。源码使用 `put_u32_le/put_float_le`，不直接发送 C 结构体的编译器填充。

### 3.2 节点 ID

| 节点 | ID |
| --- | ---: |
| STM32H757 | `0x01` |
| ESP32-S3 | `0x02` |
| App（仅作为节点命名保留） | `0x03` |
| ROS2（保留） | `0x04` |
| C5（保留） | `0x10` |
| Broadcast | `0xFF` |

当前 STM32 默认 `SRC=0x01,DST=0x02`；当前 S3 默认 `SRC=0x02,DST=0x01`。

### 3.3 FLAGS

| 位 | 宏 | 含义 |
| ---: | --- | --- |
| 0 | `ACK_REQUIRED=0x01` | 接收端应返回 `0x0005` |
| 1 | `ACK_FRAME=0x02` | 当前帧是 ACK |
| 2 | `ERROR_FRAME=0x04` | 当前帧是 ERROR |
| 3 | `RETRY=0x08` | 重发标志，当前管理器重发时没有额外设置该位 |
| 4 | `STREAM_DATA=0x10` | 连续遥测/日志流 |
| 5 | `CONFIG=0x20` | 配置帧 |
| 6..7 | `0xC0` | 保留；解码器拒绝非零值 |

### 3.4 CRC

```c
crc = 0xFFFF;
for each byte b in frame[2 .. 11 + LEN] {
    crc ^= b;
    repeat 8 times:
        crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
}
```

这是 CRC16-MODBUS 的 reflected 实现；`AA 55`、`CRC_L`、`CRC_H` 不参与计算。

### 3.5 规范性 C 视图

以下是规格书的无填充视图；工程实现仍应使用逐字段读写，不能依赖 ABI：

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t  sof0;       /* 0: 0xAA */
    uint8_t  sof1;       /* 1: 0x55 */
    uint8_t  version;    /* 2 */
    uint8_t  priority;   /* 3 */
    uint8_t  src;        /* 4 */
    uint8_t  dst;        /* 5 */
    uint16_t msg_id;     /* 6, LE */
    uint8_t  seq;        /* 8 */
    uint8_t  flags;      /* 9 */
    uint16_t length;     /* 10, LE */
    /* payload[length], then uint16_t crc16_le at offset 12+length */
} scbp_frame_header_t;

typedef struct {
    uint16_t acknowledged_msg_id; /* 0, LE */
    uint8_t  acknowledged_seq;    /* 2 */
    uint8_t  result;              /* 3: 0 OK, 1 failed */
    uint8_t  error_code;          /* 4 */
} scbp_ack_payload_t;

typedef struct {
    uint8_t source;               /* 0 */
    uint8_t error_code;           /* 1 */
    uint16_t message_id;          /* 2, LE */
    uint8_t sequence;             /* 4 */
} scbp_error_payload_t;
#pragma pack(pop)
```

## 4. 完整 MSG_ID 清单

“当前使用”表示在 STM/S3 业务路径中有实际发送、接收或转发；“仅定义”表示目前只出现在 ID/优先级/标志定义中，未找到业务处理器或确定载荷布局。

| Msg ID | 消息名称 | 当前方向 | 当前载荷长度 | 功能/状态 |
| :--- | :--- | :--- | :---: | :--- |
| `0x0001` | PING | 双向可编码；S3 当前接收 | 0 | S3 收到后回 `PONG`；无通用心跳任务 |
| `0x0002` | PONG | S3 -> STM（当前响应） | 0 | PING 响应；STM 仅记录接收 |
| `0x0003` | VERSION | 未使用 | 未定义 | 仅定义，无业务处理器 |
| `0x0004` | RESET | 未使用 | 未定义 | 仅定义，无业务处理器 |
| `0x0005` | ACK | 双向 | 5 | ACK `0x0302` 或 `0x0401`；按 ID+SEQ 关联 |
| `0x0006` | ERROR | 双向可发 | 5 | 对已解码但不支持/长度错误的单播帧回复 |
| `0x0007` | BOOT_READY | STM -> S3 | 2 | `state,result`；`state=1` 为等待 PWM=0 同步 |
| `0x0100` | MOTOR_CONTROL | 未使用 | 未定义 | 定义为 ACK_REQUIRED/Realtime，暂无实现 |
| `0x0101` | PWM_SET | 未使用（保留活动命令 ID） | 未定义 | 不能把它当作 `0x0302` 的 READY 替代 |
| `0x0102` | PARAM_SET | 未使用 | 未定义 | 定义为 ACK_REQUIRED，暂无实现 |
| `0x0200` | IMU_STATUS | STM -> S3 -> App | 38（S3/App 兼容 43） | 旧式 IMU 状态；当前 STM 发 38 |
| `0x0201` | ATTITUDE | STM -> S3 -> App | 30 或 80 | 30 字节 legacy；80 字节 schema=2 DualAHRS |
| `0x0202` | IMU_CAL_STATUS | STM -> S3 -> App | 11 | 兼容校准进度帧 |
| `0x0203` | IMU_BIAS | STM -> S3 -> App | 12 | 三轴加速度偏置 |
| `0x0204` | VIBRATION_STATUS | STM -> S3 -> App | 17 | 旧式雷达振动 RMS |
| `0x0205` | IMU_CAL_RESULT | STM -> S3 -> App | 14 或 26 | LSM303 或 BMI323 标定结果 |
| `0x0206` | IMU_VIBRATION_PROFILE | STM -> S3 -> App | 26 或 42 | LSM303 或 BMI323 振动档案 |
| `0x0207` | IMU_TELEMETRY | STM -> S3 -> App | 30 | source-tagged 双 IMU 遥测 |
| `0x0208` | DUAL_IMU_STATUS | STM -> S3 -> App | 16 | 双 IMU 生命周期/进度 |
| `0x0300` | RADAR_CONTROL | 未使用 | 未定义 | 定义为 ACK_REQUIRED/Realtime，暂无实现 |
| `0x0301` | RADAR_STATUS | S3 -> STM，并转 App | 2 | `[online, speed_percent]`；由 S3 雷达控制产生 |
| `0x0302` | RADAR_PWM_READY | S3 -> STM；ACK STM -> S3 | 1 | `[speed_percent]`，包括 PWM=0 同步级 |
| `0x0400` | CAL_START | 未使用 | 未定义 | 定义为 ACK_REQUIRED/Realtime，暂无实现 |
| `0x0401` | CAL_EVENT | 双向 | 1 | STM 发送 id=1/2；S3 发送 id=3；均需 ACK |
| `0xF000` | LOG | STM -> S3 -> BLE | 8..104 | 8 字节日志头 + 最多 96 字节文本 |

### 4.1 由 `scbp_message_priority/flags` 得到的默认属性

- Realtime：`ACK`、`BOOT_READY`、`MOTOR_CONTROL`、`PWM_SET`、`PARAM_SET`、`RADAR_CONTROL`、`RADAR_PWM_READY`、`CAL_START`、`CAL_EVENT`、`ATTITUDE`。
- Debug：`LOG`、`ERROR`。
- Normal：其它 ID（包括 IMU 状态/校准遥测和 PING/PONG）。
- `ACK_REQUIRED` 默认施加于 `MOTOR_CONTROL/PWM_SET/PARAM_SET/RADAR_CONTROL/RADAR_PWM_READY/CAL_START/CAL_EVENT`。
- `STREAM_DATA` 默认施加于所有 IMU/RADAR 状态、`ATTITUDE` 和 `LOG`。

## 5. 关键载荷二进制布局

### 5.1 `0x0005` ACK（5 字节）

```c
#pragma pack(push, 1)
typedef struct {
    uint16_t ack_msg_id; /* offset 0, LE; 被确认的 SCBP MSG_ID */
    uint8_t  ack_seq;    /* offset 2; 被确认请求的 SEQ */
    uint8_t  result;     /* offset 3; 0=OK, 1=FAILED */
    uint8_t  error;      /* offset 4; 0=OK, otherwise SCBP_ERROR_* */
} scbp_ack_t;
#pragma pack(pop)
```

ACK 帧本身的 `MSG_ID=0x0005`、`FLAGS=ACK_FRAME`。ACK 的帧头 `SEQ` 是 ACK 发送端自己的新序列号；关联请求的序列号只出现在载荷 offset 2。

### 5.2 `0x0006` ERROR（5 字节）

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t  source;      /* offset 0; 产生错误的本端节点 */
    uint8_t  error_code;  /* offset 1 */
    uint16_t msg_id;      /* offset 2, LE; 出错的请求 ID */
    uint8_t  seq;         /* offset 4; 出错请求 SEQ */
} scbp_error_t;
#pragma pack(pop)
```

错误码：`0x01 UNKNOWN_MSG`、`0x02 INVALID_LENGTH`、`0x03 CRC`、`0x04 BUSY`、`0x05 TIMEOUT`、`0x06 NOT_READY`、`0x07 SENSOR`、`0x08 PARAM`。CRC/头错误不能信任源字段，因此只走 parser error callback，不回复 ERROR。

### 5.3 `0x0007` BOOT_READY（2 字节，STM -> S3）

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t state;   /* offset 0; 当前固定 WAIT_SYNC=1 */
    uint8_t result;  /* offset 1; 0=成功 */
} scbp_boot_ready_t;
#pragma pack(pop)
```

S3 只接受 `state=1,result=0` 作为雷达校准同步边界，然后进入 `PWM=0` 的 `RADAR_SET_PWM`。

### 5.4 `0x0202` IMU_CAL_STATUS（11 字节）

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t  stage;         /* 0 */
    uint8_t  radar_pwm;     /* 1; percent, 0..100 */
    uint32_t sample_count;  /* 2..5, LE; 当前样本数 */
    uint32_t sample_total;  /* 6..9, LE; 目标样本数 */
    uint8_t  error;         /* 10; imu_error_t 数值 */
} scbp_imu_cal_status_t;
#pragma pack(pop)
```

`stage` 当前值：`0 WAIT_RADAR_READY`、`1 STATIC_STABLE_WAIT`、`2 STATIC_SAMPLE`、`3 VIBRATION_STABLE_WAIT`、`4 VIBRATION_SAMPLE`、`5 COMPLETE`、`6 ERROR`。STM 用 `put_u32_le` 写入，S3/App 按 32 位计数计算展示进度。文档中的旧 7/8/9 字节布局仍是 App 兼容分支，不是当前 STM-S3 发送布局。

### 5.5 `0x0208` DUAL_IMU_STATUS（16 字节）

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t  phase;              /* 0; imu_phase_t */
    uint8_t  lsm_progress;       /* 1; percent */
    uint8_t  bmi_progress;       /* 2; percent */
    uint8_t  overall_progress;   /* 3; percent */
    uint8_t  error;              /* 4; imu_error_t */
    uint8_t  flags;              /* 5; completion/active/event bits */
    uint8_t  vibration_index;    /* 6; 0-based 20/40/60/80/100 index */
    uint8_t  radar_pwm;          /* 7; percent */
    uint32_t phase_start_time;   /* 8..11, LE; monotonic ms */
    uint32_t phase_end_time;     /* 12..15, LE; 0 while active */
} scbp_dual_imu_status_t;
#pragma pack(pop)
```

flags：bit0 `LSM_PHASE_COMPLETE`、bit1 `BMI_PHASE_COMPLETE`、bit2 `PHASE_ACTIVE`、bit3 `EVENT_WAITING`。`phase`：`0 IDLE, 1 INIT, 2 SELF_TEST, 3 STATIC_CALIBRATION, 4 VIBRATION_CAPTURE, 5 VERIFY, 6 READY, 7 FAILED`。

### 5.6 `0x0302` RADAR_PWM_READY（1 字节，S3 -> STM）

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t speed_percent; /* offset 0; 0..100 */
} scbp_radar_pwm_ready_t;
#pragma pack(pop)
```

它是 PWM 调度/就绪通知，不是 `0x0101 PWM_SET`。STM 收到后检查当前生命周期和期望百分比，并通过 `0x0005` ACK（兼容回调参数仍是 `[speed,result]` 两字节）。

### 5.7 `0x0401` CAL_EVENT（1 字节）

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t event_id; /* offset 0 */
} scbp_cal_event_t;
#pragma pack(pop)
```

| event_id | 名称 | 方向/时机 |
| ---: | --- | --- |
| `0x01` | `STATIC_CAL_DONE` | STM -> S3；静态窗口完成 |
| `0x02` | `VIBRATION_STEP_DONE` | STM -> S3；每个 PWM 档完成，ID 重复使用 |
| `0x03` | `COMPLETE` | S3 -> STM 或 STM -> S3 的完成通知路径；当前 S3 在五档完成后发送，STM 在 VERIFY 完成后也可发送 |

`CAL_EVENT_ACK` 是旧业务回调名，不是独立的 SCBP MSG_ID；线上使用 `0x0005` ACK 的 5 字节载荷。

### 5.8 `0x0201` ATTITUDE legacy（30 字节）

```c
#pragma pack(push, 1)
typedef struct {
    float    roll_rad;   /* 0..3, rad */
    float    pitch_rad;  /* 4..7, rad */
    float    yaw_rad;    /* 8..11, rad */
    float    roll_deg;   /* 12..15, degree */
    float    pitch_deg;  /* 16..19, degree */
    float    yaw_deg;    /* 20..23, degree */
    uint32_t timestamp_ms; /* 24..27, LE, monotonic ms */
    uint8_t  source;     /* 28; current LSM303 AHRS source=1 */
    uint8_t  valid;      /* 29; 0/1 */
} scbp_attitude30_t;
#pragma pack(pop)
```

STM 内部 AHRS 使用弧度；`imu_runtime.c` 只在序列化时计算角度值。`s3_service.c` 对旧 26 字节输入会补当前时间戳并重排为 30 字节；S3 对线上 30 字节只校验长度并原样转发。

### 5.9 `0x0201` schema=2 DualAHRS（80 字节）

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t  schema;          /* 0: 0x02 */
    uint8_t  flags;           /* 1; bit0 primary valid, bit1 redundant valid */
    uint16_t reserved;        /* 2..3, LE, must be 0 */
    uint32_t timestamp_ms;    /* 4..7, LE */
    uint32_t sample_sequence; /* 8..11, LE */
    float primary_roll;       /* 12..15, rad */
    float primary_pitch;      /* 16..19, rad */
    float primary_yaw;        /* 20..23, rad */
    float primary_qw;         /* 24..27 */
    float primary_qx;         /* 28..31 */
    float primary_qy;         /* 32..35 */
    float primary_qz;         /* 36..39 */
    float redundant_roll;     /* 40..43, rad */
    float redundant_pitch;    /* 44..47, rad */
    float redundant_yaw;      /* 48..51, rad */
    float redundant_qw;       /* 52..55 */
    float redundant_qx;       /* 56..59 */
    float redundant_qy;       /* 60..63 */
    float redundant_qz;       /* 64..67 */
    float delta_roll_rad;     /* 68..71, wrapped rad */
    float delta_pitch_rad;    /* 72..75, wrapped rad */
    float delta_yaw_rad;      /* 76..79, wrapped rad */
} scbp_dual_attitude80_t;
#pragma pack(pop)
```

S3 必须同时检查 `LEN==80`, `schema==2`, `reserved==0`；之后将 80 字节原样放入独立 App BLE `type=0x11`，不做单位或四元数顺序转换。

### 5.10 其它传感器/状态载荷

#### `0x0200` IMU_STATUS（当前 38 字节）

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t sensor_id; /* 0; 当前 legacy producer 写 0x02 表示 LSM303 */
    uint8_t online;    /* 1; 0/1 */
    float accel_x;     /* 2..5 */
    float accel_y;     /* 6..9 */
    float accel_z;     /* 10..13 */
    float gyro_x;      /* 14..17; 当前 LSM producer 为 0 */
    float gyro_y;      /* 18..21; 当前为 0 */
    float gyro_z;      /* 22..25; 当前为 0 */
    float mag_x;       /* 26..29 */
    float mag_y;       /* 30..33 */
    float mag_z;       /* 34..37 */
} scbp_imu_status38_t;
#pragma pack(pop)
```

当前源代码没有在这个 legacy 载荷上写单位标签；数值保持驱动输出单位。S3/App 兼容 43 字节历史变体，在 offset 38..42 另带 `calibration_state(u8), sample(u16), total(u16)`，但当前 STM `imu_runtime.c` 发送 38 字节。

#### `0x0203` IMU_BIAS（12 字节）

`float accel_bias_x` offset 0、`accel_bias_y` offset 4、`accel_bias_z` offset 8；IEEE-754 float32 LE，单位跟随校准数据的驱动单位。

#### `0x0204` VIBRATION_STATUS（17 字节）

offset 0=`radar_pwm(u8, percent)`；offset 1/5/9/13 为 `rms_x/rms_y/rms_z/total_rms(float32 LE)`。

#### `0x0205` IMU_CAL_RESULT（14 或 26 字节）

- LSM303 14 字节：`sensor_id(0)=1`、`flags(1)=ACCEL(0x01)`、加速度偏置 float xyz 位于 2/6/10。
- BMI323 26 字节：`sensor_id(0)=2`、`flags(1)=ACCEL|GYRO(0x03)`、加速度偏置位于 2/6/10，陀螺偏置位于 14/18/22。

#### `0x0206` IMU_VIBRATION_PROFILE（26 或 42 字节）

公共头：offset 0=`sensor_id`，1=`radar_pwm`，2=`sample_count(u32 LE)`，6=`timestamp(u32 LE)`。

- LSM303 26 字节：RMS accel xyz 在 10/14/18，total RMS 在 22。
- BMI323 42 字节：上述字段相同，另有 gyro RMS xyz 在 26/30/34，gyro total RMS 在 38。

#### `0x0207` IMU_TELEMETRY（30 字节）

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t  sensor_id; /* 0: IMU_SENSOR_LSM303=1, BMI323=2 */
    uint8_t  flags;     /* 1: bit0 accel, bit1 mag/gyro, bit2 online */
    uint32_t timestamp; /* 2..5, LE */
    float accel_x;      /* 6..9 */
    float accel_y;      /* 10..13 */
    float accel_z;      /* 14..17 */
    float vector_x;     /* 18..21; LSM=mag, BMI=gyro */
    float vector_y;     /* 22..25 */
    float vector_z;     /* 26..29 */
} scbp_imu_telemetry30_t;
#pragma pack(pop)
```

S3 bridge 保留 payload byte 1 的 validity/online 位，不转换数据；App 在未设置 bit2 时仍兼容由 bit0/bit1 推导 online 的旧行为。

#### `0x0301` RADAR_STATUS（2 字节）

`online(u8, offset 0, 0/1)`、`speed_percent(u8, offset 1, 0..100)`。S3 仅在雷达控制状态 RUNNING 时周期性发送给 STM，并同时向 App 发送对应状态。

#### `0xF000` LOG（8..104 字节）

offset 0=`source(u8)`、1=`level(u8, 0 debug/1 info/2 warn/3 error)`、2..5=`timestamp_ms(u32 LE)`、6..7=`text_length(u16 LE)`、8..=`UTF-8 text`。STM 限制文本最多 96 字节，故最大载荷 104 字节；S3 `log_bridge` 要求 `frame.length == 8 + text_length`。

## 6. 应答、防重与超时

### 6.1 哪些帧需要 ACK

帧编码器的默认规则由 `scbp_message_flags()` 给出：`0x0100/0101/0102/0300/0302/0400/0401` 设置 `ACK_REQUIRED`。当前实际业务事务只有：

1. S3 -> STM `0x0302 RADAR_PWM_READY`，STM 检查期望速度后回 `0x0005`。
2. STM -> S3 `0x0401 CAL_EVENT`，S3 在进入本地校准管理器前先回 `0x0005`，确保 STM 停止等待。
3. S3 -> STM `0x0401 id=3` 完成通知，STM 可回 `0x0005`；VERIFY 本地结果通过后即使最终 ACK 丢失也可进入 READY。

`0x0200..0208`、`0x0301`、`0xF000` 是流数据，不要求逐帧 ACK。

### 6.2 ACK 关联规则

`scbp_pending_tx_match_ack()` 必须同时满足：

- 收到帧 `MSG_ID==0x0005`、`FLAGS==ACK_FRAME`、长度正好 5；
- ACK `SRC` 等于原发送目标，ACK `DST` 等于本地节点；
- ACK payload `ack_msg_id` 等于待确认消息 ID；
- ACK payload `ack_seq` 等于原请求帧 `SEQ`；
- `result` 只能是 0/1；result=0 时 error 必须为 `SCBP_ERROR_OK`。

匹配成功后清除单项 pending transaction。当前实现每个端点只有一个全局 pending transaction，因此不支持同一方向并发等待多个不同 ACK。

### 6.3 重发与状态超时

| 机制 | 间隔/窗口 | 最大重试 | 超时结果 | 证据 |
| --- | ---: | ---: | --- | --- |
| STM CAL_EVENT ACK | 500 ms | 3 | 普通事件失败进入 `IMU_ERROR_CAL_EVENT_TIMEOUT`；VERIFY 完成事件的本地结果优先 | `imu_boot_manager.c` |
| S3 RADAR_PWM_READY ACK | 500 ms | 3 | 回 `RADAR_WAIT_SYNC`，PWM 恢复 0 | `radar_calibration_manager.c` |
| S3 静态事件等待 | 75 s | 不适用 | 回 `WAIT_SYNC` | S3 manager |
| S3 每级振动事件 | 22 s | 不适用 | 回 `WAIT_SYNC` | S3 manager |
| S3 id=2 防重复保护 | 前 11 s 不接受旧事件 | 不适用 | 忽略旧 id=2，不推进档位 | S3 manager |
| STM UART 新鲜度日志 | 2000 ms 无 RX | 不适用 | 记录 `UART_LINK_STALE` | `s3_service.c` |
| STM radar status 新鲜度 | 3000 ms | 不适用 | 标记 stale；保留当前 PWM/filter | `s3_service.c` |

`RADAR_CAL_COMPLETE_TIMEOUT_MS=5000` 在 S3 头部被定义，但当前代码未使用，不能当成生效的重试参数。

### 6.4 防重/幂等

- Parser 对每个 `SRC` 保存上一 SEQ，诊断 `GAP/DUPLICATE/OUT_OF_ORDER`，但不会自动丢弃有效帧。
- S3 `command_bridge` 对 DualAHRS 统计长度/schema/CRC/SEQ gap/duplicate，并保持 payload 转发。
- S3 校准管理器：id=1 只有在 `WAIT_EVENT + WAIT_STATIC_DONE + PWM=0` 才推进；重复 id=1 只重 ACK 不重复切换；id=2 还要经过 11 s 时间保护；id=3 在完成后只记录 duplicate。
- STM 对重复 `RADAR_PWM_READY` 在当前期望 PWM 下保持接受；对 CAL_EVENT ACK 关联 event id，错误/过期 ACK 不推进状态。

## 7. 分包、黏包与解析状态机

两端 parser 都是非阻塞字节流状态机：

1. 空闲时丢弃所有非 `0xAA` 字节；看到 `AA` 后等待 `55`。
2. 收满固定头到 offset 11 后读取 LE16 `LEN`；`LEN>128` 立即报告 `LEN_FAIL` 并寻找下一个 `AA`。
3. 计算 `expected_length = 14 + LEN`，允许本次 UART read 中继续收集剩余字节，天然支持半帧、整帧和多帧黏包。
4. 收到 `expected_length` 后验证版本、priority、reserved flags、长度和 CRC；成功才调用业务 callback。
5. 失败时保留缓冲区内最后一个可作为新起点的 `AA`，重新寻找 `AA 55`，避免错误帧吞掉后续合法帧。
6. 成功帧增加 `frame_index`，更新源节点 SEQ 状态，然后清空 parser 状态继续处理同一输入块的下一帧。

容量边界：parser 单帧 142 字节；STM 原始 RX ring 512 字节；S3 UART storage ring 4096 字节。超过环形缓冲会丢字节，协议层不会自动从丢失数据中恢复完整帧，只能依赖下一次 `AA 55` 重同步。

## 8. 与 App BLE envelope 的边界

S3 的 App 侧使用另一种封装：

```text
AA | APP_VERSION(0x01) | TYPE | LEN_L | LEN_H | PAYLOAD | CRC16_L | CRC16_H | 55
```

它是 8 字节固定开销、最大 128 字节载荷、尾部 `0x55` 的 envelope；CRC 覆盖 `VERSION` 到 payload。S3 `command_bridge.c` 只在该边界重建 envelope，SCBP 的 `MSG_ID`、payload 长度、单位、float 和四元数顺序不被改写。SCBP `0x0201` 会变成 App type `0x11`，但这不是把 App type 当成 UART MSG_ID。

## 9. 兼容旧单字节 TYPE（仅适配层）

`SC_TYPE_*` 仍存在于 `sc_frame.h`，但不再出现在 SCBP-V3 帧头；`sc_frame_encode()` 将其转换为 16 位 MSG_ID。主要映射：

| 旧 TYPE | SCBP MSG_ID | 说明 |
| ---: | ---: | --- |
| `0x01 PING` | `0x0001` |  |
| `0x02 PONG` | `0x0002` |  |
| `0x03 ACK` | `0x0005` | 旧 ACK 回调转换为 5 字节 V3 ACK |
| `0x16 RADAR_PWM_READY` | `0x0302` | S3 -> STM |
| `0x17 RADAR_PWM_ACK` | `0x0005` | ACK `0x0302` |
| `0x18 CAL_EVENT` | `0x0401` | 1 字节 event id |
| `0x19 CAL_EVENT_ACK` | `0x0005` | ACK `0x0401` |
| `0x1C STM_BOOT_READY` | `0x0007` | STM -> S3 |
| `0x20 IMU_STATUS` | `0x0200` | 38/历史 43 |
| `0x21 ATTITUDE` | `0x0201` | 30/80 active，26 legacy input |
| `0x22 IMU_CAL_STATUS` | `0x0202` | 当前 11 |
| `0x23 RADAR_STATUS` | `0x0301` | 2 |
| `0x24 RADAR_VIBRATION_STATUS` | `0x0204` | 17 |
| `0x25 IMU_CAL_RESULT` | `0x0205` | 14/26 |
| `0x26 IMU_VIBRATION_PROFILE` | `0x0206` | 26/42 |
| `0x27 IMU_TELEMETRY` | `0x0207` | 30 |
| `0x28 DUAL_IMU_STATUS` | `0x0208` | 16 |
| `0x30 LOG` | `0xF000` | 8..104 |

## 10. 已确认的差异与未闭环项

1. `0x0200` legacy IMU_STATUS 当前 producer 在 byte 0 写 `0x02` 表示 LSM303，而新 `0x0207` 使用 `IMU_SENSOR_LSM303=0x01`、`BMI323=0x02`。这是两个历史载荷的现状差异，接收端不能跨消息套用 sensor_id 语义。
2. S3/Swift 接受 `0x0200` 的 43 字节历史扩展，但当前 STM 源码只生成 38 字节；43 字节扩展的来源是兼容解码分支，不是当前 STM 生产路径。
3. `SCBP_MSG_ID_VERSION/RESET/MOTOR_CONTROL/PWM_SET/PARAM_SET/RADAR_CONTROL/CAL_START` 目前没有实际业务 handler 或完整 payload 定义；不能据此编造长度、方向或控制语义。
4. 帧编解码器没有通用的 500 ms/最大 N 次重发器；500 ms/3 次仅存在于校准状态机。其它 ACK_REQUIRED ID 目前没有完整事务实现。
5. 当前工作区只提供源代码、IOC/生成 MSP 和静态文档证据。UART 电气连接、波形、CRC 抓包、帧吞吐、S3 BLE 通知和设备运行行为均仍需单独验证。

## 11. 建议的验证清单（不代表已执行）

- [ ] 逻辑分析仪确认 STM PA2/PA3 与 S3 GPIO17/18 的 TX/RX 交叉、共地和 115200 8N1。
- [ ] 用固定 PING 向量验证 `AA 55 01 02 01 02 01 00 2A 00 00 00 D3 67` 的 CRC。
- [ ] 分别注入半帧、两帧黏包、错误 LEN、错误 CRC 和帧内 `AA`，确认两端均能重同步。
- [ ] 抓取 `0x0202` 11 字节、`0x0208` 16 字节、`0x0201` 30/80 字节并按本文件偏移解码。
- [ ] 验证 `0x0302` ACK 的 `ack_msg_id + ack_seq` 关联，以及重复/乱序 ACK 不推进校准状态。
- [ ] 验证 STM/S3 校准管理器的 500 ms 超时、3 次重试、断线回到同步状态和 id=2 重复保护。
