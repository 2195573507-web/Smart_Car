# BMI323 Driver Refactor Report

日期：2026-08-09  
范围：仅 STM32H757 BMI323 驱动内部；未烧录，未修改 LSM303、IMU Manager、
运行时、滤波、校准、姿态、SRPv4、GPIO 定义或其他工程配置。

## 1. 原问题

原活动实现位于 `STM32H757/Middleware/Sensor/BMI323/bmi323.c`，将以下职责
混在同一文件中：

- BMI323 寄存器地址、SPI 命令和数据换算；
- BMI323 CS GPIO 控制、延时与 BSP SPI 调用；
- WHO_AM_I、复位、配置和失败诊断。

这使 SPI 事务难以独立审查，也没有独立的寄存器访问接口。旧的
`STM32H757/Drivers/IMU/BMI323/` 不属于当前 CM7 BMI323 源文件，未作为本次
实现入口使用。

## 2. 新架构

```text
IMU Manager (existing API, unchanged)
        |
        v
Middleware/Sensor/BMI323/bmi323.c
  - BMI323 register semantics
  - read/write frame construction
  - WHO_AM_I and configuration
  - unit conversion and diagnostics
        |
        | bmi323_port_spi_read/write
        v
Middleware/Sensor/BMI323/bmi323_port.c
  - CS GPIO via BSP_GPIO_BMI323_CS
  - blocking BSP SPI transaction
  - CS setup/hold delay
  - raw HAL status forwarding
        |
        v
STM32H757/BSP/SPI/bsp_spi.c -> HAL_SPI_TransmitReceive/Transmit
```

文件职责：

| 文件 | 职责 |
| --- | --- |
| `bmi323.c` | 器件逻辑、寄存器帧、WHO_AM_I、配置、数据换算、统计、日志 |
| `bmi323.h` | 现有 Manager API、寄存器访问 API、诊断数据类型 |
| `bmi323_reg.h` | BMI323 地址、读掩码、复位值、状态位常量 |
| `bmi323_port.c` | CS、SPI、延时和 BSP/HAL 状态适配 |
| `bmi323_port.h` | port 层硬件适配接口 |

port 层不判断 WHO_AM_I，不解释寄存器，不执行加速度或陀螺仪换算。

## 3. SPI 调用路径

### 读寄存器

`bmi323_read_reg(reg, data, len)` 创建固定大小栈帧：

```text
TX[0] = reg | 0x80
TX[1..] = 0x00
RX[0] = command phase
RX[1] = dummy phase
RX[2..] = register payload
```

然后调用：

```text
bmi323_read_reg
  -> bmi323_port_spi_read
  -> bsp_spi_write_read
  -> HAL_SPI_TransmitReceive
```

### 写寄存器

`bmi323_write_reg(reg, data, len)` 清除读位并创建：

```text
TX[0] = reg & 0x7F
TX[1..] = payload (LSB first for 16-bit BMI323 registers)
```

调用路径为：

```text
bmi323_write_reg
  -> bmi323_port_spi_write
  -> bsp_spi_transmit
  -> HAL_SPI_Transmit
```

## 4. CS 时序

当前 port 使用软件 CS `BSP_GPIO_BMI323_CS`，其静态映射保持为 PC4：

1. `bmi323_port_cs_low()` 拉低 CS；
2. 等待 2 us；
3. 执行一个完整的 blocking SPI 事务；
4. 事务结束后等待 2 us；
5. `bmi323_port_cs_high()` 释放 CS；
6. 再等待 2 us 作为保持间隔。

CS 失败会与 SPI 状态合并返回。port 不在 CS 低电平期间输出日志，避免日志
发送改变事务时序。

## 5. WHO_AM_I 流程

初始化顺序为：

```text
bmi323_port_init()
    |
    v
bmi323_read_reg(0x00, &who_am_i, 1)
    |
    +-- SPI error/timeout -> WHO_AM_I_FAIL, offline
    |
    +-- value != 0x43     -> WHO_AM_I_FAIL, offline
    |
    v
write CMD 0x7E = 0xAF 0xDE
    |
    v
post-reset CHIP_ID read
    |
    v
write/read ACC_CONF
    |
    v
write/read GYR_CONF
    |
    v
ONLINE
```

启动日志格式：

```text
[BMI323][SPI]
TX: xx xx xx xx
RX: xx xx xx xx
HAL: <raw HAL status>

[BMI323][INIT]
WHO_AM_I=0x43
STATE=ONLINE
```

初始化失败时 `STATE=WHO_AM_I_FAIL` 或对应初始化失败状态，并输出：

```text
[BMI323][DEBUG]
online=0
whoami=0x00
read_ok=0
read_fail=<count>
write_fail=<count>
spi_error=<count>
whoami_fail=<count>
last_error=WHO_AM_I_FAIL
```

## 6. 错误统计

`bmi323_diagnostics_t` 保留原有 `spi_error_count`、SPI 读计数和数据
`read_ok/read_fail`，并增加：

- `write_count`、`write_ok`、`write_fail`；
- `spi_error`（与原 `spi_error_count` 同步，便于诊断字段直接对应）；
- `whoami_fail`；
- `last_error`。

SPI 读/写失败由驱动统一累计 `spi_error`。WHO_AM_I 读取失败或数值不符单独
累计 `whoami_fail`。重复调用 `bmi323_init()` 不清零累计失败计数，只重置本次
初始化的在线状态、身份值和配置快照。

BMI323 未在线时，`bmi323_read_accel/read_gyro` 仅更新 BMI323 自身的失败统计，
不改变 `imu_init()` 的 LSM303 返回值，也不调用或修改 LSM303 状态。

## 7. 资源与实时性检查

- 事务缓冲区为固定大小栈数组，最大读帧 28 字节、最大写帧 3 字节，不使用
  堆分配；
- `bmi323_read_reg()` 的构建记录栈用量为 80 字节，port 读写函数为 32 字节；
- 首次 SPI 事务和初始化诊断会格式化日志，数据周期路径不重复输出 SPI 日志，
  避免 UART 日志放大实时性影响；
- SPI 调用仍是 blocking BSP 事务，单次超时上限为 20 ms。若在线器件运行中
  连续失联，首次失败仍会占用该超时，实际任务周期和栈余量需要在板上继续
  观察；
- 本轮没有引入中断、DMA、锁、重试或新的外设竞争，也没有修改 FreeRTOS
  任务栈配置。

## 8. 修改文件

- `STM32H757/Middleware/Sensor/BMI323/bmi323.c`
- `STM32H757/Middleware/Sensor/BMI323/bmi323.h`
- `STM32H757/Middleware/Sensor/BMI323/bmi323_port.c`
- `STM32H757/Middleware/Sensor/BMI323/bmi323_port.h`
- `STM32H757/Middleware/Sensor/BMI323/bmi323_reg.h`
- `STM32H757/CM7/CMakeLists.txt`（仅增加 `bmi323_port.c`）
- `BMI323_DRIVER_REFACTOR_REPORT.md`

## 9. 后续 bring-up 步骤

1. 不烧录其他变更，使用本驱动构建产物烧录目标板；
2. 使用独立 USART1/日志链路捕获 `[BMI323][SPI]`，确认 `TX=80 ...`、RX
   有效且 `HAL=0`；
3. 确认 `[BMI323][INIT] WHO_AM_I=0x43` 与 `STATE=ONLINE`；
4. 观察 `[BMI323][DEBUG] online=1` 且 `read_ok` 持续增长、`spi_error` 不
   持续增长；
5. 再检查加速度和陀螺仪数据的变化、CS 波形和 2 us 时序；
6. 在独立任务中评估长期 SPI 错误、栈余量和 BMI323 失败对 LSM303 周期的影响；
7. 本轮不进入姿态融合，也不修改校准或通信协议。

## 10. 验证边界

源码审查和 CM7 编译只能证明文件已纳入构建、接口可链接以及分层依赖方向
正确。当前未烧录、未抓取 SPI 波形、未读取实际 BMI323，因此不能在本报告中
宣称已经达到 `WHO_AM_I=0x43`、`online=1` 或 `read_ok>0` 的硬件目标。
