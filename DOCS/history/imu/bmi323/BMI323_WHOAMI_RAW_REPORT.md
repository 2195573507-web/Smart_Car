# BMI323 WHO_AM_I 原始数据报告

## 修改文件

- `STM32H757/Middleware/Sensor/BMI323/bmi323.c`
- `BMI323_WHOAMI_RAW_REPORT.md`

本轮没有修改 IMU Manager、LSM303、校准、姿态、通信协议、寄存器配置、CubeMX 或烧录流程。

## 当前 SPI 读帧

`bmi323_read_reg()` 为 BMI323 SPI 读操作分配完整的两字节事务（寄存器地址字节加一个 dummy 字节）：

| 字节 | MOSI/TX | MISO/RX | 说明 |
| --- | --- | --- | --- |
| 0 | `reg \| 0x80` | dummy | BMI323 SPI 读标志和寄存器地址 |
| 1 | `0x00` | register value | 读取的寄存器数据 |

因此 WHO_AM_I（`reg=0x00`）的源码期望帧为：

```text
TX: 80 00
RX: 00 43
```

原始日志保留三个字节显示槽，以便和既有诊断格式一致；WHO_AM_I 实际事务长度是 2，第三个槽为缓冲区填充字节，不属于 SPI 事务。

## WHO_AM_I 原始日志

初始化首次读取 `reg=0x00` 后输出一次：

```text
[BMI323][WHOAMI_RAW]
reg=0x00
value=0xXX
expected=0x43
actual=0xXX
[BMI323][SPI_RAW]
tx_len=2
rx_len=2
TX:
xx xx xx
RX:
xx xx xx
```

`static bool trace_done` 在第一次日志前后控制采集和输出，后续 WHO_AM_I 访问不会重复打印。

## 实际数据与判断

本轮按要求未烧录、未连接运行设备，也没有串口或逻辑分析仪采集。因此当前仓库没有可确认的实际 TX/RX 电平或 `WHO_AM_I` 返回值，不能把 `0x43` 或其他值宣称为硬件实测结果。

源码层面可以确认：

- TX 地址阶段为 `0x80`，随后发送 `0x00` dummy；
- RX 的 `RX[1]` 被作为 WHO_AM_I 值；
- `0x43` 是期望值；
- 失败时日志仍会保留第一次事务的原始缓冲区；如果 HAL 没有写入 RX，缓冲区初值为 `0x00`，这不等同于传感器实际返回值。

## 下一步

在允许硬件验证的下一轮：

1. 烧录包含本次日志的固件，并抓取首次 `[BMI323][WHOAMI_RAW]` 输出。
2. 用逻辑分析仪同时确认 `CS(PC4)`、`SCK(PA5)`、`MISO(PA6)`、`MOSI(PA7)`；确认 CS 在两个字节期间保持低电平。
3. 对照 `TX=80 00`、`RX=00 XX` 和 HAL 返回状态，区分传感器未选中、MISO 无响应、SPI 配置/连线错误或芯片 ID 值错误。

在获得上述运行证据前，只能确认软件帧格式，不能判断最终是软件 SPI 配置问题还是硬件链路问题。
