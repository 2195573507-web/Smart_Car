# BMI323 SPI WHO_AM_I 原始链路诊断

日期：2026-08-09  
范围：仅 `STM32H757/Middleware/Sensor/BMI323/`；未烧录，未修改
IMU Manager、LSM303、校准、姿态、协议、S3、App、CubeMX 或 GPIO 定义。

## 1. 修改文件

- `STM32H757/Middleware/Sensor/BMI323/bmi323.c`
  - 仅第一次 `reg=0x00` 访问输出 `[BMI323][SPI_TRACE]`；
  - 输出 TX/RX 长度、原始字节、HAL 原始状态、CS 前后状态；
  - 增加 `WHO_AM_I_TIMEOUT`、`SPI_TX_FAIL`、`SPI_RX_FAIL`、
    `WHO_AM_I_VALUE_ERROR` 分类和对应计数。
- `STM32H757/Middleware/Sensor/BMI323/bmi323_port.c/.h`
  - 通过 trace 参数采集 CS 前后状态；
  - 第一次 trace 事务输出 `BMI323_CS_LOW/HIGH`；
  - 初始化输出活动 SPI1 的源配置：MODE0、8-bit、约 1.875 MHz；
  - `SPI_HandleTypeDef` 在 `bsp_spi.c` 中是私有静态对象，本次允许范围不能
    读取其地址，因此日志标记为 `spi_handle=BSP_PRIVATE`，没有伪造地址。
- `STM32H757/Middleware/Sensor/BMI323/bmi323.h`
  - 扩展诊断枚举和 `spi_tx_fail/spi_rx_fail` 字段，保留现有接口。

## 2. WHO_AM_I SPI 时序

本次原始 WHO_AM_I 访问使用一个命令字节和一个 dummy 字节：

```text
TX[0] = 0x00 | 0x80 = 0x80
TX[1] = 0x00
RX[0] = dummy
RX[1] = WHO_AM_I data
```

`bmi323_read_reg()` 只在 `reg=0x00` 时传递 trace，port 层不解释寄存器。
读数据从 `RX[1]` 开始复制。其他寄存器继续通过同一层级化读接口访问。

## 3. 原始 TX/RX 格式

第一次访问的日志格式为：

```text
[BMI323][SPI_CONFIG]
instance=SPI1
clock_mode=MODE0
datasize=8BIT
baudrate=1875000Hz
spi_handle=BSP_PRIVATE

[BMI323][CS] BMI323_CS_LOW
[BMI323][CS] BMI323_CS_HIGH

[BMI323][SPI_TRACE]
reg=0x00
tx_len=2
rx_len=2
tx:
80 00 00
rx:
00 xx 00
hal_status=<raw HAL status>
cs_before=HIGH
cs_after=HIGH
```

第三个 TX/RX 字节是固定 trace 展示位，二字节事务中为 0，不参与寄存器
判断。期望 `RX[1]=0x43`。

当前 `BSP/SPI/bsp_spi.c` 还保留历史的首笔事务 trace；本轮禁止修改 BSP，
因此上板时可能先看到旧格式的 `[BMI323][SPI_TRACE] len/tx/rx/hal`，再看到本
驱动的 `reg/tx_len/rx_len/cs_before/cs_after` 格式。两者均为一次性诊断，不能
据此判断有多个 WHO_AM_I 周期访问。

CS 由 `bmi323_port_cs_low/high()` 调用 `bsp_gpio_write(BSP_GPIO_BMI323_CS,
BSP_GPIO_LOW/HIGH)`，当前 BSP 静态映射为 `GPIOC/GPIO_PIN_4`，对应底层
`HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, RESET/SET)`。

## 4. 当前判断

### 已确认的软件事实

1. 之前活动实现发送 `len+2` 字节并从 `RX[2]` 取值；这与本任务指定的
   `command + 1 dummy` 原始格式不一致，属于确定的软件帧偏移问题。
2. 当前 BSP 源码配置为 SPI1、主机、Mode 0、8-bit、MSB-first、软件 NSS、
   `/128`，记录的 SPI123 kernel clock 为 240 MHz，约 1.875 MHz。
3. HAL SPI handle 由 `bsp_spi.c` 私有持有；仅修改 BMI323 目录无法读取其
   `SPI_HandleTypeDef` 地址。
4. 日志中的 `read_ok=0/read_fail>4000` 是 WHO_AM_I 初始化失败后的下游结果：
   `bmi323_ready` 保持 false，后续 accel/gyro 接口快速返回并累计 `read_fail`，
   并不表示已经发生了 4000 次 WHO_AM_I SPI 访问。

### 尚不能从源码确定的部分

本轮没有烧录、串口捕获或逻辑分析仪波形，因此不能证明目标板已经收到
`0x80 0x00`，也不能证明 MISO 返回了实际器件数据。

### 诊断结论边界

- 若 trace 显示 `hal_status=HAL_TIMEOUT`，当前分类为 `WHO_AM_I_TIMEOUT`，
  应优先检查 SPI 初始化/资源占用及总线电气状态；
- 若 `hal_status` 成功、CS 前后为 HIGH，但 `RX[1]=0x00/0xFF`，软件事务已
  完成，优先定位 BMI323 供电、GND、CS、MISO 接线或器件未响应；
- 若 `hal_status` 成功且 `RX[1]` 为其他值，分类为 `WHO_AM_I_VALUE_ERROR`，
  需要检查器件型号、SPI mode、bit order 和采样波形；
- 当前 build 不能在“软件 SPI 配置问题”和“硬件链路问题”之间替代实测作
  最终判断。已有确定的软件帧偏移问题已被原始诊断路径纠正，剩余归因必须
  依赖下一步 capture。

## 5. 下一步硬件检查建议

1. 烧录前确认本轮不改变 CubeMX、GPIO、BSP 或上层模块；本次工作尚未烧录。
2. 连接独立日志链路，确认带 `reg=0x00` 的新增 trace 只出现一次；旧 BSP
   格式的首笔 trace 可能另外出现一次。
3. 用逻辑分析仪同时观察 PC4(CS)、PA5(SCK)、PA7(MOSI)、PA6(MISO)：
   - CS 是否为 HIGH -> LOW -> HIGH；
   - MOSI 是否为 `80 00`；
   - SPI mode 是否为 CPOL=0/CPHA=0；
   - MISO 第二字节是否为 `43`。
4. 确认 BMI323 VDD/VDDIO 电压、共地、CS 电平和 SDO/MISO 连通性。
5. 只有出现 `RX[1]=0x43`、`hal_status=HAL_OK` 和 `cs_after=HIGH` 后，才可
   继续评估在线状态和后续数据读取；本任务不进入 accel/gyro 或姿态验证。

## 验证边界

`cmake --preset Debug` 与 CM7 编译只证明源码集成和链接正确，不证明实际
WHO_AM_I、SPI 波形或硬件链路。硬件判断保持 **UNVERIFIED**，直到取得目标板
原始 trace。
