# BMI323 开源实现对比与当前工程读数失败分析

## 结论先行

当前工程存在一个不依赖硬件的确定性故障：正式启动路径在
`STM32H757/Middleware/Sensor/imu_manager.c:93` 调用
`bmi323_init_diag()`。该诊断分支在
`STM32H757/Drivers/IMU/BMI323/bmi323.c:444-450` 只读取并校验
`CHIP_ID`，随后直接返回；它既没有执行软复位和传感器配置，也没有把
`bmi323_ready` 置为 1。之后 `bmi323_read_acc()`、
`bmi323_read_gyro()` 和 `bmi323_read_temperature()` 会在各自的
`bmi323_ready == 0` 检查处返回 `BSP_STATUS_NOT_READY`。

因此，即使 SPI 物理链路已经正确并且 CHIP_ID 读到了 `0x0043`，当前
启动流程仍不能进入可读状态。这是目前最直接、证据最充分的原因。

如果日志中的 CHIP_ID 本身是 `0x0000`、`0xFFFF` 或 HAL 超时，则还存在
独立的 SPI/电源/连线问题；本仓库没有示波器、逻辑分析仪或最新串口运行日志，
不能仅靠静态源码断言具体是哪一根线或哪一个器件损坏。

## 公开开源实现检索

以下仓库在 2026-08-03 可访问，并按“是否真正覆盖 BMI323 SPI 读事务”分类。
提交号用于固定本次对照版本。

| 仓库 | 版本 | 接口证据 | 结论 |
| --- | --- | --- | --- |
| [Bosch BMI3XY_SensorAPI](https://github.com/boschsensortec/BMI3XY_SensorAPI/tree/b3033e78bc6e2c2e473f24e6d79afef0e16c4655) | `b3033e7` | 官方 `bmi3.c`、`bmi3.h`、`bmi323_defs.h` 和 SPI 示例 | 主参考实现 |
| [Ahreo/BMI323-Driver](https://github.com/Ahreo/BMI323-Driver/tree/d19d3770c9210ba48b96f0d29319113a83c793e8) | `d19d37` | Mbed SPI 读函数和 SPI mode 0/3 说明 | 可用于交叉验证时序 |
| [7semi-solutions/7Semi-BMI323](https://github.com/7semi-solutions/7Semi-BMI323/tree/a6f73849070c151838d4b3d05fd722670e9966a5) | `a6f7384` | `examples/SPI_Basic`，8-bit、mode 0、CS 独立控制 | SPI 应用示例 |
| [DFRobot/DFRobot_BMI323](https://github.com/DFRobot/DFRobot_BMI323/tree/46f8913034a594aa5fe2988ce2d049eb8bb9bde5) | `46f8913` | Arduino/I2C 封装，内含 Bosch API | 不能直接当作 SPI 实现 |
| [nopnop2002/esp-idf-bmi323](https://github.com/nopnop2002/esp-idf-bmi323/tree/a35afbb802e98131cb5753d6b7d5b0c236216ca1) | `a35afbb` | ESP-IDF I2C master 回调 | 不能直接当作 SPI 实现 |
| [Ueberspannung/BMI323](https://github.com/Ueberspannung/BMI323/tree/b2b25d9f22e669df36402652528fbac73393fed3) | `b2b25d9` | Arduino 风格寄存器封装，存在未完成路径 | 仅作补充参考 |

### Bosch 官方 API 的关键证据

对应文件：
[bmi3.c](https://github.com/boschsensortec/BMI3XY_SensorAPI/blob/b3033e78bc6e2c2e473f24e6d79afef0e16c4655/bmi3.c)
和
[bmi323_defs.h](https://github.com/boschsensortec/BMI3XY_SensorAPI/blob/b3033e78bc6e2c2e473f24e6d79afef0e16c4655/bmi323_defs.h)。

1. `bmi3_init()` 在 SPI 接口下将 `dummy_byte` 设为 1；I2C 则为 2。
2. `bmi3_get_regs()` 把读地址 OR 上 `0x80`，调用底层读函数时请求
   `payload_length + 1` 个字节，并从 dummy 字节之后复制 payload。
3. `bmi3_set_regs()` 把写地址与 `0x7F` 相与，配置寄存器按低字节在前发送。
4. `bmi3_soft_reset()` 向 `0x7E` 写入 `0xDEAF`，等待复位，然后在 SPI
   下再次做一次 dummy read，以完成复位后的 SPI 状态恢复。
5. `BMI323_CHIP_ID` 为 `0x0043`，`CHIP_ID` 寄存器地址为 `0x00`。
6. 官方 COINES 示例把 SPI 配置为 mode 0、10 MHz；Ahreo 的 Mbed 实现
   说明 BMI323 兼容 mode 0 和 mode 3，并在 CS 上电后产生一次边沿。

对一次 2 字节 CHIP_ID 读取，线上的典型完整帧应是：

```text
TX: 80 00 00 00
RX: xx xx 43 00
             ^  ^
             |  +-- CHIP_ID 高字节
             +----- CHIP_ID 低字节
```

其中前两个 RX 字节分别是命令阶段返回值和一个 dummy byte；具体前缀值
不应被当作 CHIP_ID。

## 当前工程逐项对比

| 项目 | 当前工程 | 公开实现要求 | 判断 |
| --- | --- | --- | --- |
| MCU 引脚 | SPI1: PA5=SCK、PA6=MISO/SDO、PA7=MOSI/SDI；PC4=CS；PB2=INT1 | SPI 三线/四线信号加独立 CS | 命名与常见接法一致，实际连线未验证 |
| SPI 格式 | 8-bit、MSB first、mode 0 | Bosch 官方示例 mode 0；部分实现用 mode 3 | 配置本身合理 |
| SPI 频率 | `240 MHz / 128 ~= 1.875 MHz` | 芯片支持的常用低速至 10 MHz 范围 | 频率不是首要嫌疑 |
| 读地址 | `reg | 0x80` | 官方同样使用 `BMI3_SPI_RD_MASK = 0x80` | 一致 |
| dummy 处理 | `HAL_SPI_TransmitReceive()` 总长度为 `len + 2`，丢弃 `rx[0:1]` | 命令后 1 个 dummy，再取 payload | 对完整 SPI 帧的处理一致 |
| CHIP_ID | `0x00`，期望 `0x0043` | `0x00`，`0x0043` | 一致 |
| 上电/初始事务 | 先发送 `0x7F, 0x00` 的写事务，再读 CHIP_ID | 官方 API 直接进入软复位/寄存器读；公开 Mbed 代码首次事务为读地址 | 高风险协议偏差，需逻辑分析仪确认 |
| 正式初始化调用 | `imu_manager` 调 `bmi323_init_diag()` | 可读路径必须完成 reset、配置并标记 ready | **确定性错误** |
| `ready` 状态 | 只有非诊断分支末尾才写 `bmi323_ready = 1` | 成功初始化后应允许数据读函数执行 | **诊断分支必然 NOT_READY** |
| 生成工程关系 | BSP 自建 `hspi1_bsp`，不复用 CubeMX `hspi1` | 允许自有 HAL handle，但必须验证 MSP/时钟/引脚 | 编译通过不等于总线已工作 |

## 为什么当前代码读不到

### P0：启动路径把“探针诊断”当成“完整初始化”

调用链如下：

```text
main.c:134
  -> imu_runtime_start()
     -> imu_manager.c:137 imu_init()
        -> imu_manager.c:93 bmi323_init_diag()
           -> bmi323.c:444 read CHIP_ID
           -> bmi323.c:450 return
        -> imu_manager.c:127 imu_update()
           -> bmi323_read_acc/gyro/temp()
              -> bmi323_ready == 0
              -> BSP_STATUS_NOT_READY
```

`bmi323_init_diag()` 这个函数的行为实际上是“初始化 SPI、打印 GPIO、
尝试读取 CHIP_ID”，不是“初始化 BMI323 并开放数据读取”。由于
`imu_init()` 固定传入日志/诊断路径，正常启动每次都会触发该错误。

### P1：SPI 模式切换事务与公开实现不一致

当前 `bmi323_enter_spi_mode()` 向地址 `0x7F` 发送一个写事务
(`0x7F, 0x00`)。Bosch API 没有这个写保留地址步骤；其标准路径是把读地址
OR `0x80`，再按 SPI dummy 规则取数据。公开 Mbed 实现也把上电后的首次
事务写成 `0x80 | register` 的读操作。

这不一定在所有板卡上都失败，但它是当前源码相对于公开实现的真实协议偏差。
当 CHIP_ID 读不到时，应优先用逻辑分析仪验证第一笔 CS/SCK/MOSI/MISO 事务，
并用标准的 `80 00 00 00` CHIP_ID 读帧做对照。

### P1：诊断日志不能证明数据链路

工程能链接出 CM7 ELF，且现有构建目录执行
`cmake --build STM32H757/CM7/build/Debug -- -j2` 返回 `ninja: no work to do`。
这只证明当前源文件已经被构建系统接受，不证明：

- BMI323 的 VDD/VDDIO 电压和地线正确；
- PC4 真的连到传感器 CSB，且空闲为高；
- PA6 没有悬空、短路或与 PA7 对调；
- 第一笔事务有实际 SCK 边沿；
- `HAL_SPI_TransmitReceive()` 的 RX 字节在物理 MISO 上变化。

### P2：硬件/时序待验证项

1. BMI323 芯片和模块的供电必须处于数据手册允许范围，VDD 与 VDDIO
   都稳定后再产生 CS/SCK 活动。
2. CSB 应保持高电平，事务期间才拉低；不要把 BMI323 的 SDO/SDI
   标签与 MCU 的 MISO/MOSI 方向接反。
3. mode 0 和 mode 3 都是公开实现使用过的模式，但必须在同一笔事务中保持
   一致，不能由 HAL 或外部器件改变 SCK 空闲电平。
4. 先用低速标准读帧验证 `0x43`，再增加频率；不要在 CHIP_ID 失败时先
   诊断加速度缩放、FreeRTOS 调度或中断映射。

## 建议的最小修复顺序

本轮只做分析，未修改源代码。实际修复应按以下顺序进行：

1. 将生产启动路径改为调用完整的 `bmi323_init()`；诊断函数只负责
   在完整初始化前后打印探针信息，或重命名为明确的 `probe` 并禁止其
   进入数据采集路径。
2. 让成功的初始化路径唯一负责设置 `bmi323_ready = 1`，并确保软复位、
   ACC/GYR 配置和复位后的 dummy read 都执行完毕。
3. 暂时移除 `0x7F,0x00` 的自定义模式切换写事务，按 Bosch API 的标准
   CHIP_ID 读帧抓包；若硬件手册明确要求额外上电边沿，再单独保留并验证。
4. 在硬件上抓取一笔完整的 `CS/SCK/MOSI/MISO` 波形，保存 TX/RX 原始字节；
   只有看到 `rx[2] = 0x43`、`rx[3] = 0x00` 后，才进入数据配置和采样验证。
5. 最后再验证 INT1、ODR、量程和 SI 单位换算。INT1 不能替代 CHIP_ID
   轮询，也不能用 watchdog 或重试掩盖 SPI 读失败。

## 证据边界

- 已验证：当前源代码控制流、寄存器地址/掩码、公开仓库中的官方 API 和
  SPI 示例、CM7 现有构建目录可复用。
- 未验证：当前板卡的电压、焊接、CS/MISO/MOSI 实际连线、逻辑分析仪波形、
  串口运行日志和 BMI323 实物型号。
- 因此，本报告把“诊断分支导致 `BSP_STATUS_NOT_READY`”列为确定性根因，
  把 SPI 首笔事务和硬件连接列为需要现场证据确认的独立风险。
