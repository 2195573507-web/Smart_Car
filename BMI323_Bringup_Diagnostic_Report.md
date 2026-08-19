# BMI323 Bring-up Diagnostic Report

日期：2026-08-08  
范围：仅检查 BMI323 SPI/WHO_AM_I 底层路径，并增加一次性启动诊断。  
明确排除：LSM303、IMU Manager、校准流程、协议、烧录和硬件操作。

## 结论

1. 当前 CM7 实际编译的 BMI323 实现是
   `STM32H757/Middleware/Sensor/BMI323/bmi323.c`，不是同名的旧路径
   `STM32H757/Drivers/IMU/BMI323/bmi323.c`。依据是
   `STM32H757/CM7/CMakeLists.txt:22-25`。
2. 本次要求检查的静态项目没有发现确定性配置错误：SPI mode 0、CPOL=0、
   CPHA=0、MSB-first、寄存器地址 `0x00`、读位 `addr | 0x80` 以及软件 CS
   时序均与当前 BMI323 SPI 读框架一致。
3. 因此，单靠当前源码不能把 `WHO_AM_I_FAIL` 继续定位到某一根线、某个
   电源或某个器件故障。`WHO_AM_I_RAW` 和 CS 状态现在会在首次启动尝试时
   被记录；必须结合目标板串口日志及 CS/SCK/MOSI/MISO 波形才能闭合根因。
4. BMI323 失败策略未改变：`bmi323_init()` 仍返回失败，现有上层仍保留
   LSM303 初始化和运行路径。

## 1. 实际活动路径

| 项目 | 当前事实 | 证据 |
| --- | --- | --- |
| 编译源文件 | `Middleware/Sensor/BMI323/bmi323.c` | `STM32H757/CM7/CMakeLists.txt:22-25` |
| 头文件解析 | `Middleware/Sensor/BMI323/bmi323.h` | 活动驱动同目录 `#include "bmi323.h"` |
| 初始化入口 | `bmi323_init()` | `STM32H757/Middleware/Sensor/imu_manager.c:510` |
| WHO_AM_I helper | `bmi323_read_who_am_i()`；没有名为 `bmi323_read_reg()` 的活动函数 | `Middleware/Sensor/BMI323/bmi323.c:215` |
| 通用读 helper | `bmi323_read_registers()` | `Middleware/Sensor/BMI323/bmi323.c:180` |

仓库中保留的 `Drivers/IMU/BMI323` 旧实现包含另一套 `bmi323_init_diag()`，
但当前 CM7 CMake 没有把它作为 BMI323 实现源文件编译。两套路径不能混用作
当前运行时结论。

## 2. SPI 初始化检查

| 检查项 | 当前配置 | 判定 |
| --- | --- | --- |
| 外设/方向 | SPI1、Master、2-line、8-bit | 符合 |
| SPI mode | `CLKPolarity = SPI_POLARITY_LOW`，`CLKPhase = SPI_PHASE_1EDGE` | mode 0，即 CPOL=0、CPHA=0 |
| Bit order | `FirstBit = SPI_FIRSTBIT_MSB` | 符合 |
| NSS/CS | `SPI_NSS_SOFT`，CS 由 `BSP_GPIO_BMI323_CS` 软件控制 | 符合独立 CS 方案 |
| 分频 | `SPI_BAUDRATEPRESCALER_128` | 静态值明确 |
| 频率 | IOC 记录 `RCC.SPI123Freq_Value=240000000`；`240 MHz / 128 = 1.875 MHz` | 这是源码计算值，未用示波器实测 |

实现位置：`STM32H757/BSP/SPI/bsp_spi.c:44-67`；频率设计记录位于
`STM32H757/Smart_Car_H757.ioc:347`。当前 BSP 的注释按 240 MHz SPI123
kernel clock 计算 1.875 MHz；本任务没有修改时钟树，也没有声称运行时已测得
该频率。

## 3. WHO_AM_I 读事务检查

### 地址和读位

- WHO_AM_I/CHIP_ID 地址：`BMI323_REG_CHIP_ID = 0x00`
  (`Middleware/Sensor/BMI323/bmi323.c:12`)。
- 通用读路径写入：`tx[0] = reg | BMI323_SPI_READ_MASK`，其中
  `BMI323_SPI_READ_MASK = 0x80` (`:20`, `:192`)。
- WHO_AM_I 专用事务为 `TX = {0x80, 0x00, 0x00}`，并把 `rx[2]` 作为
  `WHO_AM_I_RAW` (`:215-218`, `:247-257`)。前两个接收字节分别对应命令阶段
  和 SPI dummy 阶段；通用多字节读也从 `rx[index + 2]` 取数据
  (`:208-210`)。

### CS 时序

WHO_AM_I 路径的实际顺序为：

```text
CS HIGH -> 2 us -> CS LOW -> 2 us -> HAL_SPI_TransmitReceive()
       -> 2 us -> CS HIGH -> 2 us -> read back CS state
```

证据为 `bmi323_read_who_am_i()` 的 `:237-257`。通用读写路径也使用同一套
软件 CS 选择/释放函数 (`:159-177`, `:270-281`)。从源码看，CS 没有在 SPI
传输期间被提前释放；读失败时会记录 SPI 结果、CS 选择/释放结果和最终 CS 电平。

## 4. 新增一次性启动诊断

修改位置：`STM32H757/Middleware/Sensor/BMI323/bmi323.c:72-88`，调用位置
为 `:311-326`。

首次启动尝试输出一行：

```text
[BMI323][STARTUP_DIAG] BMI323_CS=PC4 CS_LEVEL=HIGH SPI_MODE=0 SPI_FREQ=1875000Hz WHO_AM_I_RAW=0x00
```

字段含义：

| 字段 | 含义 |
| --- | --- |
| `BMI323_CS=PC4` | 静态 CS 映射 |
| `CS_LEVEL` | 事务结束后读回的 CS 电平；SPI 初始化失败时为 `UNKNOWN` |
| `SPI_MODE` | 编译期对应 BSP 的 mode 0 配置 |
| `SPI_FREQ` | 按 IOC 240 MHz 与 `/128` 计算的 1,875,000 Hz |
| `WHO_AM_I_RAW` | 本次 WHO_AM_I 事务取得的 `rx[2]`，读失败时保留默认 `0x00` |

`bmi323_startup_diag_logged` 是驱动静态一次性门控，不会因后续恢复调用
重复刷屏；没有修改初始化返回值、重试策略或任何上层状态机。

## 5. 根因判断边界

### 已确认

- 软件配置检查项与 BMI323 SPI 读框架一致。
- 读地址、读位、dummy 偏移和 CS 基本时序均能在当前活动源码中定位。
- 新诊断字符串已进入 CM7 ELF。

### 仍未确认

以下事项不能由源码、CMake 或 ELF 证明：

- PC4 是否实际连接到 BMI323 CSB，且空闲保持高电平；
- PA5/PA6/PA7 是否分别连到 SCK/SDO/SDI，是否存在 MISO/MOSI 对调；
- VDD、VDDIO、GND 和上电时序是否满足器件要求；
- 运行时 SPI123 kernel clock 的实际来源和 SCK 频率；
- CS 拉低期间是否真的产生 8-bit SCK 边沿，MISO 是否返回数据；
- 目标板实际安装的器件是否为 BMI323。

### 根据诊断值的判读

| 目标日志/波形 | 解释边界 |
| --- | --- |
| `WHO_AM_I_RAW=0x43` | WHO_AM_I 软件读通；若随后失败，应转查 reset/config，而不是继续归因于 WHO_AM_I |
| `0x00` 或 `0xFF`，HAL 成功 | 优先抓物理 MISO/CS/SCK，检查供电、CS 连接、MISO 浮空/短路和器件响应 |
| 非 `0x43` 的稳定值 | 检查器件型号、位采样边沿、线序和电平完整性 |
| `SPI` HAL timeout/error | 先查 SPI 外设状态、kernel clock、总线占用和 CS 波形 |

因此，本次静态结论是：**没有发现请求检查项中的确定性软件错误；当前
`WHO_AM_I_FAIL` 的具体根因仍在硬件/运行时波形证据边界之外。**

## 6. 失败策略与受限范围确认

- 未修改 `STM32H757/Drivers/IMU/LSM303/`。
- 未修改 `STM32H757/Middleware/Sensor/imu_manager.c/.h`。
- 未修改 `STM32H757/Middleware/Calibration/`。
- 未修改任何协议头、帧编码或协议文档。
- `bmi323_init()` 失败仍返回 `false`；上层 LSM303 先初始化并保持原有运行
  逻辑 (`imu_manager.c:489-512`)。
- 未执行烧录、复位、串口采集或逻辑分析仪操作。

## 7. 验证记录

| 验证 | 结果 | 证据 |
| --- | --- | --- |
| CMake 配置 | PASS | `cmake --preset Debug` |
| CM7 clean build | PASS | `cmake --build build/Debug --target Smart_Car_H757_CM7 --clean-first -j2` |
| ELF 链接 | PASS | `Smart_Car_H757_CM7.elf`，FLASH 96,508 B/1 MB，RAM 40,024 B/128 KB |
| 诊断字符串进入产物 | PASS | `strings build/Debug/Smart_Car_H757_CM7.elf` 找到 `STARTUP_DIAG`、`SPI_MODE`、`SPI_FREQ`、`WHO_AM_I_RAW` |
| 补丁空白/格式检查 | PASS | `git diff --check` |
| 设备运行/波形 | NOT RUN | 按任务要求不烧录、不做硬件操作 |

## 8. 本次改动

| 文件 | 改动 |
| --- | --- |
| `STM32H757/Middleware/Sensor/BMI323/bmi323.c` | 增加一次性启动诊断及静态诊断常量；不改变读事务、失败返回或上层策略 |
| `BMI323_Bringup_Diagnostic_Report.md` | 本报告 |

根因闭合的下一步仅是使用新增日志配合一次目标板抓包：保存 CS、SCK、MOSI、
MISO 四线波形和完整 RX 字节，再把 `WHO_AM_I_RAW` 与 HAL/CS 状态对应起来。
