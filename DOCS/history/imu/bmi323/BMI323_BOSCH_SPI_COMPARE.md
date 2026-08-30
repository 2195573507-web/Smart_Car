# BMI323 SPI 实现对比：当前驱动、Bosch SensorAPI 与 Zephyr

日期：2026-08-09  
范围：只读审查 SPI 寄存器读事务；未修改任何 C/H/IOC/CMake 文件，未烧录、未连接设备、未抓取波形。

## 1. 参考边界

当前 CM7 构建实际纳入的是：

- `STM32H757/Middleware/Sensor/BMI323/bmi323.c`
- `STM32H757/Middleware/Sensor/BMI323/bmi323_port.c`
- `STM32H757/BSP/SPI/bsp_spi.c`

依据：`STM32H757/CM7/CMakeLists.txt:22-25`。旧的
`STM32H757/Drivers/IMU/BMI323/bmi323.c` 没有作为当前 BMI323 驱动实现使用，
仅作为历史对照。

参考源码版本：

| 参考 | 固定版本/来源 | 本文使用的 SPI 证据 |
| --- | --- | --- |
| Bosch BMI3XY SensorAPI | [`b3033e7`](https://github.com/boschsensortec/BMI3XY_SensorAPI/tree/b3033e78bc6e2c2e473f24e6d79afef0e16c4655) | `bmi3.c:1827-1835,1878-1910,1928-1955`; `bmi323_examples/common/common.c:242-250,319-337` |
| Zephyr BMI323 | [`main`](https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/sensor/bosch/bmi323/bmi323_spi.c)（文件 blob `be2d214a643ce811767cdcaab2624755fc376467`） | `bmi323_spi.c:11-43`、`:46-69` |

Bosch SensorAPI 的 `read`/`write` 是由应用提供的接口回调；SensorAPI 核心不
直接控制 MCU GPIO CS，也不直接调用 STM32 HAL。Zephyr 的 `spi_dt_spec` 则把
CS、mode 和频率交给设备树/SPI 控制器配置。

## 2. 当前 `bmi323_read_reg` 逐行展开

来源：`STM32H757/Middleware/Sensor/BMI323/bmi323.c:214-255`。

```text
214  bmi323_read_reg(reg, data, len)
216  tx[0 .. MAX+0] = 0; rx[0 .. MAX+0] = 0
218  frame_length = len + 1
222  检查 data != NULL、len != 0、len <= 26
227  tx[0] = reg | 0x80       // SPI read bit
228  tx[1] = 0                // 发送一个 dummy/时钟字节
230  bmi323_port_spi_read(tx, rx, frame_length, ...)
234  捕获 WHO_AM_I 原始 RX
235  HAL/BSP 失败则返回 false
251  data[index] = rx[index + 1]
```

底层调用链（来源：`bmi323_port.c:97-121`、`bsp_spi.c:105-145`）：

```text
bmi323_read_reg
  -> bmi323_port_spi_read
     -> CS LOW
     -> bsp_spi_write_read
        -> one HAL_SPI_TransmitReceive(..., length = len + 1)
     -> CS HIGH
```

因此当前 `bmi323_read_reg(reg, data, 2)` 实际只请求 3 个 SPI 字节：
`[reg|0x80, 0x00, 0x00]`，并把 `RX[1]`、`RX[2]` 作为两个 payload 字节。这里
的 `RX[0]` 是命令阶段返回值；若 BMI323 还需要一个命令后的 dummy 周期，
则当前帧少一个 payload 时钟，且 `RX[1]` 实际仍是 dummy，而不是第一个数据字节。

当前初始化中的 CHIP_ID 调用是 `bmi323_read_reg(BMI323_REG_CHIP_ID, &who_am_i, 1U)`
（`:310`），所以该调用实际请求 **2 个 SPI 字节**：`80 00`，并把 `RX[1]`
当作单字节 CHIP_ID。它不是 Zephyr 的 16-bit CHIP_ID word transaction。

## 3. Bosch SensorAPI 逐行对应

来源：Bosch `bmi3.c`。

```text
1827-1835  bmi3_init(): SPI 时 dev->dummy_byte = 1，I2C 为 2
1895-1898  SPI 读地址 OR BMI3_SPI_RD_MASK (0x80)
1900       dev->read(reg_addr, temp_buf, len + dev->dummy_byte, intf_ptr)
1901       读回后 delay_us(2)
1905-1909  data[index] = temp_buf[index + dev->dummy_byte]
1938-1941  SPI 写地址清除读位（& 0x7F）
1992-1999  soft reset 后，SPI 再执行一次 CHIP_ID dummy read
```

这里的 `dummy_byte=1` 是 **SensorAPI 回调返回缓冲区中的有效前缀数量**，
不是完整 wire transaction 的总字节数。回调本身收到寄存器地址和
“payload + 1”的读取长度；如果回调以全双工 SPI 发送 1 个地址字节，那么线
上总时钟数就是 `1 (address) + 1 (dummy) + len (payload)`，即 `len + 2`。
CS、SPI 控制器如何把地址、dummy 和 payload 组成一个连续 transaction，由回调实现决定。Bosch
COINES 示例只在 `common.c:250` 配置 `COINES_SPI_MODE0` 和 10 MHz，随后把
事务交给 `coines_read_spi()`（`:319-325`）；这些 COINES 内部 CS 细节不在
SensorAPI 仓库的 callback 代码中展开。

## 4. Zephyr BMI323 SPI 逐行对应

来源：Zephyr `drivers/sensor/bosch/bmi323/bmi323_spi.c`。

```text
22-23  address = { offset | 0x80, 0x00 }
25-29  TX buffer 长度 = 2 字节
31-34  RX[0] 丢弃 2 字节；RX[1] 接收 words_count * 2 字节 payload
39     一次 spi_transceive_dt(spi, TX, RX)
41     transaction 返回后 k_usleep(2)
55     写地址 = offset & 0x7F
57-63  写操作由“地址 buffer + word payload buffer”组成
65     一次 spi_write_dt(spi, ...)
```

对一个 16-bit BMI323 word（例如 CHIP_ID 或一个配置寄存器），Zephyr 的
读传输请求为：

```text
MOSI: 80 00 00 00
MISO: xx xx <word_LSB> <word_MSB>
      ^^^^^  两个 RX 前缀字节被丢弃
```

`spi_transceive_dt()` 的 CS 由 `spi_dt_spec`/控制器驱动包住整个组合传输；
BMI323 驱动文件自身没有手工拉高/拉低 CS。CS 保持时间、SPI mode 和频率
来自 Zephyr 的 SPI device-tree/controller 配置，不由 `bmi323_spi.c` 固定。

## 5. 五项检查结果

| 检查项 | 当前工程 | Bosch SensorAPI | Zephyr BMI323 SPI | 判定 |
| --- | --- | --- | --- | --- |
| 1. CS 是否覆盖整个 transaction | `bmi323_port_spi_read()` 在 `:116-121` 先 CS LOW，单次 SPI 返回后 `bmi323_port_finish_transaction()` 在 `:45-58` 才 CS HIGH；读写均如此。 | SensorAPI 核心不拥有 CS；由 `dev->read`/`dev->write` 回调负责。COINES 示例把总线交给 `coines_*_spi`。 | `spi_transceive_dt()`/`spi_write_dt()` 通过 `spi_dt_spec` 管理 CS；一次 API 调用对应一个 transaction。 | **当前：是（源码路径确认）**；Bosch：回调责任；Zephyr：是（SPI API 语义） |
| 2. 是否一次 HAL_SPI_TransmitReceive | `bmi323_read_reg()` 只调用一次 port read；BSP 在 `:114-116` 只调用一次 `HAL_SPI_TransmitReceive()`。 | 没有 HAL 约束；只有一次 `dev->read()` 回调调用。 | 一次 `spi_transceive_dt()`，但不是 STM32 HAL 接口。 | **当前：是**；参考实现均为单次底层总线调用 |
| 3. dummy byte 处理 | `frame_length = len + 1`，TX 仅显式填 `tx[1]`，payload 从 `rx[index+1]` 取（`:218,227-228,251-254`）。`len=1` 的 CHIP_ID 调用只有 2 个 SPI 字节，`len=2` 只有 3 个。 | SPI `dummy_byte=1`；核心要求 callback 返回 `len+1` 字节并从 `temp_buf[index+1]` 取 payload（`:1827-1835,1900-1909`）。由于地址字节由 callback/总线层另行发送，完整 wire 长度通常为 `len+2`。 | 明确发送 2 字节 address `{read_addr,0}`，丢弃 RX 前 2 字节，再接收 `words_count*2` payload（`:22-39`）。一个 word 的典型总长度为 4 字节。 | **当前比 Bosch/Zephyr 的完整 wire 读帧少 1 个时钟/前缀字节；当前 `rx[index+1]` 偏移不能直接视为 BMI323 payload 偏移。** |
| 4. SPI mode | `bsp_spi.c:49-57`：CPOL LOW、CPHA 1EDGE、MSB first，即 mode 0。 | COINES 示例 `common.c:250` 明确 `COINES_SPI_MODE0`。 | `bmi323_spi.c` 不设置 mode；由 `spi_dt_spec` 的 device tree flags 继承，当前文件不能单独证明 mode。 | **当前 mode 0；与 Bosch 示例一致；Zephyr 需查具体 board DTS。** |
| 5. SPI frequency | `bsp_spi.c:55-56` 使用 SPI123 240 MHz、`/128`，静态计算约 1.875 MHz；IOC `Smart_Car_H757.ioc:347` 为 `RCC.SPI123Freq_Value=240000000`。 | COINES 示例为 10 MHz（`common.c:250`）。这是示例值，不是 SensorAPI 核心的硬编码上限。 | `bmi323_spi.c` 不固定频率；由 DTS 的 `spi-max-frequency`/controller 配置决定。 | **当前频率明显低于 Bosch 示例但通常适合 bring-up；不是与 Zephyr 可直接比较的固定值。运行时实际 SCK 仍需波形确认。** |

## 6. 关键结论

### 已确认的匹配项

1. 当前 CS 软件控制覆盖单次 HAL transaction 的前后；没有在 HAL 调用中间
   释放 CS。
2. 当前读路径确实只执行一次 `HAL_SPI_TransmitReceive`。
3. 当前使用 MSB-first、SPI mode 0，与 Bosch COINES 示例一致。

### 需要重点修正/验证的差异

1. **当前读帧比 Bosch/Zephyr 的完整 wire 读帧少一个字节。** 当前通用接口用
   字节长度 `len`，直接向 HAL 请求 `len+1` 个全双工字节；Bosch SensorAPI
   的回调长度虽为 `len+1`，但地址字节由回调另行产生，因此线上的总长度是
   `len+2`。Zephyr 对一个 16-bit word 也明确形成 4-byte frame（2 个前缀 +
   2-byte payload）。当前启动调用 `len=1` 更明确地只产生 `80 00` 两个字节，
   却把单个 `RX[1]` 当作 CHIP_ID。
2. **Bosch 的 `dummy_byte=1` 不能直接拿来证明当前 HAL 帧正确。** 它描述的
   是 SensorAPI callback 返回缓冲区偏移；当前工程把 command byte 也放进同一
   `HAL_SPI_TransmitReceive` 缓冲区，必须按线上的完整字节数与 RX 偏移验证。
3. **Zephyr mode/frequency 不是 BMI323 驱动文件常量。** 需要查看实际 board
   DTS 后，才能对具体 Zephyr 工程给出 mode/frequency 数值。

## 7. 建议的最小验证（不在本任务执行）

| 步骤 | 观察点 | 通过标准 |
| --- | --- | --- |
| 静态 | 保持当前源码不变，确认调用链和长度参数 | 启动 `read_reg(len=1)` 的 HAL size 应为 2；通用 `read_reg(len=2)` 的 HAL size 应为 3 |
| 逻辑分析仪 | 同时抓 CS、SCK、MOSI、MISO | 比较当前 `80 00 00` 与 Zephyr/Bosch 适配器预期的连续 word 帧 |
| 设备 | 读取 CHIP_ID 的完整 RX 字节 | 16-bit word 的低字节为 `0x43`，并确认 payload 偏移，而非只看 HAL 返回 `HAL_OK` |
| 频率 | 测量 SCK 周期 | 与静态 1.875 MHz 计算值相符；不能用源码值替代实测 |

## 8. 修改与证据边界

- 修改文件：仅新增本文档 `BMI323_BOSCH_SPI_COMPARE.md`。
- 未修改：所有 C/H、CubeMX IOC、CMake、FreeRTOS、LSM303、IMU 管理器及协议。
- 验证：完成源码行号、当前构建路径、Bosch 固定版本和 Zephyr 主线文件的静态核对；未执行构建、烧录、串口采集或 SPI 波形测试。
- 因此本文可以确认软件源码的调用与帧构造差异，不能确认 BMI323 实物是否已
  返回 `0x43`，也不能把 1.875 MHz 视为运行时示波器实测值。
