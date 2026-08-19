# BMI323 初始化卡死只读分析

## 范围与证据

- 审查日期：2026-08-09。
- 范围：仅当前 CM7 活动目标中的 BMI323 初始化、SPI1 BSP、UART 日志和
  FreeRTOS 启动顺序。
- 未修改 `bmi323.c`、`bmi323_port.c`、`imu_manager.c`、SPI 配置或其他源文件。
- 这是源代码静态审查。没有连接调试器、烧录或采集新的运行日志，因此不能把
  单条日志当作某个运行时 PC 的直接证据。

`STM32H757/CM7/CMakeLists.txt:24-25` 明确编译的是
`Middleware/Sensor/BMI323/bmi323.c` 和 `bmi323_port.c`。并存的
`Drivers/IMU/BMI323/` 不是本报告的执行路径。

## 结论

**不能仅凭 `[BMI323][SPI_CONFIG]` 精确确认 CPU 已卡在某一条指令。**
该日志之后，当前源码没有在每一步打印标记，因而可将位置缩小到下面两个有轮询
的区间，但不能在没有现场寄存器/调用栈的情况下把其中一个写成已证实事实。

| 优先级 | 最早/首个可能阻塞点 | 精确位置 | 静态结论 |
| --- | --- | --- | --- |
| P1 | `bmi323_port_delay_ms(10)` | `STM32H757/Middleware/Sensor/BMI323/bmi323.c:276` -> `bmi323_port.c:172-179`，循环为 `:176` | 这是 `[SPI_CONFIG]` 返回后的**第一个无条件轮询**。若 `HAL_GetTick()` 不再递增，循环永久不退出。工程的 `SysTick_Handler()` 在 `STM32H757/CM7/Core/Src/stm32h7xx_it.c:324-331` 调用 `HAL_IncTick()`，所以正常中断运行时应在约 10 ms 后退出。 |
| P2 | 首笔 WHO_AM_I 的 SPI 轮询 | `STM32H757/BSP/SPI/bsp_spi.c:114-116` -> `HAL_SPI_TransmitReceive()`，`STM32H757/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c:1318` | 正常 HAL tick 运行时，传入 timeout 为 20 ms，失败应返回而不是永久卡死。若 `HAL_GetTick()` 冻结，HAL 的轮询循环也会持续自旋。 |

因此，日志现象更精确的表述是：**最后可确认完成的是 SPI 配置日志调用；程序尚未
产出首笔 WHO_AM_I 事务完成后的日志。** 它不能单独证明 CS、SPI 或 UART 已永久
卡死。

## A. 精确位置与日志后的控制流

`[BMI323][SPI_CONFIG]` 的唯一产生点是：

```text
STM32H757/Middleware/Sensor/BMI323/bmi323_port.c:28-38
  bmi323_port_log_spi_config()
  -> uart_log_write(line, 100 ms)
```

该函数在 `bmi323_port_init()` 的 `:93` 调用。日志调用返回后，代码按如下顺序执行：

```text
bmi323_port_init() bmi323_port.c:93
  -> return bmi323_port_cs_high()                         :94
     -> bsp_gpio_write(PC4, HIGH)                         :112
  -> bmi323_init() 再次 bmi323_port_cs_high()             bmi323.c:272
  -> bmi323_port_delay_ms(10)                             bmi323.c:276
     -> while (HAL_GetTick() - start < 10)                bmi323_port.c:176
  -> bmi323_spi_probe()                                   bmi323.c:277
     -> bmi323_port_init()                                bmi323.c:338
        （配置日志被静态标志抑制，见 bmi323_port.c:23-26）
     -> bmi323_port_spi_read()                            bmi323.c:341
        -> bmi323_port_cs_low()                           bmi323_port.c:139
        -> bsp_spi_write_read()                           bmi323_port.c:141
           -> HAL_SPI_TransmitReceive()                   bsp_spi.c:114-116
```

### CS 检查

CS 仅经过 `bsp_gpio_write()`：

- 拉低：`STM32H757/Middleware/Sensor/BMI323/bmi323_port.c:97-107`。
- 拉高：`STM32H757/Middleware/Sensor/BMI323/bmi323_port.c:110-118`。
- GPIO 写入本体：`STM32H757/BSP/GPIO/bsp_gpio.c:75-88`，直接调用
  `HAL_GPIO_WritePin()`，没有 BUSY 等待、循环、mutex 或 semaphore。

结论：**CS 拉低/拉高不是源码中的阻塞点。** `bmi323_port_cs_low()` 成功后只有
2 us 的延时；它也依赖 DWT 周期计数在
`bmi323_port.c:163-170` 自旋，若 DWT `CYCCNT` 不递增才会卡住，但在当前日志之后
它位于 P2 SPI 区间内部，不是最早的 P1 位置。

### 调用链校正

用户给出的链条中，`bmi323_read_reg(CHIP_ID)` 并不是首笔 WHO_AM_I 事务的调用者。
当前实际路径如下：

```text
main()                                                       main.c:195
  -> imu_runtime_start()                                     imu_runtime.c:501
    -> imu_init()                                            imu_manager.c:538-542
      -> imu_init_internal()                                 imu_manager.c:480
        -> bmi323_init()                                     imu_manager.c:522
          -> bmi323_port_init()                              bmi323.c:268
          -> bmi323_spi_probe()                              bmi323.c:277
            -> bmi323_port_init()                            bmi323.c:338
            -> bmi323_port_spi_read()                        bmi323.c:341
              -> bsp_spi_write_read()                        bmi323_port.c:141
                -> HAL_SPI_TransmitReceive()                bsp_spi.c:114-116
```

`bmi323_read_reg(BMI323_REG_CHIP_ID, ...)` 仅在首笔 probe 成功、软复位已发送后才发生：
`STM32H757/Middleware/Sensor/BMI323/bmi323.c:292-299`。它同样经由
`bmi323_port_spi_read()`，但不是当前日志之后的第一笔读。

## B. SPI 阻塞审查

### 是否卡在 `HAL_SPI_TransmitReceive()`

存在同步调用，位置为：

```text
bmi323_port_spi_read()                         bmi323_port.c:141
  -> bsp_spi_write_read()                       bsp_spi.c:105-145
    -> HAL_SPI_TransmitReceive(&hspi1_bsp, ...) bsp_spi.c:114-116
```

传入的 `timeout_ms` 是 `BMI323_SPI_TIMEOUT_MS = 20`：
`STM32H757/Middleware/Sensor/BMI323/bmi323.c:11`，在 probe 的 `:341` 传入。

HAL 8-bit 轮询主体在 `stm32h7xx_hal_spi.c:1545-1607`；EOT 等待在 `:1613-1617`，
其帮助函数的 `while` 在 `:3972-3985`。这些循环都以 `HAL_GetTick()` 与 20 ms
timeout 退出。没有传入 `HAL_MAX_DELAY`，也没有 BMI323 自己的 `while (BUSY)`。

**例外条件：** 若 SysTick/HAL tick 停止，则 `HAL_GetTick()` 不变，HAL 的 timeout
判定也永远不成立；此时 P1 的 10 ms 延时和 P2 的 HAL SPI 轮询都会表现为永久卡死。
源码显示 SysTick handler 会无条件先调用 `HAL_IncTick()`
(`stm32h7xx_it.c:324-329`)，故该条件需要由现场确认，而非已证实。

### `HAL_SPI_GetState()`、`HAL_SPI_GetError()` 与当前状态

| 项目 | 源码事实 | 当前运行值 |
| --- | --- | --- |
| `HAL_SPI_GetState()` | HAL 实现在 `stm32h7xx_hal_spi.c:3307-3311`，返回 `hspi->State`；活动 BMI/BSP 路径没有调用它。 | **未知**。`hspi1_bsp` 是 `bsp_spi.c:10` 的 `static` 私有对象，未提供 state getter。 |
| `HAL_SPI_GetError()` | HAL 实现在 `stm32h7xx_hal_spi.c:3319-3323`，返回 `hspi->ErrorCode`；活动 BMI/BSP 路径没有调用它。 | **未知**。没有运行时读取或日志。 |
| 调用入口状态 | HAL 在 `:1343-1346` 要求 `HAL_SPI_STATE_READY`；否则立即返回 `HAL_BUSY`。 | 静态期望为 `READY`，不能证明设备当时确实为该状态。`HAL_BUSY` 是立即返回，不是等待。 |
| 调用期间状态 | HAL 在 `:1357` 置 `HAL_SPI_STATE_BUSY_TX_RX`，在完成/timeout 清理路径置回 `READY`（如 `:1601`、`:1622`）。 | 若断点命中 HAL 轮询，预期为 `BUSY_TX_RX`；无现场断点则未知。 |
| 错误码 | HAL 在 `:1358` 先清为 `HAL_SPI_ERROR_NONE`。RXP timeout 会在 `:1595-1606` 置 `HAL_SPI_ERROR_TIMEOUT`；EOT wait 失败在 `:1614-1617` 置 `HAL_SPI_ERROR_FLAG`。 | 若函数尚未返回，`bsp_spi_get_last_hal_status()` 仍可能是前次值（初始 `-1`），因为赋值在 `bsp_spi.c:114-116` 的 HAL 调用返回后才完成。 |

`bsp_spi_get_last_hal_status()` 只能返回最后一次**已返回**的 HAL 状态
(`STM32H757/BSP/SPI/bsp_spi.c:148-151`)；它不等价于当前 `State` 或 `ErrorCode`。

### SPI1 状态异常与 timeout

- `bsp_spi_init()` 将 `spi_ready` 置位后不再重新初始化
  (`STM32H757/BSP/SPI/bsp_spi.c:39-77`)；`bmi323_spi_probe()` 的第二次
  `bmi323_port_init()` 会快速返回。
- 若 SPI 状态不是 READY，HAL 在入口立即 `HAL_BUSY` 返回。BSP 会映射为
  `BSP_STATUS_ERROR`（`bsp_spi.c:15-24`），并继续 CS 收尾；这不是永久阻塞。
- `bmi323_port_apply_spi_bringup_config()` 在 `bmi323_port.c:41-46` 直接清 SPI1
  `SPE` 并调整分频。该函数没有等待；之后 HAL 自己在
  `HAL_SPI_TransmitReceive():1386-1392` 使能并启动主机事务。
- 静态审查不能读取此次失败时的 SPI 寄存器、`State`、`ErrorCode`、`SR` 或实际
  `HAL_GetTick()` 增量，故不能把“SPI1 状态异常”升级为已证实根因。

## C. timeout、死循环与同步原语检查

| 检查项 | 结论 | 证据 |
| --- | --- | --- |
| BMI323 的 `while (SPI BUSY)` | 未发现 | BMI323 port/BSP 不轮询 `HAL_SPI_GetState()` 或 SPI BUSY 标志。 |
| SPI HAL 轮询 | 有，且理论上 20 ms 超时 | `HAL_SPI_TransmitReceive()`，见上节。只有 HAL tick 不动时才无期限。 |
| BMI323 ms/us 延时 | 有自旋 | `bmi323_port.c:167`、`:176`；分别依赖 DWT cycle counter 与 `HAL_GetTick()`。 |
| BMI323/SPI mutex/semaphore | 未发现 | `bmi323.c`、`bmi323_port.c`、`bsp_spi.c` 均不使用 `xSemaphoreTake`。 |
| IMU data mutex | 存在但不在初始化调用之前 | `imu_manager.c:108`、`:122` 的数据锁；`bmi323_init()` 在 `imu_init_internal():522` 直接调用，未持有 BMI 数据锁。 |
| FreeRTOS assert 死循环 | 作为通用风险存在，非当前链条的直接证据 | `Config/FreeRTOSConfig.h:65-71` 的 `configASSERT` 会关中断并 `for (;;)`。此时 HAL tick 会停止，从而可使任何 tick timeout 表现为卡死。 |

## D. `uart_log_write` 是否阻塞

`[BMI323][SPI_CONFIG]` 的发送路径是同步的：

```text
bmi323_port_log_spi_config()                     bmi323_port.c:38
  -> uart_log_write()
    -> bsp_uart_log_write_level()                 bsp_uart.c:221-235
      -> uart_log_write_usart1()                  bsp_uart.c:165-184
        -> uart_transmit_locked()
          -> xSemaphoreTake(uart_tx_mutex, 100)   bsp_uart.c:73
          -> HAL_UART_Transmit(..., <=100 ms)     bsp_uart.c:97-98
      -> uart_log_write_usart2()
        -> uart_link_send(..., <=20 ms)           bsp_uart.c:186-219
```

结论：

- `uart_log_write()` **会同步等待** USART1 TX mutex 和 `HAL_UART_Transmit()`；但本次
  timeout 是 `BMI323_PORT_LOG_TIMEOUT_MS = 100 ms`
  (`bmi323_port.c:13`)，不是无限等待。
- `uart_transmit_locked()` 对 mutex 使用 `pdMS_TO_TICKS(timeout_ms)`
  (`bsp_uart.c:73`)，并将剩余时间传给 HAL（`:80-103`）；USART1 HAL 发送同样使用
  tick timeout（`stm32h7xx_hal_uart.c:1155-1187`）。
- USART2 发送由 `uart_link_send()` 再使用独立 mutex 和 20 ms timeout
  (`uart_link.c:125-163`; `uart_link.h:16`)。
- 初始化发生在 `vTaskStartScheduler()` 前：`main.c:195` 先执行
  `imu_runtime_start()`，`main.c:200` 才启动调度器。此时两个 TX mutex 已在
  `main.c:179-182` 初始化，且没有已运行任务能持有它们，因此首个 `xSemaphoreTake`
  静态上应立即取得，不构成 mutex 等待死锁。
- 如果 HAL tick 正常，日志路径最多造成有限延迟/timeout，不能解释永久停住。
  如果日志字节已完整出现在 USART1，上述 USART1 `HAL_UART_Transmit()` 已至少把该
  字节流送出；仍不能据此排除随后 USART2、延时或 SPI 阶段的问题。

## 运行时确认清单（未执行）

要把 P1/P2 从候选提升为精确根因，应在复现时暂停 CM7 并记录：

1. PC/LR 和调用栈：是否位于 `bmi323_port_delay_ms():176`，还是
   `HAL_SPI_TransmitReceive():1545-1607`/`:3976`。
2. `HAL_GetTick()` 连续两次读取的值，以及 `PRIMASK`/异常状态；若不递增，先定位
   SysTick 被屏蔽、HardFault 或 `configASSERT`。
3. `hspi1_bsp.State`、`hspi1_bsp.ErrorCode`、`SPI1->SR`、`SPI1->CR1`；当前 BSP
   未导出前两个值，不能从既有日志获得。
4. CS PC4、SCK PA5、MOSI PA7、MISO PA6 的逻辑分析仪波形。源代码不能证明实际电平、
   时钟或器件响应。
5. 若 PC 在 UART：`uart_tx_mutex`/`s_tx_mutex` 是否被占用、UART `gState` 与
   `HAL_GetTick()` 是否递增。

## 验证边界

本报告已完成源代码调用链、轮询、timeout、CS、mutex/semaphore、SPI 状态接口与 UART
日志路径审查。它没有产生设备运行、SPI 波形、UART 抓包或调试器寄存器证据；硬件连接、
SPI1 实际状态和最终卡死 PC 仍属 **UNVERIFIED**。
