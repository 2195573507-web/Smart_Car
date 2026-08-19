# BMI323 日志路径检查报告

检查范围：只读源码审查。未修改代码、工程配置或通信协议；未执行设备运行验证。

## 1. BMI323 日志入口

当前入口位于：

```text
STM32H757/Middleware/Sensor/BMI323/bmi323.c
```

- `bmi323_log_whoami_raw()` 在 `bmi323.c:84-120` 组装 `[BMI323][WHOAMI_RAW]` 和 `[BMI323][SPI_RAW]`。
- `bmi323_init()` 首次 WHO_AM_I 读取后在 `bmi323.c:302-304` 调用该日志函数。
- 日志最终调用 `uart_log_write(line, BMI323_LOG_TIMEOUT_MS)`（`bmi323.c:119`、`171`、`182`）。
- BMI323 没有直接调用 BLE、FFE3 或 `printf` 输出；`snprintf` 只负责格式化文本。

## 2. STM32 日志出口

入口映射：

```text
uart_log_write()
  -> bsp_uart_log_write()
  -> bsp_uart_log_write_level(INFO, ...)
```

证据：

- `bsp_uart.h:68-71`：`uart_log_write()` 是 BSP 包装函数。
- `bsp_uart.c:246-249`：默认日志级别为 INFO。
- `bsp_uart.c:221-234`：同一条文本先尝试 USART1 调试输出，再调用 USART2 链路输出。
- 未发现独立的 `UART_LOG`、`STM_LOG` 或 `LOG_TX` 发送函数；当前实际出口是上述 `bsp_uart_log_write_level()`。
- `type=0x30` 不是独立发送路径，而是 `SC_TYPE_LOG` 枚举值；SCBP-V3 链路使用其映射的 `SCBP_MSG_ID_LOG=0xF000`。

USART1 失败不会阻止 USART2：只有参数非法时才跳过 USART2（`bsp_uart.c:230-233`）。

## 3. SCBP LOG 封装与 USART2

STM32 USART2 日志封装路径：

```text
bsp_uart_log_write_usart2()
  -> payload[0..7] = source/level/timestamp/text_length
  -> sc_frame_encode(SC_TYPE_LOG, ...)
  -> uart_link_send(frame, frame_length)
  -> HAL_UART_Transmit(USART2, ...)
```

关键事实：

- `SC_TYPE_LOG = 0x30`，并映射为 `SCBP_MSG_ID_LOG = 0xF000`（`STM32H757/Middleware/Communication/SmartCar_Frame/sc_frame.h:68,125`、`sc_frame.c:114-136`）。
- `sc_frame_encode()` 使用 STM32 为 source、S3 为 destination（`sc_frame.c:311-317`）。
- `uart_link.h:13-16` 定义 `USART2`、115200 baud。
- `uart_link.c:125-162` 使用 `HAL_UART_Transmit()` 发出帧。
- `CM7/Core/Src/main.c:177-199` 初始化 USART1、`uart_link_init()` 和 UART link task。

### 单条日志长度限制

`bsp_uart_log_write_usart2()` 在 `bsp_uart.c:198-201` 将单条文本截断为 `BSP_UART_LOG_TEXT_MAX=96`（`bsp_uart.h:29`）。当前 BMI323 将 WHOAMI 和 SPI_RAW 合并在一条文本中，因此：

- SCBP LOG 帧可以进入 USART2；
- BLE 桥最多收到该条文本的前 96 字节；
- SPI_RAW 的后部字段可能在 STM32 出口处已经丢失。

## 4. S3 UART 与 BLE 桥

S3 接收路径：

```text
STM32 USART2 TX
  -> S3 UART_NUM_2 RX(GPIO18)
  -> stm_uart_task() 接收并存入环形存储
  -> smartcar_service_task()
  -> sc_frame_parser_feed()
  -> command_bridge_on_frame()
  -> log_bridge_handle()
  -> s3_ble_log_notify_send()
  -> FFE3 Notify characteristic
```

证据：

- `ESPS3/components/stm_uart/include/stm_uart.h:10-14`：UART2、GPIO17 TX、GPIO18 RX、115200 baud。
- `stm_uart.c:62-90`：UART2 接收 task；`stm_uart.c:167-211`：UART2 发送 API。
- `ESPS3/components/smartcar_service/command_bridge.c:355-377`：从 `stm_uart_receive_nonblock()` 取数据并喂给 SCBP parser。
- `command_bridge.c:244-274`：校验目标后，`SCBP_MSG_ID_LOG` 调用 `log_bridge_handle()`。
- `log_bridge.c:15-52`：校验 LOG payload/source/level/length，重新编码为 SmartCarLog，并调用 `s3_ble_log_notify_send()`；STM source 会输出 `STM_LOG_RX`。
- `s3_ble.c:34-45`：FFE3 UUID 和 Notify 属性。
- `s3_ble.c:472-501`：只有 BLE 已初始化、客户端已连接且 LOG CCC Notify 已启用时才发送 FFE3。
- `s3_ble.c:362-383`：客户端写入 LOG CCC 后启用 Notify，并刷新 pending logs。

## 5. App 接收条件

macOS App 使用 FFE3 接收 STM32 日志：

- `BLEManager.swift:76`：日志 characteristic 为 `0000FFE3-0000-1000-8000-00805F9B34FB`。
- `BLEManager.swift:315-343`：发现 FFE3 后调用 `setNotifyValue(true, ...)`。
- `BLEManager.swift:348-373`：FFE3 数据进入 `SmartCarLogParser`，解析后按 source 写入 `stmLogStore` 或 `s3LogStore`。
- `SmartCarLog.swift:3-5,59-127`：要求 AA55/version/source/level、payload <=96、CRC 正确。
- `DeviceLogView.swift:14-16` 默认过滤 INFO 及以上；BMI323 使用 INFO 日志入口，因此满足默认显示级别。

## 6. 结论与证据边界

从源码看，BMI323 日志具备完整的 STM32 -> USART2 -> S3 parser -> `UART_LOG_BRIDGE` -> FFE3 Notify -> App 接收路径，但存在 96 字节单条日志截断边界。能否在 BLE 看到日志还取决于：

1. STM32 UART link 初始化成功且物理 USART2 链路连通；
2. S3 UART2 接收/SCBP CRC 校验通过；
3. S3 已连接 BLE 客户端且 FFE3 LOG Notify CCC 已启用；
4. App 处于连接状态并成功解析 SmartCarLog CRC。

本报告是源码路径确认，不等同于 STM32、S3、UART、BLE 或 App 的运行验收；当前没有设备抓包或蓝牙日志证据。
