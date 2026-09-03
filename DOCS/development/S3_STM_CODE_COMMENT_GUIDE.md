# S3 与 STM32 代码注释和调试参数规范

## 目的

本规范用于 `ESPS3/`、`STM32H757/` 和实际共享的 `Common/` 自研代码。
目标是让维护者在不阅读函数实现细节的情况下，能够确认接口职责、调用上下文、
参数单位、失败语义和安全边界。注释属于源码契约说明，不改变运行行为。

## 函数注释字段

公共函数和跨模块调用的关键静态函数，使用中文 Doxygen 风格块注释，至少包含：

```c
/**
 * @brief  一句话说明职责和数据流方向。
 * @author 创建人：Zhiqin / 待确认（以 Git 首次引入记录为准）。
 * @date   创建或本次契约补充日期：YYYY-MM-DD。
 * @param  name 参数含义、单位、所有权、允许的 NULL/范围。
 * @return 返回值含义、错误条件以及调用方必须采取的动作。
 * 调用方式：调用时机、线程/任务/ISR 上下文、初始化前置条件和示例。
 * 线程约束：是否可重入、是否持锁、是否会阻塞、缓冲区生命周期要求。
 */
```

无法从当前 Git 历史可靠确认首次创建人的函数，不得猜测姓名，统一写
“创建人：待确认（当前维护人：Zhiqin）”，并保留现有代码作者信息。
供应商、CubeMX/ESP-IDF 生成代码不批量改写；其原有英文版权/作者头保持不动。

### 声明与定义的权威边界

- 公共 API 的详细契约以项目自有头文件中的声明为唯一权威；`.c` 定义处可保留简短中文
  实现摘要，避免参数/失败语义形成两份容易漂移的副本。
- 所有项目自有 `static` 函数必须在定义处使用完整中文 Doxygen，因为它们没有外部声明可承载契约。
- HAL/GATT/RTOS 等框架回调如果没有项目自有头文件声明，也必须在 `.c` 定义处完整说明参数、
  同步调用栈、ISR/任务上下文、借用缓冲生命周期和允许的 API。
- 工作树可能并发增加源文件和接口，完成检查必须动态枚举当前路径，不能复用固定旧文件清单。

## 调试宏规则

- 跨模块复用的调试开关、原始抓包长度、日志周期和诊断探针容量集中放在
  `Common/SmartCarDebug/smartcar_debug_config.h`。
- 每个宏必须写明默认值、单位、用途、对实时性的影响和是否允许生产固件开启。
- 使用 `#ifndef` 保留编译器预定义覆盖能力；宏迁移不得改变默认值。
- `cmake -D` 或 `idf.py -D` 设置的是 CMake cache 变量，只有 `CMakeLists.txt`
  显式声明并传入 `target_compile_definitions()` 的选项才会成为 C 预处理宏。
  其余数值参数应直接修改统一头，或由目标 CMake 显式增加编译器定义。
- 协议 ID、帧长度、CRC、GPIO、PWM/速度限幅、看门狗和安全超时不属于“调试宏”，
  继续由原模块头文件或协议注册表维护，避免配置漂移。
- 打开高频日志前应确认 UART/BLE 队列、任务栈和链路带宽余量；日志不能绕过急停、
  同步、姿态有效性或 BUS_OFF 门控。

### 当前统一宏清单

| 类别 | 宏示例 | 默认值 | 主要使用方 |
| --- | --- | ---: | --- |
| 诊断镜像 | `SMARTCAR_BMI323_DEBUG_ONLY`、`SMARTCAR_RAW_DIAGNOSTICS` | `0` | S3/CM7 构建选项和原始探针 |
| 原始数据日志 | `IMU_RUNTIME_ENABLE_RAW_DATA_LOG`、`RADAR_UART_RAW_LOG_ENABLED`、`RADAR_BLE_RAW_UART_LOG_ENABLED` | `0` | CM7 IMU、S3 雷达 UART/BLE |
| 抓包上限 | `STM_UART_BOOT_CAPTURE_BYTES`、`SRP_DEBUG_RAW_FRAME_MAX_BYTES`、`RADAR_*_HEX_LOG_BYTES` | 32/64/32/20 | UART/SRP/雷达诊断 |
| 日志节流 | `*_LOG_PERIOD_MS`、`*_INTERVAL_MS` | 见头文件 | S3/STM 服务、姿态、雷达上行 |
| 诊断资源 | `CM7_RAW_DIAG_WAIT_SPINS`、`CM7_RAW_DIAG_SLOT_COUNT` | 20000/16 | CM7 原始探针 |

当前统一头含 34 个实际配置宏（不含 include guard），完整默认值和单位以
`Common/SmartCarDebug/smartcar_debug_config.h` 为准；表格只作导航。

## 调用上下文约定

注释必须明确以下至少一项：

| 上下文 | 必须说明 |
| --- | --- |
| 普通任务 | 是否会阻塞、最长等待、需要的锁和调用周期 |
| 中断/回调 | 允许的 API、是否只能复制/置位标志、输入指针有效期 |
| 初始化 | 调用顺序、失败后的回滚/重试方式 |
| 跨芯片接口 | 帧所有权、字节序、序号/时效、同步和安全门 |
| 缓冲区接口 | 读写所有权、容量、是否复制、短缓冲行为 |

## 调用示例

### S3 接收并解析 SRP

```c
uint8_t rx[256];
int received = stm_uart_receive_nonblock(rx, sizeof(rx));
if (received > 0) {
    (void)srp_parser_feed(&parser, rx, (size_t)received);
}
```

`stm_uart_receive_nonblock()` 只搬运字节，`srp_parser_feed()` 才负责帧定界和
CRC；完整帧回调必须复制 payload 后再交给业务队列。

### STM32 通过服务层发送 SRP

```c
uint8_t payload[SRP_PAYLOAD_CHASSIS_STATE_SIZE] = {0U};
int result = s3_service_send_message(
    SRP_PRIORITY_TELEMETRY,
    SRP_MSG_ID_CHASSIS_STATE,
    SRP_FLAG_STREAM_DATA,
    payload,
    (uint8_t)sizeof(payload));
if (result != 0) {
    /* 保留停止/降级状态，并记录有界诊断；不要绕过服务层重发。 */
}
```

`s3_service_send_message()` 统一执行会话、BUS_OFF、锁和发送准入。普通业务代码不得
自行编码后直接调用 `uart_link_send()`；底层入口只供服务层和隔离诊断使用。

### 开启受限诊断

```text
cmake -S STM32H757/CM7 -B STM32H757/CM7/build/Debug \
  -DSMARTCAR_RAW_DIAGNOSTICS=ON
```

诊断完成后应恢复 `OFF` 并重新生成 `STM32H757/CM7/build/Debug`，避免把原始日志带入
默认安全镜像。S3 的 `SMARTCAR_BMI323_DEBUG_ONLY` 已由工程 CMake 映射，可通过
`idf.py -D SMARTCAR_BMI323_DEBUG_ONLY=1 build` 在独立构建目录验证。其他统一头宏
不能假定 `idf.py/cmake -D` 自动生效，应按本节规则配置并检查 `compile_commands.json`。

## 验证要求

1. 用 `rg` 检查公共声明前是否有中文字段，确认没有把注释误写成代码。
   本任务可使用 `.planning/s3-stm-cn-comments/audit_header_comments.pl` 逐声明检查
   `@author`、`@date`、参数/返回值、调用方式和线程约束；脚本同时覆盖函数指针回调。
   使用 `.planning/s3-stm-cn-comments/audit_source_comments.pl` 配合
   `AUDIT_STATIC_ONLY=1 AUDIT_SUMMARY=1` 检查生产 `.c` 的所有内部函数；未在项目头声明的
   框架回调再用无 static 过滤的结果单独复核。
2. 用 `git diff --check` 检查空白和换行。
3. 运行受影响的主机测试和构建；构建通过只证明源码/链接，不证明 UART、BLE、传感器或车辆行为。
4. 对调试宏分别检查默认关闭和显式开启的预处理结果，避免把调试路径误编入安全默认镜像。

## 覆盖范围

当前已完成项目自有公共接口、函数指针回调和首批关键静态状态机的逐函数补充：

- S3：`main` 启动、`stm_uart`、`smartcar_protocol`、`smartcar_service`、`s3_ble`、
  `radar_control`，以及雷达 parser/FIFO/telemetry/uplink 链。
- STM32 CM7：BSP（ADC/GPIO/I2C/SPI/PWM/TIMER/UART）、STM-S3 UART/SRP 服务、
  IMU 管理/标定/水平校准/DualAHRS、姿态启动安全、底盘/PID、MotorBoard、日志/RTOS 健康。
- 共享代码：`Common/SRP` 和 `Common/SmartCarLog` 的线缆布局、编解码、CRC、ACK/重试和日志 API。

以下文件不纳入逐函数注释：STM32CubeMX 生成的 `Core/Src`/MSP/启动文件、HAL/CMSIS/FreeRTOS/ESP-IDF
第三方实现、供应商 SDK、测试构建产物和历史归档。它们已有供应商版权/作者头，修改会破坏生成
流程或超出本次注释目标；其调用边界在项目自研头文件中说明。

## 维护记录

- 2026-08-31：纳入并发新增的 FFE3 队列、断连统计、雷达 telemetry age/queue/
  observability/uplink 及其 host tests。当前动态审计结果为：头文件函数/回调/inline
  `checked=462 missing=0`（431 个声明/回调加 31 个 inline）、类型 `129/0`、成员
  `869/0`、STM/共享生产 static `353/0`、S3 生产定义 `254/0`、host test 函数/宏
  `95/0`，另有 4 个 STM 无头声明框架/stdio 入口 `missing=0`。统一头 34 个配置宏均有
  消费者；新增 telemetry 日志周期保持 2000 ms 默认值。
  SRP、BLE、雷达、odometry host tests 均通过；ESP-IDF 5.5.4 生成 S3 binary
  `0x12dc60`（app 分区余 83%），CM7 canonical `build/Debug` clean build 的 FLASH/RAM/
  RAM_D2 为 18.18%/47.78%/0.17%。未烧录，未验证 UART、BLE、传感器、雷达或车辆运行。
- 2026-08-31：前一续轮把覆盖扩展到实现层和自有 host tests。当时动态审计结果为：公共头
  `checked=411 missing=0`，STM/共享生产 static `353/0`，S3 生产函数定义 `215/0`，
  host test 函数/断言宏 `71/0`。公共 API 继续以头文件为权威，所有 static 与未在自有头
  声明的框架回调在定义处写完整契约。SRP、5 组雷达、odometry host tests、S3 构建和
  CM7 `build/Debug` clean build通过；这些证据仍不等同于烧录、UART/BLE、传感器或车辆验收。
- 2026-08-31：完成第二轮全量公共声明复核；审计覆盖 405 个项目自有函数/回调，
  `missing=0`。补充 MotorBoard/RTOS、雷达/BSP、IMU/标定、底盘/PID、SRP/BLE/UART
  等契约，并对跨模块 parser/link/GATT/BUS_OFF/运动事务关键静态函数补充上下文说明。
  该结果只证明字段覆盖和源码/构建一致性，不构成 UART、BLE、传感器或车辆硬件验收。
- 2026-08-31：第二轮修正 UART/BLE/SRP/雷达控制契约，补齐首批高风险公共
  API 的创建人、日期、参数、返回值和线程约束；后续优先补齐雷达、BSP、
  IMU/标定、MotorBoard 和 RTOS health 头文件。
- 2026-08-31：建立首版规范，并将 S3/STM 调试参数统一入口设为
  `Common/SmartCarDebug/smartcar_debug_config.h`。
