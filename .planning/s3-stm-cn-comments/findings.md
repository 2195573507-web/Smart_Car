# S3 与 STM 中文注释复核发现

## 已继承并复核的上下文

- 当前分支为 `codex/s3-stm-cn-comments`，工作树存在大量并发修改。
- 已建立 `Common/SmartCarDebug/smartcar_debug_config.h` 和 `DOCS/development/S3_STM_CODE_COMMENT_GUIDE.md`。
- 前序复审发现的 STM 阻塞发送、S3 UART 初始化/波特率、雷达互斥等待、BUS_OFF 重复回调、SRP 线长与结构体大小等误导注释已修正；本轮仍需从当前文件重新验证。
- 已完成的优先接口批次包括 S3 UART/BLE/服务/雷达控制、STM UART/S3 服务、SRP codec/link、MotorBoard transport/protocol。
- 待重点处理：MotorBoard task、RTOS health/raw diagnostics、雷达/BSP、IMU/标定、PID/底盘/启动/滤波等头文件。

## 不属于本任务的并发修改

- iOS 会话日志相关修改。
- IMU、姿态、LSM303、DualAHRS 行为修改。
- 底盘 freshness 门控和安全协调器行为修改。
- STM/S3 全量审计规划文件。

## 当前协议与验证边界

- STM32 与 S3 当前活动通信协议为 `Common/SRP` 的 SRP v4。
- STM USART2 PA2/PA3 对应 S3 UART2 GPIO17/18，默认 921600 8N1；雷达 UART1/GPIO44 为独立链路。
- 构建成功不证明 UART、BLE、传感器或车辆运行通过。

## 2026-08-31 第二轮首批发现

- `motor_board_task.h` 的 7 个公共接口仍只有单行摘要，缺少逐函数 `@author`、`@date`、参数、返回值和调用上下文。
- `rtos_health.h` 只在个别接口写了简略调用方式，未完整说明 `.noinit` 跨软复位保留、hook-only 限制、快照复制并发条件和 `noreturn` 停机行为。
- `cm7_raw_diag.h` 的接口未完整说明启用诊断时会关闭 IRQ、轮询 USART1、占用静态槽位以及限频规则；这些约束直接影响实时性判断。
- `rtos_health.c` 文件顶部存在两条重复模块说明，应只保留一条，不改变实现。
- `motor_board_task` 的调用点来自底盘任务、S3 服务、安全协调器和启动路径；注释必须明确目标顺序 `[RR, RF, LR, LF]`、临界区复制、强停的串口排队语义，以及查询接口的快照性质。
- `SMARTCAR_SERVICE_TELEMETRY_LOG_PERIOD_MS` 曾在一次搜索时无消费者，但并发中的
  `s3_service.c` 随后新增周期遥测摘要使用点；当前宏及非零校验必须保留。
- `motor_board_force_stop()` 通过 `MB_Protocol_SendPwm()` 将零 PWM 文本排入 USART6 TX ring；返回 true 不代表字节已物理发送、电机板已接收或车辆已停止。
- `motor_board_set_target_wheel_speeds()` 的非零目标成功仅表示四路目标已在临界区复制到 RAM；全零目标会委托 `motor_board_force_stop()`，因此返回值语义不同。
- 采用简单声明结束行扫描并排除 FreeRTOS 第三方后，共有 54 个项目头文件含函数声明，其中 33 个至少包含一个 `@date`；仍有 21 个整文件无日期字段候选，另需检查“部分函数有字段”的漏项。
- 本机 `rg -L` 表示跟随符号链接，不能用作 files-without-match；后续统一使用 `rg --files-without-match`。
- 21 个整文件无日期候选中，`srp_def.h` 与 `srp_registry.h` 仅因 `_Static_assert(...);` 被简单扫描误判，文件内没有函数声明。
- `sc_app_parser_feed()` 当前返回本次成功完成并通过校验的帧数，不是输入消费字节数；原头文件说明错误。
- `srp_wire_read_f32_array_le()` 要求 `length` 精确等于 `count * sizeof(float)`；遇到非有限值时返回 false，但此前元素及当前非有限值可能已经写入输出，失败输出不可使用。
- `smartcar_log_encode()` 失败时不保证改写 `output_length`；payload 与 output 不应重叠。`smartcar_log_decode()` 成功后 payload 只借用输入 frame 生命周期。
- App parser 的实际 owner 是 `command_bridge` 服务任务：初始化位于服务启动，feed 位于 BLE RX 队列消费循环，完整帧/错误回调在 feed 调用栈内同步执行。
- `log_bridge_handle()` 只从同一个序列化服务解析分发点调用；其静态输出缓冲、限频时间和抑制计数没有锁，因此接口必须保持单任务 owner。
- `log_bridge_handle()` 忽略 `s3_ble_log_notify_send()` 返回值。FFE3 未连接/未启用时返回 `ESP_ERR_INVALID_STATE` 且不增加 notify failure 计数，因此原“发送失败通过计数/日志体现”说明不成立；当前可能静默丢弃。
- 新增逐声明只读审计脚本后，当前范围仍有 242 个声明缺少至少一个规范字段；主要是零参数说明、调用方式和线程约束，另有若干整段未注释接口。
- 该 242 是保守审计候选数，不包含 typedef 回调，且需排除解析器假阳性；只有逐文件核对和计数回归后才能作为完成依据。
- 并发修改期间，宏消费者可能在首次搜索后出现；最终构建前必须重新检查统一头每个宏的当前消费者。
- 扩展审计脚本纳入函数指针 typedef、CM7 PID 和 BSP_TEST 后，项目自有头文件全部通过：405 个普通公共声明/回调均具备作者、日期、参数/返回、调用方式和线程约束。
- 统一调试头目前有 33 个实际配置宏（不含 include guard），每个至少有一个头文件外消费者；迁移前后可见默认值保持一致。
- `command_bridge_on_ble_disconnect()` 在 GATT 任务上下文中不仅置停机请求，还会无锁清除两个 pending motion 的 `valid` 位；真正 SRP 零速发送仍由服务任务完成。注释应明确该现状和并发限制，本任务不改变行为。
- `ctags` 为系统 BSD 版本，不支持 Universal/GNU 长选项；关键静态函数改用 BSD `ctags -x`、注册调用点和源码逐段复核。
- `bsp_test_compile()` 只有编译引用、没有运行调用点，但函数一旦显式执行会访问 SPI1/I2C4 等真实 BSP；原“does not touch hardware”说明错误，已改为台架副作用契约。

## 2026-08-31 续轮发现

- 上一轮 `405 missing=0` 只覆盖项目自有头文件的公共声明和函数指针 typedef，不能证明 `.c` 内部函数定义全部具备详细中文契约。
- 当前续轮应把项目自有生产 `.c` 函数定义纳入审计；CubeMX/Core 生成代码、HAL/CMSIS/FreeRTOS、ESP-IDF managed components 继续排除。
- 系统提供 BSD `ctags -x`，可稳定给出 `.c` 函数名与定义起始行，适合用作实现函数审计的权威枚举来源。
- 首次把全部 `.c` 定义都要求重复完整字段时得到 `checked=647 missing=625`；该口径包含已经在头文件具备权威详细契约的公共 API，会造成重复维护。
- 续轮实现层的强制口径调整为：所有 `static` 函数必须在定义处完整说明；公共 API 继续以头文件契约为权威，`.c` 可保留实现摘要；HAL/RTOS 等未在自有头声明的外部回调单独审计。
- BSD ctags 对不同文件中的同名静态函数会打印 duplicate warning；审计脚本改为逐文件调用 ctags，避免噪声且不漏定义。
- 调整为 static-only 后，STM/共享生产代码基线为 `checked=352 missing=330`；缺口最多的文件为 `imu_manager.c` 45、`dual_ahrs.c` 35、两份标定实现各 22、Drivers BMI323 19、MotorBoard task 18、MotorBoard protocol 17、UART Link 16。
- 共享 `Common/SRP` 与 `Common/SmartCarLog` 当前有 19 个缺字段 static helper，是两端共同依赖且适合优先收敛的稳定批次。
- S3 子批次发现现存行为风险：`command_bridge.c::send_motion_stop()` 的航向零目标分支把 `start_motion_command()` 的 bool 结果赋给 int 后再统一判断 `result == 0`，导致该分支成功/失败返回语义反转。续轮只改注释，不越权修复运动行为。
- 并发工作树新增 `chassis_state_task`、`chassis_odometry` 和 MotorBoard 速度快照 API 后，公共头审计从 405 增到 411，并准确报出 6 个新契约缺口；续轮使用动态文件枚举而非固定旧清单。
- 新 odometry 的 `invalidate()` 保留累计 pose/distance/timestamp，仅清 anchor/valid；0 或 >200 ms 间隔路径先更新 timestamp/yaw、保留 anchor，再返回 INVALID。相关测试明确覆盖这一“下一帧可继续积分”的现有语义。
- 生产实现当前权威结果：STM/共享 `checked=353 missing=0`（static-only），S3 `checked=215 missing=0`（全部生产定义），公共头 `checked=411 missing=0`，STM HAL/stdio 外部回调检查无缺口。
- IMU/姿态批次报告但未修复的主要风险：`imu_manager` LSM valid/timestamp 跨锁重读可能不一致；DualAHRS 主时间戳在样本完整接受前推进、`s_dual` 跨任务无锁且 64 位时间可能撕裂、冗余磁场退化检查不一致；BMI323 首次 trace 可能在 CS-low 时阻塞 UART 最长 100 ms。
- BSP/系统批次报告但未修复的主要风险：SPI 微秒忙等无超时；`__io_putchar` 单字符可阻塞 100 ms 且 `_write` 可能静默成功；RTOS `.noinit` 快照存在无锁 TOCTOU；filter mutex 创建失败静默；raw diag 关 IRQ 忙等；log drop 的多任务自增非原子。
- 路径完成性复核：`Drivers/Encoder/encoder.c` 与 `Drivers/Motor/motor.c` 仅 include 对应预留头、没有函数；`Common/Src/system_stm32h7xx_dualcore_boot_cm4_cm7.c` 标明 `MCD Application Team` 与 ST 版权，属于供应商 CMSIS system source；CM4/CM7 其余 `Core/Src` 为 CubeMX/运行库生成边界，按规范不批量改写。

## 2026-08-31 类型续轮发现

- BSD ctags 能枚举 typedef 名称但不枚举 struct/enum 成员；新增 `audit_type_comments.pl` 直接解析 UTF-8 头文件。
- 全量基线为 `types=118 type_missing=94 members=791 member_missing=773`；样本与目视结果一致，主要缺少类型用途、字段单位、枚举状态语义和所有权说明。
- 类型/字段注释必须保持 ABI：不得修改字段顺序/类型、packed/aligned 属性、枚举值、协议大小或宏，只允许增加紧邻中文注释。
- 快速收尾时的最新动态结果为 `types=127 type_missing=20 members=853 member_missing=170`；共享/S3 已为 `59/0`、`348/0`，STM IMU 批次也已落盘，剩余缺口集中在 STM BSP/通信/MotorBoard/System/PID/odometry/chassis 类型。
- 恢复继续后已补齐剩余 STM BSP/通信/MotorBoard/System/PID/odometry/chassis 以及并发新增 observability 类型；最终为 `types=129 type_missing=0 members=869 member_missing=0`。

## 2026-08-31 BLE/雷达并发增量发现

- 三个审计脚本都依赖调用方显式传入当前文件列表；`audit_header_comments.pl` 和 `audit_type_comments.pl` 在无参数时会以 0 退出但实际检查 0 项，`audit_source_comments.pl` 则返回 usage。最终回归必须动态枚举路径并启用 summary，不能把无输出当作零缺口。
- `DOCS/development/S3_STM_CODE_COMMENT_GUIDE.md` 的维护记录仍是并发新增 BLE/雷达代码前的 `411/215` 计数；应在当前审计收敛后更新，避免把旧基线误报为当前全量结果。
- 本轮新增函数仍遵循头文件公共 API 为权威、实现层 static/无头声明框架回调写完整契约的边界；只补注释，不修改 BLE 队列、遥测统计、TCP 重连或雷达重同步行为。
- 新增 host test 动态复核当前 `checked=74 missing=21`；缺口全部位于 `test_s3_ble_log_tx.c`（含 CHECK 宏）和 `test_radar_telemetry_observability.c`，既有雷达测试仍维持完整契约。
- 新增 `RADAR_UPLINK_TELEMETRY_LOG_INTERVAL_MS=2000 ms` 只用于 telemetry 可观测日志节流和重连退避期间的日志唤醒，属于统一调试配置；已保持默认值迁入 `Common/SmartCarDebug/smartcar_debug_config.h`。BLE 队列容量、worker 栈/优先级/分片 delay、TCP 重试和 telemetry 帧龄会改变资源或运行语义，继续留在模块内。
- BLE `partial_drop` 精确语义：断连或关闭 CCC 时，active 已有在途分片或成功前缀会计数；GATT 提交失败时只有 `active_offset > 0`（此前已有成功前缀）才计数，首分片直接提交失败只增加 `send_fail`。
- BLE 固定调度状态对象 host 尺寸为 5456 bytes，worker 栈配置为 3072U；队满覆盖同级最旧帧仍返回入队成功，notification 提交成功不证明手机收到。以上为实现现状，本注释任务未调整资源或传输行为。
- 既有声明审计只识别以 `);` 结束的公共声明，BSD ctags 又未枚举头文件 inline 定义；当前动态范围另有 31 个 `static inline` 函数（1 个 radar age helper、30 个 STM BSP 兼容包装）需要独立纳入，否则 `431/0` 仍不是完整头文件函数覆盖。
- 已增强头文件审计器直接解析 `static inline`：完整注释检查全部字段，`@copydoc` 包装同时要求被引用的权威声明已通过契约审计；当前合计 `checked=462 missing=0`。

## 2026-09-01 多轮可读性/严谨性/准确性/完整性复审

- 用户新目标不是继续堆叠字段，而是进行多轮人工复审；已有 `missing=0` 只能证明机械字段覆盖，不能证明文字易读或语义与实现一致。
- 当前工作树仍大量并发修改；本轮只修注释、审计脚本和本任务文档，不回退或改写现有功能代码。
- 当前权威边界经知识库定位后仍须由源码复核：SRP v4；STM USART2 PA2/PA3 对 S3 UART2 GPIO17/18，默认 921600 8N1；雷达 UART1/GPIO44 独立；轮序 `[RR, RF, LR, LF]`；CM7 保留最终运动准入权。
- 构建、host test 与静态扫描只属于 source/build 证据，不证明 UART/BLE/雷达/传感器/车辆实测。
- 多轮复审标准：可读性检查主语、动作、单位和边界是否一遍可读；严谨性检查绝对化措辞和证据等级；准确性必须对照实现与调用点；完整性同时覆盖声明、static/inline、框架回调、类型成员、宏和测试代码。
- 当前动态机械基线保持通过：头文件函数/回调/inline `462/0`，类型 `129/0`、成员 `869/0`，S3 定义 `254/0`，STM/共享 static `353/0`，host test `95/0`。这些计数仅作为覆盖基线。
- STM 可读性热点按 `>120/>140/>160` 字符注释行计数：`imu_manager.c` 53/17/2、Drivers BMI323 27/23/14、Drivers LSM303 24/17/11、Middleware BMI323 19/17/9、BSP UART 18/11/8、BSP SPI 11/7/7。优先拆分 `>160` 且同时混合返回、阻塞、所有权和验证边界的句子。
- 纯英文 `@brief` 命中主要来自 CubeMX/运行库生成的 CM7 Core 文件，属于明确排除边界；自研范围仅发现少量中英混排术语（如 owner、latest-only、header-only），需按技术准确性判断是否保留或换成中文说明。
- 含糊词扫描命中 `必要时`、`相关`、`正常情况下` 等；只有无法从上下文唯一判断动作/条件的命中才修正，避免把正常中文词汇一律判错。
- 第一轮优先批次通过把阻塞、同步方式、调用上下文和所有权拆成独立句，消除了 50 条超过 160 字符的注释行；未机械重排仍清晰的 120 字符附近语句。
