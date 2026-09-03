# STM32H757 / ESP32-S3 当前工作树深化审计：第 38-47 轮

> 审计日期：2026-08-31（Asia/Shanghai）
> 起始快照：分支 `codex/s3-stm-cn-comments`，HEAD `f703453727a136d15ff7cacea4530beab6e9c08a`，工作树 151 项
> 证据级别：当前脏源码、配置、锁定依赖、Git 元数据和跨端消费者静态审计
> 边界：仅修改审计 Markdown；不读取/输出 secret 原值，未构建、测试、烧录、抓包、连接目标板或运行车辆。

## 1. 审计规则

- 第 38-47 轮只记录新的 UB/复位/持久化/数值/时序/生命周期/对抗预算/可观测性/供应链证据，或对前 37 轮结论的独立关闭/修正。
- 当前源码优先于旧报告；同一 HEAD 下脏文件可能漂移，终稿必须重新锚定行号与非 secret 哈希。
- 每个建议包含位置、原因、最小内容、潜在影响和验证方法，但本轮不实施。
- Critical secret 事件只允许记录路径、数量、相等/命中关系和处置状态，禁止记录值或可逆摘要。

## 2. 第 38 轮：编译器诊断、UB、格式与转换

### `R38-FAULT-ALIAS-001` - Medium - CM7 retained fault record按`uint32_t *`遍历聚合对象违反strict aliasing

- checksum和clear把 `fault_record_t *` 转成 `uint32_t *`逐word访问整个struct。对象8 B对齐、位于`.noinit`且当前字段全为u32，但标准C不允许用成员标量类型lvalue访问聚合对象；代码也没有assert锁定无padding/确切word数。
- Debug是O0、Release是Os且只开Wall，优化版本可能与Debug行为不同。风险集中在唯一上次fault证据的checksum误判/clear不完整，不直接改变motion。
- 建议显式按成员计算/清理，或通过`unsigned char`/本地byte snapshot合法序列化；锁定size/offset并保持checksum字节合同。若算法变化须bump record version。验证O0/Os golden record、逐字段bit flip、partial commit、reset report/clear和strict-alias diagnostics。

### `R38-ROS-NARROW-002` - Medium - ROS参数未在double域验证即窄化，索引转int发生在范围检查前

- bridge把任意double angle/range/frequency直接cast float；positive int64 samples直接转size_t且无上限，随后按samples分配ranges/intensities。
- mapper先计算 `ceil((angle-angle_min)/increment)` 并cast int，之后才检查index bounds。极端finite/Inf参数可使double->float或float->int超可表示范围；巨大samples可抛bad_alloc/length_error终止节点。
- 默认YAML有限且samples=360是保护。建议在double域检查finite、float可表示、angle/range顺序、frequency>0、samples硬上限；bin quotient先检查finite/INT范围再cast。验证NaN/Inf/FLT范围外、反向range、0/1/巨大samples、极小increment，必须明确拒绝且不巨量分配。

### `R38-CPP-HEADER-003` - Low/Medium - Common/SRP公共头使用C专用断言/对齐拼写

- ROS C++17直接include `srp_def.h/srp_registry.h`，而公共头使用 `_Static_assert/_Alignof`；`extern "C"`只改变linkage，不把这些拼写变为标准C++。
- GCC/Clang可作为extension接受，但严格C++17/其他compiler可能warning或失败。建议 `SRP_STATIC_ASSERT/SRP_ALIGNOF` 在C++映射 `static_assert/alignof`、C映射C11拼写，不改wire/RAM layout。验证GCC/Clang C11+C++17 `-pedantic-errors`和golden codec。

### `R38-VENDOR-UB-004` - Low/Medium（当前memory路径不可达）- vendored serial lock对象含边界和未初始化读取

- ROS CMake仍编译YDLidar `lock.c`；其中FHS空filename可读 `p-1`、固定80 B路径用sprintf，UUCP stat失败打印未初始化`name`；LFS分支还有malloc未判空、recv==1024写越界与recv<=0后读未初始化buffer。
- 当前OfficialDecoder只用`parseMemoryChannel`且未定义USE_LOCK_FILE，未确认生产调用这些函数，因此不评High；但UB对象仍进入静态库并妨碍sanitizer/未来直连serial安全。
- 最小方案是memory-only build不编译serial/lock对象；若保留则修复长度/初始化/recv边界并保存vendor patch provenance。验证ASan/UBSan空/长path、recv边界及memory decoder回归。

### `R38-DIAG-005` - Low/Medium - 现有诊断profile不足以关闭优化/语言边界问题

- CM7仅Wall、gnu11、O0/Os分离，无extra/conversion/format=2/cast-align/strict-alias gate；ignored S3 sdkconfig未锁static analyzer/stack check；ROS有Wall/Wextra/Wpedantic但无Werror/conversion/sanitizer；Swift没有完整strict-concurrency setting。
- 建议first-party分层warning矩阵，CM7同时编译O0/Os，host Common/ROS加ASan/UBSan，Swift启用完整concurrency检查；vendor/generated单独baseline。影响是会暴露大量第三方warning，必须分scope收敛。验证固定工具链下0 warning或显式baseline，O0/Os wire/control golden一致。

### 第 38 轮已有保护

- fault record有magic-last commit、DMB/DSB、version/size/checksum与8 B alignment；ROS默认参数合理且addNode已有finite/positive/bounds后置检查。
- SRP公共头断言的wire size/offset意图正确；本轮只修语言可移植性，不改变协议字节。

## 3. 第 39 轮：复位、掉电、watchdog 与安全恢复

### `R39-RESET-REARM-001` - High - peer reboot没有epoch，CM7重启后App heartbeat可未经新授权恢复非零

- S3周期sync request固定 `{4,0,0,0}`，BOOT_INFO只含协议/STM state/request sequence，没有CM7 boot nonce/epoch。
- CM7已同步时收到相同CMD_SYNC_REQ只回复BOOT_INFO并刷新session，不清chassis/heading/MotorBoard目标，因为它无法区分100 ms heartbeat和S3 reboot；S3也无法从BOOT_INFO识别CM7刚重启。
- CM7-only reset不必断开S3 BLE；App wheel heartbeat只依赖BLE connected且保留非零wheelTargets。CM7启动先发zero，但快速重同步后，旧App owner会继续提交同一非零目标，无需操作者重新arm。
- 建议双方增加boot nonce/epoch；任一peer epoch改变时先zero、清pending/inflight/heading/wheel，并要求App新session/显式re-arm。影响是任一芯片reset后控制不会自动续航，App需展示re-arm。验证运动中在send/ACK/heartbeat边界分别reset CM7/S3，未新授权前最后输出始终zero。

### `R39-BOOT-STOP-002` - High - CM7物理zero晚于clock/外设初始化，早期hang无独立停机保证

- `motor_board_force_stop()` 到main中USART6、GPIO、clock等完成后才调用；在此之前HAL_Init/SystemClock和多个外设可失败。
- VOS0 ready使用无deadline busy loop；PLL/clock/USART失败进入关中断永久Error_Handler。若外部MotorBoard保持上次PWM，CM7在本次zero排入USART6前hang，源码没有独立motor enable/brake或板端command watchdog证据。
- 成功启动时USART6已可在scheduler前发送zero是保护，但不覆盖brownout/VOS/PLL/前置外设失败。
- 最小方案优先建立板端command watchdog/硬件enable-brake；在默认安全clock/最早可用GPIO阶段立即assert disable，所有早期错误保持disable。影响硬件/boot合同与调试流程；验证慢降压、VOS/PLL/UART fail、NRST/CPU reset时的物理停机上界。

### `R39-RESET-CAUSE-003` - Medium - 两端不发布reset cause/boot id，棕断与故障恢复无法归因

- CM7启动不读取/清RCC reset status flags，也无项目PVD配置/handler；S3 app_main不读取 `esp_reset_reason()`。fault/RTOS noinit record没有boot id/reset cause。
- 本机ignored S3 sdkconfig的brownout/panic设置不能代表tracked release defaults或已刷设备；无当前eFuse/option/device证据。
- 建议最早快照RCC RSR后再清RMVF，S3记录reset reason，生成boot_id/peer epoch并将watchdog/brownout/fault reset设为安全lockout直到重新arm。验证POR/BOR/NRST/software/watchdog/panic及reset storm的唯一归因。

### `R39-RETAINED-LOG-004` - Medium - retained fault/RTOS记录在确认日志被接受前清除

- boot报告连续调用void `LOG_ERROR()`后立即 `fault_record_clear()/rtos_health_clear()`；日志队列未创建、已满或logger task失败都只有drop counter，没有accepted结果。
- 因此唯一previous-boot证据可在USART/SRP真正可消费前丢失。magic/checksum保证record完整，却不保证报告交付。
- 建议log submit返回accepted并在至少进入可消费持久队列后clear；带boot id/report attempt。若要求掉电保留，使用另行设计的CRC双槽Flash，不能在fault path直接复杂写Flash。验证queue absent/full、logger create fail、reboot loop和报告后clear顺序。

### 第 39 轮边界

- motion目标位于普通BSS，不会由`.noinit`直接跨CM7 reset恢复；风险是外部MotorBoard保持上次输出和仍连接App重新发送，不是RAM旧目标自动复活。
- CM7正常启动zero、WAIT_FOR_HOST、S3/CM7 timeout和BLE真实断连清目标均是现有保护，但缺少peer boot identity和最早硬件stop。

## 4. 第 40 轮：NVS、OTA、Flash 与持久化原子性

### `R40-NVS-ERASE-001` - Medium - 两类NVS初始化错误直接整区擦除，无迁移/恢复标记

- `main_nvs_init()` 对NO_FREE_PAGES或NEW_VERSION_FOUND直接 `nvs_flash_erase()` 后重试一次；项目无namespace schema/version/migration/backup或factory-reset确认。
- 本机ignored sdkconfig启用Wi-Fi NVS与BLE SMP bond NVS，整区擦除至少可能清协议栈持久状态；未读设备NVS，不能声称当前已有bond/配置被清。
- 保护是只对两类错误擦除，erase/retry失败后app_main不继续。建议区分可迁移/损坏/factory reset，记录reset reason并进入re-provision/degraded流程；整区擦除应是显式运维动作。验证耗尽、新版本、损坏页和掉电边界及bond重配。

### `R40-OTA-ROLLBACK-002` - Medium（发布门）- 双OTA分区不等于安全升级/回滚实现

- 活动32 MB表有otadata+ota_0+ota_1，但sdkconfig未启app rollback；活动项目源码没有esp_ota/partition更新owner、mark-valid或版本兼容逻辑，project version也未形成明确合同。
- 当前没有在线OTA入口，因此不是“现有OTA可被利用”；缺口是分区容量容易被误称为已支持安全升级。
- 最小策略先明确OTA unsupported。若启用：只写inactive slot，校验签名/target/schema/单调版本；首启pending，只有通信与安全门健康后mark valid，否则rollback。验证erase/write/otadata/首启各点断电、坏镜像和旧schema。

### `R40-PARTITION-SCHEMA-003` - Low/Medium - 两套互不兼容分区表并存，人工恢复可误选offset

- tracked `partitions-16MiB.csv` 是factory app@0x10000+FAT；活动 `partitions.csv` 是32 MB双OTA app@0x30000，defaults明确选择后者，正常idf build无歧义。
- 旧表未被当前构建引用且文档警告旧offset，这是保护；但手工esptool/恢复若选旧表会使用错误app/data布局。
- 建议旧表加retired/board metadata或归档，release manifest唯一绑定chip/flash size/table hash/app offset，写前硬校验。验证故意传旧表/错flash size必须拒绝。

### `R40-SESSION-ATOMIC-004` - Low/Medium - App session日志无partial/完成标志和原子提交

- writer直接创建最终 `.md`并sync header；每条append不sync，close时才sync/close。没有format/session ID、completion footer、`.partial -> final` atomic rename。
- 正常disconnect异步close，deinit同步等待只保护正常生命周期；crash/掉电/磁盘满可留下截断文件且无法与完整session区分。
- 建议`.partial`、版本化header/session id、受控批量sync、footer+sync+atomic rename；恢复只读完整行并标incomplete。影响是额外fsync成本和临时文件清理；验证create/header/append/footer/rename各阶段kill、磁盘满、只读目录。

### 第 40 轮关闭项

- CM7应用当前无HAL Flash erase/program或文件系统写；S3项目无自研SPIFFS/FAT写；App UserDefaults仅语言/角度且未知值回退。
- MotorBoard Flash命令/磨损属于既有外板合同，本轮不重复；所有掉电/磨损/OTA结论仍无设备验证。

## 5. 第 41 轮：控制与姿态数值鲁棒性

### `R41-MIN-WHEEL-001` - Medium/High - 航向模式最小轮速修正改变平均线速度并绕过线速度命令语义

**确定事实（CONFIRMED_SOURCE）**

- heading controller先将 `linear_mm_s`按400 mm/s² ramp，再用差速运动学得到 `left=v-d/right=v+d`。
- 当 `|linear|>80` 且任一内侧轮低于同方向80 mm/s时，`chassis_apply_min_safe_wheel_speed()`把该轮设为±80，并对另一侧施加相同增量；差值保持，但左右平均值从v变为 `v+deficit`（或负向等价）。
- heading correction上限2 rad/s、track 193 mm，单侧差速量最多193 mm/s；例如低速v略高于80且修正接近上限时，平均目标可逐步接近约273 mm/s。最终tuple仍由1000 mm/s setter上限拒绝保护。

**推断风险与建议**

- 操作者请求的低速巡航可能被算法提升数倍，且 `CHASSIS_MAX_ACCEL_MM_S2`只约束原始v，不约束min-speed平移后的实际平均轮速。修正slew限制每周期变化，但不恢复最终线速度合同。
- 位置：heading min-speed policy/telemetry。原因：防stall策略不能隐式改写车体速度而无上层合同。内容：明确选择“保持平均v并缩小角修正”或“允许提升v但显式限幅/报告applied v”；优先按线速度预算缩小d，不能只事后平移两轮。
- 潜在影响：低速转向能力/克服静摩擦会变化，不能无台架数据删除80 mm/s历史门。验证v=80附近、正负航向误差、前进/后退、角修正0..2 rad/s，比较requested/applied v、w、轮速和物理停转；保持轮序/极性/193 mm不变。

### 第 41 轮已确认数值保护

- kinematics对输入/输出finite和单轮±1000 mm/s做前置拒绝；heading correction有finite、积分、角速度和slew限幅，dt限制1..100 ms，yaw wrap到±180°。
- MotorBoard PID检查finite、目标/积分/总PWM限幅并做抗饱和；本轮不重复固定50 ms dt和feedback watchdog旧发现。

## 6. 第 42 轮：传感器时序、标定来源与质量门

### `R42-CAL-PROVENANCE-001` - Medium - 内部标定结果/质量没有活动端到端结果合同

- 静态窗口内部已统计每流expected/minimum/actual rate/count、valid ratio、accel std、gyro RMS和leveling状态，并只在全部quality gate通过后进入READY。
- Common/SRP当前没有 `IMU_CAL_RESULT` message ID，CM7无发送调用，S3 relay白名单只有CAL_STATUS/IMU_TELEMETRY；只有App保留0x25的14/26 B decoder。因此内部result/sample quality/static stats不会形成端到端标定结果，App bias可持续为`--`。
- 当前内部结果只在本次boot注入DualAHRS、未持久复用是保护；缺口是操作者/验收无法取得bias、sample quality、leveling与来源证据。
- 建议位置：新增versioned calibration-result SRP/S3/App联合合同和evidence log。内容应含boot/cal epoch、config hash、sensor identity/ODR、counts/quality、temperature若可用、leveling status及bias；旧App 0x25只能在明确兼容层使用，不能孤立存在。
- 潜在影响：增加遥测带宽和App model；不得因此把单姿态accel mean误存为bias。验证同一boot重启窗口、ODR变更、motion reject、leveling fallback和设备替换，每份结果可唯一归属且失败结果不标complete。

### `R42-ODOM-TIME-002` - Medium - 新odometry用wheel source dt与当前yaw组合，采样时刻不一致

- 当前工作树新加入构建的 `chassis_state_task` 从MotorBoard取得带timestamp/sequence的wheel snapshot，但姿态通过 `dual_ahrs_get_heading_state()`读取当前yaw/gyro，不返回姿态source timestamp。
- `chassis_odometry_update()`用相邻wheel timestamp计算distance，却用调用时current yaw投影x/y；二者可相差task调度/传输时间，freshness只保证wheel age<=200 ms，没有时间对齐/插值。
- 直行误差小，转弯、任务抖动或接近freshness边界时，distance方向可使用未来yaw。它只影响新里程计/遥测，不直接控制PWM；当前ROS也未发布odom。
- 建议发布带timestamp的原子attitude snapshot，并将yaw插值/nearest到wheel sample时刻，或按同一producer snapshot计算。影响是需短历史buffer与wrap/boot epoch；验证恒定角速度、50/100/200 ms skew、timestamp wrap与stale/reanchor，和ground truth比较。

### 第 42 轮已确认时序/质量保护

- BMI样本以SPI读区间中点的64-bit单调时间戳驱动200 Hz primary；重复/回退时间戳不积分，dt限0.5..20 ms。
- LSM accel 100 Hz、mag 15 Hz分别记录读区间中点；redundant更新要求两个时间戳均前进且差值不超过100 ms，输出LSM freshness限250 ms。
- calibration按独立source timestamp去重、窗口/样本质量/motion/leveling gate；本轮不重复第23轮BMI dynamic recovery与frame/polarity合同问题。

## 7. 第 43 轮：重复初始化、重连与资源生命周期

### `R43-STM-UART-001` - High/Medium - S3 UART RX task创建失败后资源残留使直接重试不可恢复

- `stm_uart_init()` 成功安装UART driver并创建两个mutex后把 `s_initialized=true`，再创建RX task；task创建失败只把initialized置false并返回NO_MEM。
- 失败路径不删除已安装driver、event queue或两个mutex。下一次init会再次 `uart_driver_install()` 同一port，通常返回invalid state；即使内存后来可用，也没有公开deinit/rollback恢复入口。
- app_main当前不重试且只在首次启动调用，这是保护；但错误日志/注释不能支持“修复资源后重试”，只能依赖整机reset，且部分driver仍可能活动。
- 建议位置：STM UART staged init。原因：retryable失败必须逆序释放。内容：driver/mutex/task阶段owner表，task失败删除mutex/driver并清queue/handle；或将该错误明确升级fatal-reset且禁止继续打印SYSTEM READY。
- 潜在影响：cleanup与并发driver callback时序需要锁定；验证逐阶段allocation/task失败后资源计数回到基线，第二次init成功且只有一个RX task/driver。

### `R43-UPLINK-002` - Medium/High - radar uplink中后段失败不回滚network/event资源

- uplink init依次创建default event loop/netif/Wi-Fi、event group、注册Wi-Fi/IP handlers、start Wi-Fi，再分配telemetry queue/register sink/create task。
- netif/event group/handler/Wi-Fi或后段queue/task失败时，多数路径直接return；只对部分telemetry storage释放，没有统一unregister/stop/deinit/destroy。`s_initialized`仍false，所以retry可能叠加handler/netif或遇到invalid state。
- app_main当前只调用一次且Kconfig tracked默认关闭，降低当前默认风险；本机ignored配置已启用，partial resources仍会与后续模块共存并最终打印SYSTEM READY。
- 建议统一staged owner/逆序cleanup，只有task+sink成功后commit initialized；失败发布明确capability=false。影响是Wi-Fi共享event loop/netif ownership需与未来模块协调。验证每个stage故障、重试、并发event到达，handler/task/netif/heap数量守恒。

### `R43-SERVICE-003` - Medium - smartcar service非幂等且task创建后的最后注册没有事务commit

- service init重置全部无锁状态、重建static queue、注册disconnect callback、创建16 KiB task，最后才注册BLE RX callback；没有 `s_task!=NULL`/initialized guard，也没有stop/deinit。
- 当前RX注册实现固定返回OK，所以正常app_main一次调用不触发尾部失败；但重复调用会在旧task运行时改写其link/parser/queue状态并创建第二task或因内存失败留下混合状态。
- 建议用状态机 `UNINIT/INITIALIZING/RUNNING/FAILED`，先完成所有可失败注册，再创建task/commit；重复调用明确返回当前capability，不重置live owner。验证重复/并发init、task create fail和callback register fail，最多一个task且旧motion/session不被无故重置。

### 第 43 轮已确认保护

- radar UART对PSRAM/mutex/driver/task各阶段做逆序cleanup且拒绝ready后的重复init；log/chassis/attitude/MotorBoard/S3 service task start多有handle guard。
- IMU recovery只在boot READY后触发，停止BMI acquisition、重置lifecycle并重建短期init workers；长期BMI task由started guard复用。仍需第23轮所述BMI动态health/recovery完善，但未发现每次recover重复创建BMI task。

## 8. 第 44 轮：跨解析器对抗预算与重同步上界

### `R44-MB-ACK-001` - High/Medium - MotorBoard 通用成功子串没有绑定当前命令

**确定事实（CONFIRMED_SOURCE）**

- `motor_board_protocol.decode_payload()` 在排除 `NACK/NOK/ERROR/FAIL` 后，只要任意文本包含 `OK`、`ACK` 或 `Set ` 就分类为 `MB_FRAME_OK_ACK`。
- 启动状态机的 `MB_WAIT_CONFIG_OK`/`MB_WAIT_FLASH_RESPONSE` 只检查该通用类型，不核对命令 echo、步骤 token 或事务序号；随后直接推进 `mtype -> mline -> mphase -> wdiameter -> read_flash -> read_vol -> upload`。
- `read_flash` 注释声明可能先返回多行配置文本、最后才是 OK，但 parser 在 `s_read_flash_active` 的 raw-line 分支之前先匹配 `Set `；包含该子串的配置行可被当作最终 ACK。

**推断风险与建议**

- 延迟/重复的前一步 ACK、包含成功子串的无关诊断，或 read-flash 配置行可能提前推进新步骤，使参数未确认却接近/进入 RUNNING。UART6 是本地可信链路，但文本无 CRC/事务 ID，不能把“低攻击面”当作相关性证明。
- 位置：MotorBoard parser 与 sequence response contract。原因：成功必须绑定当前命令和阶段。内容：定义每一步可接受的精确响应/echo；read_flash raw lines与终止 ACK分离；旧/重复响应只计数不推进。
- 潜在影响：若 520 板固件返回格式存在版本差异，需要先收集真实响应语料并保留兼容白名单。验证每一步注入旧 ACK、`Set ...`、`NOT OK`、重复/乱序/截断响应，只有匹配当前步骤的最终响应能推进。

### `R44-APP-PARSER-002` - Medium - Swift 控制帧 parser 无容量上限且坏帧逐字节前删

- `SmartCarProtocol.Parser` 对每个通知直接 append 到 `[UInt8]`，没有 `maxBuffer`；无 head 时整批清空，但合法 head + 错 version/tail/CRC/length 会反复 `removeFirst()`，每次移动剩余数组。
- 单个协议 payload 虽限 128 B，但一次 BLE notification/GCD backlog 并不由该 parser 自身限制；连续 `0xAA` 坏候选会形成近 O(n²) copy。第 32 轮已确认 receive DispatchQueue 无 backlog 上限，因此 CPU 与排队内存风险可叠加。
- 建议使用 read index/一次搜索下一 head、固定最大 buffer（按最大帧和少量拼包裕量），达到阈值时记录 drop/reset generation。验证长无效流、每字节 `0xAA`、坏 CRC/tail、半帧跨通知和超速通知，约束 CPU/内存上界。

### 第 44 轮已确认保护与保留缺口

- S3 App C parser 使用固定 `SC_APP_FRAME_MAX_SIZE` 和逐字节状态，无 memmove；SRP parser 固定 512 B、每输入字节常数工作，重复 magic0 可立即保持 seek 状态。
- App `SmartCarLogParser` 有 2048 B 上限、read index 和批量 compact；MotorBoard line/raw/frame 均有 128 B 固定缓冲和 overflow 计数。
- SRP 半帧没有 inter-byte timeout、header 语义校验偏晚，以及 MotorBoard Poll 一次 drain 全 ring 已在前轮记录；本轮不重复编号，仍需统一 parser CPU/deadline fault corpus。

## 9. 第 45 轮：诊断、计数器与状态真实性

### `R45-WHEEL-FRESH-001` - High/Medium - 旧 MotorBoard 轮速可被周期重发并在 App 中伪装成新鲜

**确定事实（CONFIRMED_SOURCE）**

- `motor_board_send_wheel_status()` 每 50 ms 无条件序列化 `s_actual_wheel_speed`；该数组只在收到 MSPD 时更新，wire payload 没有 source timestamp、age、valid/fresh 或 MotorBoard lifecycle 字段。
- MSPD 停流后，MotorBoard task 仍按周期发送最后数组；第 22 轮已确认当前无运行期 MSPD watchdog。
- S3 仅验证四个 float 可解码后转发；App `WheelSpeedState.ingest()` 用 BLE 收到时间写 `lastUpdatedAt` 并追加 history，没有读取 source age。持续转发的旧值因此会在 UI 上保持“刚更新”。

**影响、建议与验证**

- 操作者/诊断可能把反馈停流前的非零/零轮速当成当前测量；ROS2 wheel telemetry也可取得新的 uplink ingress time。它不直接产生 PWM，但会掩盖 feedback failure并污染控制验收证据。
- 位置：MotorBoard feedback snapshot、SRP wheel status schema、S3/App/ROS consumers。原因：transport arrival time不能替代传感器 source freshness。内容：记录最后有效 MSPD monotonic time/sequence；payload 增版本化 `valid/fresh/age`，或停流时停止发布并发显式 stale status。App 显示 source stale，不用 receipt刷新 freshness。
- 潜在影响：wire schema/消费者需要联合版本迁移；冻结现有 0x16 ID 时可先新增并行 versioned status，不能静默改长度。验证运动中停 MSPD、保留 S3/BLE链路，UI/ROS必须在规定上界变 stale且不得追加“新”样本。

### `R45-BLE-COUNT-002` - Medium - 单一 BLE notify failure counter既可能丢增量也不代表交付失败全集

- FFE2/FFE3 都在 `esp_ble_gatts_send_indicate()` 同步返回失败时执行普通 `++s_ble_notify_fail_count`；该 volatile 32-bit counter由 service/log/radar等多任务调用路径共享，read 也无原子/critical protection。并发 read-modify-write可丢增量。
- 未初始化、未 ready/connected/CCC 时函数提前返回 `ESP_ERR_INVALID_STATE` 而不计数；API 返回 OK 后的异步拥塞、disconnect或空中未交付也不进入该 counter。它还不能区分 FFE2 telemetry与FFE3 log。
- 建议单 BLE TX owner分别维护 attempted/accepted/api_failed/congested/disconnected/completed（若协议有 completion）和 epoch；counter 用 owner串行更新或原子操作。影响是诊断结构/日志字段增加；验证并发生产者、invalid state、API fail、congestion和disconnect，计数守恒且不把 accepted称为 delivered。

### `R45-STATUS-003` - Medium - App `ONLINE` 与 SRP 同步事实可相反

- S3 `SC_APP_TYPE_STATUS` 不是周期heartbeat，只在SRP首次同步成功与同步超时时发送；成功payload全零，超时payload写error code。
- App收到任意 STATUS都刷新 `lastStatusAt`，只要BLE connected且状态帧年龄小于3秒就显示ONLINE，未先解释timeout error。结果是健康同步约3秒后可显示STALE，而新到的timeout状态反而短暂显示ONLINE。
- 建议先按error/sync state判定，再使用age；把BLE_CONNECTED、SRP_SYNCED/WAITING/STALE、actuator ready分层。若需要ONLINE heartbeat，定义带epoch和明确周期的新状态，不复用transition帧。验证success、3秒、timeout、持续其他遥测和重连的虚拟时钟矩阵。

### `R45-RADAR-004` - Medium - Radar/TCP `RUNNING/READY` 只代表本地软件阶段

- 周期 `RADAR_STATUS.online` 直接取 `radar_control_is_running()`，头文件明确它不证明物理旋转、数据有效或STM同步。
- `radar_uplink_init()` 创建task后立即记录 `READY TCP uplink`，真正 socket connect成功另有 `TCP CONNECTED`；当前READY不证明peer/ROS consumer存在。
- 建议兼容保留旧bit但UI显示为 `PWM_RUNNING`，版本化增加 frame_fresh/tcp_connected/peer_seen；启动日志改 `UPLINK_TASK_READY`。验证分别断UART、停PWM、断Wi-Fi/TCP、保留TCP但无有效帧，状态只反映对应事实。

### `R45-HIDDEN-COUNT-005` - Medium - 实际递增的S3拒绝/丢弃计数没有可见出口或epoch

- `s_ble_rx_dropped`、`s_ble_rx_protocol_errors`、dual telemetry length/schema/CRC/notify/ble-not-ready等计数实际递增并在service init清零，但没有getter、周期snapshot、boot/connection/SRP epoch或reset reason；现有HWM日志只输出部分parser错误。
- 诊断中的0或缺字段可能是未暴露/刚重置，不是“未发生”。建议只读结构化snapshot携带boot_id、ble_epoch、srp_epoch、reset_reason、valid和各分类计数；用单owner/原子/饱和语义并记录wrap。验证逐类注入只增加目标计数，重连/复位后epoch明确变化。

### `R45-RADAR-SNAPSHOT-006` - Medium - radar lock-drop可少计，snapshot失败伪装成合法全零

- `s_telemetry_lock_drops` 可由service sink和uplink task的pop/stats路径并发普通 `++`，volatile RMW不原子，可能丢增量。
- `radar_uplink_get_telemetry_stats()` 先清零输出；取mutex失败时增加drop后直接返回，没有valid标志，后续日志会把全零queue stats当成真实快照。
- 建议 atomic fetch-add；stats API返回bool或带valid/snapshot_fail/epoch，失败不得发布零值为事实。验证并发mutex contention与已知失败次数，snapshot只能完整有效或显式invalid。

### `R45-LOG-007` - Medium - 日志链没有端到端分阶段丢失预算

- S3 log bridge对非法envelope只即时打印，无累计分类；重编码后忽略FFE3发送结果。pending满可返回NO_MEM，但大量调用点忽略；notify fail getter无消费者。
- CM7 suppression只在下一次允许输出时打印并清零；若后续始终不能发，最终抑制总数不可见。第31轮已记录CM7 log返回值失真，本项补齐S3->FFE3->App末端。
- 建议分别统计 `cm7_enqueue_drop/srp_send_fail/s3_envelope_drop/s3_pending_full/gatt_submit_fail/app_decode_drop` 并带boot/connection epoch；周期输出总量/增量，但不能让同一拥塞日志通道成为唯一证据。验证关闭CCC、pending满、坏envelope和GATT fail的守恒关系。

### 第 45 轮边界

- CM7 UART/MotorBoard transport 主要 stats getter 使用 task critical 复制，radar FIFO/queue stats 有 mutex或单 owner；不能笼统判为所有计数器撕裂。
- 第 20/31 轮已记录多个永远为 0 的 UART字段和“log API OK不等于送达”，本轮只新增 source freshness与共享 BLE counter的具体语义。

## 10. 第 46 轮：依赖、许可证与供应链可追溯性

### `R46-ROS-IMAGE-001` - Medium - ROS2 Docker 输入使用可漂移 tag 和未固定 apt 解析结果

- Dockerfile 基础镜像是 `ros:humble-ros-base-jammy`，没有 immutable digest；`apt-get update/install` 的 build-essential、cmake、gtest、pytest、ROS tools 等均无版本或 snapshot repository。
- 同一 Git commit 在不同日期可得到不同 base layers、Ubuntu/ROS二进制与编译器，导致 host test/bridge行为和产物 hash漂移；公开 tag 被替换或上游仓库受损也扩大供应链面。
- 建议位置：ROS2 Docker/release manifest。原因：发布/验收需要可重放输入。内容：固定 base digest和受控 apt snapshot/version set，定期显式升级；生成 SBOM/provenance并保留镜像 digest。
- 潜在影响：安全更新不再自动进入，必须建立定期升级/漏洞响应；旧 snapshot可能下线。验证同一 commit 在 clean host重复构建，base/package/source/产物清单一致，计划升级有可审查 diff。

### `R46-CM7-TOOLCHAIN-002` - Medium - CM7 toolchain 只固定可执行文件名前缀，不固定版本/来源

- `gcc-arm-none-eabi.cmake` 只要求 PATH 中有 `arm-none-eabi-*`，没有版本范围、路径/归档 hash或容器；preset固定 Debug目录和toolchain文件，但不固定真正编译器。
- flags只有 `-Wall`，没有统一的 `-Wextra/-Wformat=2/-Wconversion`或warnings-as-errors profile；不同 GCC 对 packed/alignment、format、优化和诊断行为可不同。第 18 轮已确认当前本机 GCC 版本不能代表仓库约束。
- 建议发布清单固定官方工具链版本/sha256和获取来源，configure时验证版本，记录 compiler/linker flags与 ELF/map hash。影响是开发机升级需显式迁移；验证两次 clean canonical `CM7/build/Debug` 输入清单一致，并对版本不符硬失败。

### `R46-LICENSE-003` - Medium/Low - 项目级许可证/NOTICE/SBOM 不能覆盖当前发布组合

- tracked license文件只有 CMSIS/HAL 与 vendored YDLidar等 5 个路径；仓库根和 ROS package目录没有项目 LICENSE/NOTICE，虽 `package.xml` 声明项目为 Apache-2.0。
- FreeRTOS V10.3.1大部分源文件内嵌 MIT text，但 `FreeRTOSConfig_template.h` 标为 V10.2.1，说明 vendor subset本身也有版本漂移；仓库没有统一 third-party manifest/SBOM把 FreeRTOS、CMSIS、HAL、YDLidar、ESP-IDF、ROS image与实际产物关联。
- 这不是法律违规结论；静态事实是发布接收方无法仅凭根级材料确定自研代码许可、第三方版本/修改和 notice集合。
- 建议由项目所有者确定自研许可，生成 third-party inventory/SBOM（组件、版本/commit、来源、license、local patch、产物包含关系），发布时携带 NOTICE。验证扫描 source/image/ELF依赖与 manifest一致，由合规负责人审阅。

### `R46-YDLIDAR-004` - Medium - vendored YDLidar裁剪无法从clean checkout的不可变上游重建

- `third_party/README.md` 声明来自official archive的scoped extraction并加入项目 `parseMemoryChannel` 修改；CMake标版本1.0.6，license/header保留。
- inventory没有上游URL、tag/commit、原archive SHA-256、逐文件manifest或独立patch；所引用的本地 `资料/...master.tar.xz` 被根gitignore排除，clean checkout无法取得/校验输入。
- 版本字符串不能证明内容来源。建议记录immutable upstream URL/tag/commit、archive hash、import file manifest和license，把local修改作为独立patch/diff manifest。验证空目录从声明输入重建裁剪，应用patch后逐文件hash一致。

### `R46-GENERATOR-005` - Low/Medium - 生成源码版本可见，但再生成/编译环境未锁定

- IOC记录CubeMX 6.18.0/DB 6.0.180，vendored HAL 1.11.5、FreeRTOS多数源10.3.1；生成源与CMake被跟踪，所以当前snapshot可审计。
- 仓库没有固定CubeMX安装/插件/DB checksum、regeneration diff gate或允许修改区域机器检查；Swift tools 5.9也不锁Swift compiler/macOS SDK。
- 建议release manifest记录CubeMX/DB/GCC/Swift/Xcode精确版本与来源，保存generator input/output manifest，并对regen diff/工具版本不符硬失败。影响是工具升级产生显式大diff；验证同一IOC隔离再生成两次一致并分类现有项目修改。

### 第 46 轮已确认保护

- `ESPS3/dependencies.lock` 固定 IDF `5.5.4`和 target `esp32s3`；Swift package当前无外部 package依赖。
- YDLidar scoped extraction记录版本 `1.0.6`、保留 `LICENSE.txt`和local parser patch说明；CMSIS/HAL有license与源码版本宏，FreeRTOS多数文件自带完整MIT文本。
- 仓库没有 `.gitmodules`/活动子模块，避免构建时隐式抓取未固定 submodule HEAD；仍需对vendored snapshot记录上游commit/hash。

## 11. 第 47 轮：跨域终审与停止门更新

### 11.1 发现统计与根因去重

第38-46轮记录33项详细发现：无新增Critical，2项High，另有5项High/Medium或Medium/High，其他为Medium/Low组合。第34轮公开secret Critical仍未由本审计关闭。

| 根因族 | 本十轮新增证据 | 与前37轮关系 | 终审判断 |
|---|---|---|---|
| 编译/语言边界 | fault strict alias、ROS窄化/float-to-int、C/C++ header、vendor lock UB、warning/sanitizer不足 | 加强R0/T0，不重复旧packing | O0 Debug不能关闭Os/跨语言风险 |
| reset与重新授权 | peer无boot epoch、CM7早期zero之前可hang、无reset cause、retained日志早清 | 新增motion re-arm根因 | reset必须撤销owner并要求显式re-arm |
| 持久化/升级 | NVS整擦、OTA无rollback owner、双分区表、session非原子 | 新增R0数据生命周期 | 当前OTA应明确unsupported |
| 数值/标定 | min-wheel改变applied v、CAL_RESULT无provenance、wheel/yaw odometry时间错位 | 扩展旧dt/标定质量 | requested/applied与cal evidence需分开，里程计需同一时刻snapshot |
| init事务 | STM UART task失败残留、uplink partial network、service非幂等 | 细化旧partial-start | retry必须有staged rollback/commit |
| parser/状态相关性 | MotorBoard通用ACK、Swift parser O(n²)/无上限 | 扩展旧SRP parser预算 | 成功响应必须绑定事务，所有parser有budget |
| 诊断真实性 | wheel旧值伪新、ONLINE反向、Radar READY、本地计数/snapshot/log loss | 扩展旧“accepted不等delivered” | 每个状态/计数必须带stage/freshness/epoch |
| 供应链 | mutable ROS image、PATH-only GCC、license/SBOM、YDLidar provenance、generator环境 | 细化R0复现门 | release输入需immutable且产物可追溯 |

### 11.2 停止门更新

| 门 | 本十轮新增要求 | 未通过时状态 |
|---|---|---|
| S0 secret | 无变化：先轮换公开Wi-Fi/SoftAP secret并扫描refs/artifacts/caches | 不分发含secret refs/产物；history cleanup另行授权 |
| R0 release | 固定GCC/IDF/Swift/CubeMX/ROS digest与依赖；NVS/partition/OTA策略；LICENSE/NOTICE/SBOM；YDLidar可重建；App log完整性 | 当前脏树/镜像只作开发证据，不签release；OTA标unsupported |
| P0 protocol | peer boot epoch；MotorBoard精确ACK/当前step相关；App/SRP parser预算；C/C++公共头严格编译 | 不开放产品motion/动态baud/自动re-arm |
| M0 motion | earliest hardware disable/board watchdog；reset后显式re-arm；min-wheel requested/applied合同；MSPD source freshness | 禁止车辆运动验收，reset/brownout测试只在架空轮 |
| T0 realtime | O0+Os/strict warnings/sanitizers；parser CPU/memory上界；init失败资源守恒；counter/snapshot有效性 | 不声明实时/资源/诊断闭环 |
| N0 ROS2 live | ROS参数double域/size/index硬边界；immutable image；旧wheel source time不可receipt刷新 | 保持 `transport: unconfigured` 或受控replay/PoC |
| D0 CM4 | 既有boot/memory/RTOS/IPC/cache/reset/heartbeat外，再要求跨核boot epoch/reset cause/earliest stop不退化 | 保持CM7-only；CM4首阶段仅no-op+heartbeat |
| V0 vehicle | 匹配镜像完成reset/brownout/early-clock-fail、反馈停流、低速heading requested/applied与不自动rearm验收 | 不宣称READY/车辆验收完成 |

### 11.3 冻结合同与双核结论

- `[M1:RR,M2:RF,M3:LR,M4:LF]`、193.0 mm、WHEEL_TRIM、PWM/encoder符号、SRP/BLE ID和sync/attitude/BUS_OFF/emergency-stop门继续冻结。
- 本十轮没有证据支持提前启用CM4。reset epoch、earliest stop、init rollback和diagnostic provenance在单核尚未闭环，迁移任务只会扩大故障状态空间。
- 推荐仍是CM7-only关闭motion/fault链 -> CM4 no-op+heartbeat/boot epoch -> 低风险日志诊断消费者；sensor/UART/MotorBoard/safety owner留CM7。

### 11.4 证据边界

- 33项均为当前脏工作树/配置/锁定依赖/Git元数据静态证据；无新增secret值读取或输出。
- 审计期间HEAD不变，但并行工作树从151增至至少161项并加入chassis state/odometry；所有带High发现已在13:49当前源码重读，新增模块也纳入第42轮。最终hash/status以后续静止锚点为准。
- 未执行O0/Os build、sanitizer/fuzz、NVS/OTA/掉电/reset/brownout、UART/BLE/TCP、目标板/车辆测试。
- 因此可关闭第38-47轮文档审计，不能关闭任何需要build/device/vehicle的停止门。

## 12. 终稿检查

### 12.1 最终源码锚点

- 复核时间：2026-08-31 14:29:33 CST；HEAD `f703453727a136d15ff7cacea4530beab6e9c08a`；status 161项。
- 并行源码静止后，35个非secret关键路径连续两次SHA-256一致；按下表顺序对完整hash清单再hash得到manifest `1de69c579ecfcfb4dd5d1ba2bf16e4fac9d5d5e1cd9de85589742ab9f8203ce6`。

| 路径 | SHA-256 |
|---|---|
| `STM32H757/CM7/Core/Src/stm32h7xx_it.c` | `9655c318057f2a20c6abf8f7db27fe1281bdb3c51a51ddef3f58ddcfbf98da01` |
| `STM32H757/CM7/Core/Inc/stm32h7xx_it.h` | `bc1a323557fea5c240f62fd1e59e4370975db67b06ea2d6e13d628948be34af5` |
| `ROS2_WIN/src/s3_ydlidar_bridge/src/bridge_node.cpp` | `d09c071d3c0319f3c3e31d1ed2021439487ec63ffb04de2243cd18c8b4c24f43` |
| `ROS2_WIN/src/s3_ydlidar_bridge/src/scan_mapper.cpp` | `ba32b86eea3383aaef813548c7c09ddc95f597347fd8ce462c2e38ff4a5a43dc` |
| `Common/SRP/include/srp_def.h` | `c65fd6468767776258912730c942f75cb0913525163adeeda1c17b5eee80eec2` |
| `Common/SRP/include/srp_registry.h` | `d71c8d02a78fe68575f8e4743bf65734410ff2125225304a4d465a5b0d9bfc93` |
| `Common/SRP/tests/test_srp_codec.c` | `9861cdd58d09625cb25ef7fa03134107c68f338b8120164ca68f3c2dc7bf04ae` |
| `ROS2_WIN/src/s3_ydlidar_bridge/third_party/ydlidar_sdk/core/serial/impl/unix/lock.c` | `9088daab22d18afacd2305e9cd5477d394962bc7727f8aa66982bea3344c7bc0` |
| `STM32H757/CM7/Core/Src/main.c` | `77b132a24738f00b333e5420316993fbe018a3baeb8859081ea204c5e9bb36e6` |
| `STM32H757/Middleware/Communication/Services/s3_service.c` | `b7ec3cf66310160fa7dc7f01e648c700f270c5f1b1914d39fc245d9345223e0c` |
| `ESPS3/components/smartcar_service/command_bridge.c` | `f7fa9b36b3a27462fd8bd377397c60b6b06b055dec5116f31f686a501c582136` |
| `ESPS3/main/main.c` | `fb318ac331224e3a3d91173f3b809cd2ed9a8db1d90827f9ba68ac1ab91eabe1` |
| `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/Support/SessionLogWriter.swift` | `d209bf959ca235681d7bfcebe5b06d8e1e16cc6d04bfd63dbf1c60f266cda8da` |
| `STM32H757/Application/Chassis/chassis_task.c` | `c47821d3f8365f25028de8846fa6a5eac24546883784826e2db393b5d577819a` |
| `STM32H757/Application/Chassis/chassis_state_task.c` | `269fa18869c68f918abeaeb25f531361d42b65ca5a2d5110d929643816c5d143` |
| `STM32H757/Application/Safety/attitude_startup_coordinator.c` | `5a032aa469d3c24460bfacb5c3e77d120aeca74d616d704d4e7934fa0e15836d` |
| `STM32H757/Middleware/Odometry/chassis_odometry.c` | `243fcced25786d5d3b077f7312ece948368439dcda3a2f6715abea730414dec7` |
| `STM32H757/Middleware/Attitude/DualAHRS/dual_ahrs.c` | `14ff5304b2e09d41109cea3da70f2a344ccd9ece2589c47152037137c4d33b0c` |
| `STM32H757/Middleware/Attitude/DualAHRS/dual_ahrs.h` | `8afce18e076cd242cb77651c93442e58e40ac4f7bf613107d4fa98a27325dc6f` |
| `STM32H757/Middleware/Attitude/attitude.c` | `05f85dfeda32e6580a5c677d9cf27a6cdde81cf30327dbaff3d805451b0a1456` |
| `STM32H757/Middleware/Calibration/imu_calibration.c` | `380d7cd7b8ae63007cd525bce18c36ef8c8e598403961ccbd4bcd77a2e40a824` |
| `STM32H757/Middleware/Calibration/imu_calibration.h` | `83ed1a8c4cd7c924d6862a172d42fa75d23e6c76ac6c329e0c84e52fa87a8862` |
| `STM32H757/Middleware/Sensor/imu_manager.c` | `d7899c1f3dea3a811d95b7a7c486944293ed63f4a256de5837f775c53ffe3812` |
| `STM32H757/Application/RTOS/imu_runtime.c` | `3a1023fbfa7f441be85e548f0d509bdffe66234106836321366ea21f7e01ae16` |
| `STM32H757/BSP/TIMER/bsp_timer.c` | `a86404e74e537c45ec792c850772442472a474d5daeb8b9fe54a74db6b7d7555` |
| `STM32H757/Middleware/Sensor/Magnetometer/mag_filter.c` | `b41d014ef7ac7a38e6a855c894458aa475635d207008bb4072d0fe318580a63b` |
| `ESPS3/components/stm_uart/stm_uart.c` | `6a74789695f1037afd14bf4744d60abef439bde0a3dba3513b5b742d40d7500f` |
| `ESPS3/main/radar/radar_uplink.c` | `afa5ae3ea2a3ecffc77a6f84a2659965342dea29fee2d4ff39cd5ac0910e8fb7` |
| `STM32H757/Middleware/MotorBoard/motor_board_protocol.c` | `ef2a458e145c48655749d50ac3af1d9443b39cd32105b7fcad2a311791a12c70` |
| `STM32H757/Middleware/MotorBoard/motor_board_task.c` | `97d96338287c28437b0d20cff32a55e39be39cfd578a0d77af57421dbf5f4b23` |
| `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/Model/SmartCarProtocol.swift` | `8038483415176ccbccf9b22bc109e95aca31cb3de987e1b05da769c6229c8618` |
| `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/Stores/TelemetryStore.swift` | `1e19f3f5add048038a69155de126e34be58a2fd6ad824a78bab3f9e86f0c9f55` |
| `ROS2_WIN/docker/Dockerfile` | `0441801dcbdd78d3970c1a4dd4077d370d56ccc8c56a119c2d0580e22b2f05e5` |
| `STM32H757/gcc-arm-none-eabi.cmake` | `aecb3e2ed49d60dfeacd224e3a001955e0acf6e779071e7965ecde6a59527cce` |
| `ROS2_WIN/src/s3_ydlidar_bridge/third_party/README.md` | `fd70f887f65c9791bb107b1fcb6be29a022e6e5f45fb866417ce12def6a72572` |

### 12.2 检查结果

- [x] 第38-47轮均有独立证据/关闭记录。
- [x] 7项带High发现已在最终工作树重读；secret原值未进入文档。
- [x] 正式计划/报告、findings/progress/task plan与详细证据一致，统计为33项。
- [x] 已知12个secret字面量对七份审计文档命中0；本地链接缺失0。
- [x] 非secret关键源码hash稳定；限定/全树`git diff --check`和audit-only写入边界通过。
- [x] 终稿明确未构建/测试/烧录/抓包/目标板/车辆验收。
