# BMI323 诊断状态桥接报告

## 范围

本次只修改了 BMI323 诊断缓存/API 和 IMU Manager 的 BMI323 周期 DEBUG 输出：

- `STM32H757/Middleware/Sensor/BMI323/bmi323.c`
- `STM32H757/Middleware/Sensor/BMI323/bmi323.h`
- `STM32H757/Middleware/Sensor/imu_manager.c`

未修改 SRPv4 UART codec/registry、SPI port、WHO_AM_I 判断、初始化流程、S3、BLE、App 或其他传感器模块；未烧录。

## 实现

### 诊断缓存

新增公开类型 `bmi323_diag_t`：

```c
uint8_t whoami;
uint8_t rx0;
uint8_t rx1;
uint8_t spi_status;
```

`bmi323_capture_whoami_raw()` 在首次寄存器 `0x00` SPI 事务完成后缓存：

- `whoami` / `rx1`：BMI323 返回寄存器字节；
- `rx0`：SPI 首个 dummy 接收字节；
- `spi_status`：首次事务的 HAL 状态低 8 位；状态不可用时为 `0xFF`。

新增只读拷贝接口：

```c
void bmi323_get_diag(bmi323_diag_t *diag);
```

原有 `whoami_trace_done` 一次性机制保留，缓存不会触发新的 SPI 事务，也不改变 WHO_AM_I 判断。

### 周期 DEBUG

`imu_bmi323_debug_log()` 保留原记录：

```text
[BMI323][DEBUG]
read_ok=...
read_fail=...
last_status=...
```

并在同一个既有健康周期追加一条低频 raw 诊断记录：

```text
[BMI323][DEBUG]
whoami=0xXX
rx0=0xXX
rx1=0xXX
spi_status=0xXX
```

没有新增定时器、任务或高频采样；只是每次原有 BMI323 健康日志周期增加一条 LOG 记录。两条记录均通过现有 `LOG_INFO`/SRPv4 LOG 路径发送。

## 长度检查

按 32 位计数器最大值和最长当前状态名计算：

| 记录 | 最坏长度（含 CRLF） |
| --- | ---: |
| 原有计数/状态 DEBUG | 93 bytes |
| 新增 raw DEBUG | 67 bytes |

两条记录都小于 `SMARTCAR_LOG_MAX_PAYLOAD=96`，不会触发 STM32 USART2 文本截断或 S3 长度拒绝。

## 构建验证

配置命令（在 `STM32H757/CM7`）：

```sh
cmake --preset Debug
```

结果：配置和生成成功，输出目录为 `STM32H757/CM7/build/Debug`。

构建命令（在仓库根目录）：

```sh
cmake --build STM32H757/CM7/build/Debug --parallel 2
```

结果：通过，BMI323、IMU Manager 重新编译并完成链接：

```text
[1/3] bmi323.c.obj
[2/3] imu_manager.c.obj
[3/3] Smart_Car_H757_CM7.elf
FLASH: 98596 B / 1 MB (9.40%)
RAM:   40080 B / 128 KB (30.58%)
```

ELF 中可见新增格式字符串：

```text
whoami=0x%02X
rx0=0x%02X
rx1=0x%02X
spi_status=0x%02X
```

## 结果与边界

下一次 BLE 运行日志应能在周期 `[BMI323][DEBUG]` 中看到首次 WHOAMI 原始返回值，即使启动阶段的 `[BMI323][WHOAMI]`/`[BMI323][SPI]` 已因时序丢失。构建通过只证明源码到 ELF 的编译链接，不证明 BMI323 电气响应、USART2、S3 或 BLE 运行链路；本轮未烧录、未做硬件验证。
