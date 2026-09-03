#ifndef SMARTCAR_SENSOR_BMI323_H
#define SMARTCAR_SENSOR_BMI323_H

#include <stdbool.h>
#include <stdint.h>

#include "bmi323_reg.h"

/*
 * BMI323 传感器驱动接口。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 加速度单位 m/s^2，角速度单位 rad/s；驱动只报告硬件数据和诊断状态，
 * 不负责标定、融合或运动安全判定。
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * BMI323 驱动错误分类；驱动拥有最近错误状态，查询接口仅返回枚举值副本。
 */
typedef enum
{
    BMI323_ERROR_NONE = 0U, /**< 未记录错误或最近操作成功。 */
    BMI323_ERROR_SPI_INIT = 1U, /**< BMI323 SPI/GPIO 端口初始化失败。 */
    BMI323_ERROR_WHO_AM_I_READ = 2U, /**< CHIP_ID 读取流程失败的通用错误。 */
    BMI323_ERROR_WHO_AM_I_MISMATCH = 3U, /**< 兼容的 CHIP_ID 不匹配错误项；当前实现未设置。 */
    BMI323_ERROR_SOFT_RESET = 4U, /**< 软复位命令写入失败。 */
    BMI323_ERROR_POST_RESET_READ = 5U, /**< 软复位后的 CHIP_ID 读取失败。 */
    BMI323_ERROR_ACCEL_CONFIG = 6U, /**< 加速度计配置写入或回读校验失败。 */
    BMI323_ERROR_GYRO_CONFIG = 7U, /**< 陀螺仪配置写入或回读校验失败。 */
    BMI323_ERROR_DATA_READ = 8U, /**< 数据读取参数无效或 SPI 读取失败。 */
    BMI323_ERROR_DATA_WRITE = 9U, /**< 数据写入参数无效的通用错误。 */
    BMI323_ERROR_WHO_AM_I_TIMEOUT = 10U, /**< CHIP_ID SPI 事务超时。 */
    BMI323_ERROR_SPI_TX_FAIL = 11U, /**< SPI 发送事务失败。 */
    BMI323_ERROR_SPI_RX_FAIL = 12U, /**< SPI 全双工读取事务失败。 */
    BMI323_ERROR_WHO_AM_I_VALUE = 13U /**< CHIP_ID 数值不是期望的 0x43。 */
} bmi323_error_t;

/**
 * BMI323 最近诊断状态；用于区分初始化、总线、数据就绪和身份字节异常。
 */
typedef enum
{
    BMI323_DIAG_STATUS_OK = 0U, /**< 最近诊断操作成功。 */
    BMI323_DIAG_STATUS_WHO_AM_I_FAIL, /**< CHIP_ID 探测失败的兼容汇总状态；当前实现未设置。 */
    BMI323_DIAG_STATUS_SPI_READ_FAIL, /**< SPI 读取或端口初始化阶段失败。 */
    BMI323_DIAG_STATUS_CONFIG_FAIL, /**< 复位或传感器配置阶段失败。 */
    BMI323_DIAG_STATUS_DATA_NOT_READY, /**< 所需数据就绪位尚未全部置位。 */
    BMI323_DIAG_STATUS_WHO_AM_I_TIMEOUT, /**< CHIP_ID 读取事务超时。 */
    BMI323_DIAG_STATUS_SPI_TX_FAIL, /**< SPI 写事务发送失败。 */
    BMI323_DIAG_STATUS_SPI_RX_FAIL, /**< SPI 全双工读取事务失败。 */
    BMI323_DIAG_STATUS_WHO_AM_I_VALUE_ERROR, /**< CHIP_ID 完整值与 0x43 不一致。 */
    BMI323_DIAG_STATUS_WHO_AM_I_LOW_BYTE_ERROR, /**< Bosch 帧 RX[2] 的 CHIP_ID 字节异常。 */
    BMI323_DIAG_STATUS_WHO_AM_I_HIGH_BYTE_ERROR /**< 兼容的高字节异常状态；当前实现未设置。 */
} bmi323_diag_status_t;

/* 独立 BMI323 采样任务支持的输出数据率。 */
typedef enum
{
    BMI323_SAMPLE_RATE_100HZ = 100U, /**< 加速度计和陀螺仪 ODR 均为 100 Hz。 */
    BMI323_SAMPLE_RATE_200HZ = 200U, /**< 加速度计和陀螺仪 ODR 均为 200 Hz。 */
    BMI323_SAMPLE_RATE_400HZ = 400U, /**< 加速度计和陀螺仪 ODR 均为 400 Hz。 */
    BMI323_SAMPLE_RATE_800HZ = 800U /**< 加速度计和陀螺仪 ODR 均为 800 Hz。 */
} bmi323_sample_rate_t;

/**
 * BMI323 驱动累计诊断快照；内部状态由驱动拥有，`bmi323_get_diagnostics()` 按值复制。
 */
typedef struct
{
    uint8_t who_am_i; /**< 本次初始化低速原始探针确认的 CHIP_ID。 */
    uint8_t post_reset_who_am_i; /**< 软复位后由正常驱动路径读回的 CHIP_ID。 */
    int32_t init_result; /**< 初始化结果：0 成功，负值为 `-bmi323_error_t`。 */
    uint16_t ctrl_acc; /**< 期望写入 ACC_CONF 的 16 位寄存器值。 */
    uint16_t ctrl_gyr; /**< 期望写入 GYR_CONF 的 16 位寄存器值。 */
    uint16_t acc_conf; /**< 最近读回并通过校验的 ACC_CONF 值。 */
    uint16_t gyr_conf; /**< 最近读回并通过校验的 GYR_CONF 值。 */
    uint32_t spi_error_count; /**< 累计 SPI 事务错误数，达到最大值后饱和。 */
    uint32_t spi_error; /**< 与 `spi_error_count` 同步维护的兼容错误计数。 */
    uint32_t spi_read_count; /**< 累计提交的寄存器读事务数。 */
    uint32_t spi_read_success; /**< 累计成功的寄存器读事务数。 */
    uint32_t spi_read_fail; /**< 累计失败的寄存器读事务数。 */
    uint32_t spi_tx_fail; /**< 累计 SPI 写发送失败数。 */
    uint32_t spi_rx_fail; /**< 累计 SPI 全双工读取失败数。 */
    uint32_t write_count; /**< 累计提交的寄存器写事务数。 */
    uint32_t write_ok; /**< 累计成功的寄存器写事务数。 */
    uint32_t write_fail; /**< 累计失败的寄存器写事务数。 */
    uint32_t whoami_fail; /**< 累计 CHIP_ID 探测、读取或数值校验失败数。 */
    uint32_t read_ok; /**< 累计成功的物理量或原始样本读取调用数。 */
    uint32_t read_fail; /**< 累计无数据或失败的物理量/原始样本读取调用数。 */
    int16_t accel_raw_x; /**< 最近成功样本的 X 轴加速度原始寄存器值。 */
    int16_t accel_raw_y; /**< 最近成功样本的 Y 轴加速度原始寄存器值。 */
    int16_t accel_raw_z; /**< 最近成功样本的 Z 轴加速度原始寄存器值。 */
    int16_t gyro_raw_x; /**< 最近成功样本的 X 轴角速度原始寄存器值。 */
    int16_t gyro_raw_y; /**< 最近成功样本的 Y 轴角速度原始寄存器值。 */
    int16_t gyro_raw_z; /**< 最近成功样本的 Z 轴角速度原始寄存器值。 */
    uint8_t last_whoami; /**< 最近 CHIP_ID 事务 RX[2] 中解析出的值。 */
    uint8_t last_rx0; /**< 最近 CHIP_ID SPI 帧的 RX[0] 原始字节。 */
    uint8_t last_rx1; /**< 最近 CHIP_ID SPI 帧的 RX[1] 原始字节。 */
    uint8_t last_rx2; /**< 最近 CHIP_ID SPI 帧的 RX[2] 原始字节。 */
    uint8_t last_rx3; /**< 最近 CHIP_ID SPI 帧的 RX[3] 原始字节。 */
    uint16_t sample_rate_hz; /**< 当前缓存的加速度计/陀螺仪 ODR，单位 Hz。 */
    bmi323_error_t last_error; /**< 最近驱动错误分类；成功操作不一定自动清除旧错误。 */
    bmi323_diag_status_t last_status; /**< 最近诊断操作的细分状态。 */
} bmi323_diagnostics_t;

/**
 * 当前初始化尝试的一次性低速 CHIP_ID 原始探针；驱动内部锁存，getter 按值复制。
 */
typedef struct
{
    uint8_t valid; /**< RX[2] 等于 CHIP_ID 0x43 时为 1，否则为 0。 */
    uint8_t whoami; /**< 从原始帧 RX[2] 解析的 CHIP_ID 值。 */
    uint8_t rx0; /**< 首次 CHIP_ID 原始事务的 RX[0]。 */
    uint8_t rx1; /**< 首次 CHIP_ID 原始事务的 RX[1]。 */
    uint8_t rx2; /**< 首次 CHIP_ID 原始事务的 RX[2]。 */
    uint8_t rx3; /**< 首次 CHIP_ID 原始事务的 RX[3]。 */
    uint8_t spi_status; /**< 底层 HAL 状态低 8 位；负值无法表示时写 `UINT8_MAX`。 */
} bmi323_diag_t;

/**
 * @brief 初始化 BMI323 端口，完成低速探针、软复位、身份与 ODR 配置校验。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return true 表示完整初始化成功；false 表示端口、SPI、身份或配置失败，
 *         失败后 `bmi323_is_online()`/`bmi323_is_ready()` 返回 0。
 * @note 调用方式与线程约束：由 IMU 初始化 worker 单次调用；内部包含阻塞 SPI、忙等待延时
 *       和有限日志，非线程安全，禁止与采样并发或从 ISR 调用。
 */
bool bmi323_init(void);
/**
 * @brief 执行并缓存一次三种事务形式的低速 WHO_AM_I 硬件探针。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return true 表示 Bosch 回调形式读取到 CHIP_ID 0x43；false 表示端口、SPI
 *         或身份校验失败；同一初始化周期的后续调用返回缓存结果。
 * @note 调用方式与线程约束：仅由 `bmi323_init()` 的启动诊断路径调用；会阻塞 SPI 并输出
 *       原始诊断，不能作为周期在线检测，禁止从 ISR 调用。
 */
bool bmi323_spi_probe(void);
/**
 * @brief 通过 BMI323 SPI 读协议读取连续寄存器字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] reg 起始寄存器地址，函数内部添加读标志。
 * @param[out] data 调用方提供的输出缓冲区，容量至少为 `len` 字节，不允许为 NULL。
 * @param[in] len 读取字节数，范围为 1..驱动内部最大连续读取长度。
 * @return true 表示 `data[0..len-1]` 有效；false 表示参数或 SPI 事务失败，
 *         同时更新驱动错误/诊断计数，失败时不得使用输出。
 * @note 调用方式与线程约束：仅传感器 owner 在任务上下文调用；函数使用栈缓冲并阻塞 SPI，
 *       输入/输出所有权不转移，禁止并发访问同一设备或从 ISR 调用。
 */
bool bmi323_read_reg(uint8_t reg, uint8_t *data, uint16_t len);
/**
 * @brief 通过 BMI323 SPI 写协议写入连续寄存器字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] reg 起始寄存器地址，函数内部清除读标志。
 * @param[in] data 调用方拥有的只读输入缓冲区，不允许为 NULL。
 * @param[in] len 写入字节数，范围为 1..驱动内部最大连续写入长度。
 * @return true 表示阻塞 SPI 事务成功；false 表示参数或发送失败，并更新诊断状态。
 * @note 调用方式与线程约束：仅初始化/配置 owner 在任务上下文调用；函数在返回前完成数据复制
 *       和总线事务，不保留输入指针，禁止并发访问同一设备或从 ISR 调用。
 */
bool bmi323_write_reg(uint8_t reg, const uint8_t *data, uint16_t len);
/**
 * @brief 设置 BMI323 加速度计和陀螺仪共用的输出数据率配置。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] sample_rate 仅允许 `bmi323_sample_rate_t` 定义的 100/200/400/800 Hz。
 * @return true 表示配置被接受；驱动未就绪时仅缓存值，驱动已就绪时还表示寄存器
 *         写入与回读校验成功；false 时保留先前配置。
 * @note 调用方式与线程约束：由 IMU manager 在停止/重配采样的任务上下文调用；已就绪路径
 *       会阻塞访问 SPI，非线程安全，禁止与采样并发或从 ISR 调用。
 */
bool bmi323_set_sample_rate(bmi323_sample_rate_t sample_rate);
/**
 * @brief 读取当前缓存的 BMI323 输出数据率配置。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 当前 `bmi323_sample_rate_t` 配置；不证明寄存器当前可通信或有新样本。
 * @note 调用方式与线程约束：任务上下文只读查询；不访问 SPI、不阻塞，但与重配置并发时
 *       调用方不得把该值当作原子事务快照。
 */
bmi323_sample_rate_t bmi323_get_sample_rate(void);
/**
 * @brief 阻塞读取并换算 BMI323 三轴加速度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] x X 轴加速度输出，单位 m/s^2，不允许为 NULL。
 * @param[out] y Y 轴加速度输出，单位 m/s^2，不允许为 NULL。
 * @param[out] z Z 轴加速度输出，单位 m/s^2，不允许为 NULL。
 * @return true 表示三个输出同时有效；false 表示参数、未就绪或 SPI 失败，
 *         失败时输出不得作为新样本使用。
 * @note 调用方式与线程约束：由单一 BMI323 采样任务调用；函数阻塞访问 SPI，输出归调用方，
 *       禁止并发调用或从 ISR 调用。
 */
bool bmi323_read_accel(float *x, float *y, float *z);
/**
 * @brief 阻塞读取并换算 BMI323 三轴角速度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] x X 轴角速度输出，单位 rad/s，不允许为 NULL。
 * @param[out] y Y 轴角速度输出，单位 rad/s，不允许为 NULL。
 * @param[out] z Z 轴角速度输出，单位 rad/s，不允许为 NULL。
 * @return true 表示三个输出同时有效；false 表示参数、未就绪或 SPI 失败，
 *         失败时输出不得作为新样本使用。
 * @note 调用方式与线程约束：由单一 BMI323 采样任务调用；函数阻塞访问 SPI，输出归调用方，
 *       禁止并发调用或从 ISR 调用。
 */
bool bmi323_read_gyro(float *x, float *y, float *z);
/**
 * @brief 检查双路 DATA_READY 后一次性读取原始加速度和陀螺仪计数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] accel 调用方提供的 3 元素 `int16_t` 数组，保存 XYZ 原始计数。
 * @param[out] gyro 调用方提供的 3 元素 `int16_t` 数组，保存 XYZ 原始计数。
 * @return true 表示两组数组来自同一次有效数据突发；false 表示参数、未就绪、
 *         DATA_READY 未同时置位或 SPI 失败，失败时不得消费输出。
 * @note 调用方式与线程约束：BMI323 独立采样任务按配置 ODR 调用；包含状态和数据两次阻塞
 *       SPI 事务，数组所有权不转移，禁止并发调用或从 ISR 调用。
 */
bool bmi323_read_raw_sample(int16_t accel[3], int16_t gyro[3]);
/**
 * @brief 按当前固定加速度量程把单轴原始计数换算为 m/s^2。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] raw 单轴有符号 16 位原始计数。
 * @return 对应加速度，单位 m/s^2。
 * @note 调用方式与线程约束：任务或主机回放均可调用；纯计算、无状态、不阻塞且可重入。
 */
float bmi323_accel_raw_to_mps2(int16_t raw);
/**
 * @brief 按当前固定陀螺量程把单轴原始计数换算为 rad/s。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] raw 单轴有符号 16 位原始计数。
 * @return 对应角速度，单位 rad/s。
 * @note 调用方式与线程约束：任务或主机回放均可调用；纯计算、无状态、不阻塞且可重入。
 */
float bmi323_gyro_raw_to_rads(int16_t raw);
/**
 * @brief 阻塞读取 BMI323 温度寄存器并换算为摄氏度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] temperature 调用方拥有的温度输出，单位 degC，不允许为 NULL。
 * @return true 表示输出有效；false 表示参数、未就绪或 SPI 失败，失败时输出失效。
 * @note 调用方式与线程约束：初始化后由诊断/采样任务按需调用；阻塞访问 SPI，禁止并发
 *       访问同一设备或从 ISR 调用。
 */
bool bmi323_read_temperature(float *temperature);
/**
 * @brief 读取最近一次 BMI323 驱动错误枚举。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 最近记录的 `bmi323_error_t`；读取不清零，后续操作可覆盖该值。
 * @note 调用方式与线程约束：诊断任务低频读取；不阻塞、不访问硬件，无锁读取不构成
 *       与采样操作一致的原子快照。
 */
bmi323_error_t bmi323_get_last_error(void);
/**
 * @brief 读取最近一次 BMI323 数据/配置诊断状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 最近记录的 `bmi323_diag_status_t`；DATA_NOT_READY 与总线失败需区分处理。
 * @note 调用方式与线程约束：采样失败后或诊断任务中读取；不阻塞、不访问硬件，值可能被
 *       后续采样覆盖。
 */
bmi323_diag_status_t bmi323_get_status(void);
/**
 * @brief 查询 BMI323 初始化就绪标志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 1 表示初始化成功标志仍有效，0 表示未就绪；当前实现不主动测量样本时效。
 * @note 调用方式与线程约束：任务上下文只读查询；不阻塞、不访问 SPI，不能替代
 *       `bmi323_read_raw_sample()` 的成功结果。
 */
uint8_t bmi323_is_online(void);
/**
 * @brief 查询 BMI323 是否完成初始化和配置。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 与 `bmi323_is_online()` 相同的 1/0 就绪标志。
 * @note 调用方式与线程约束：兼容只读入口；不阻塞、不访问 SPI，不证明有新数据。
 */
uint8_t bmi323_is_ready(void);
/**
 * @brief 读取 STATUS 寄存器并刷新双路 DATA_READY 诊断状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 无返回值；未就绪时不操作，SPI 失败或数据未就绪写入对应诊断状态。
 * @note 调用方式与线程约束：仅诊断任务在不与采样并发时调用；函数阻塞访问 SPI，
 *       不读取传感器样本，禁止从 ISR 调用。
 */
void bmi323_refresh_data_ready_status(void);
/**
 * @brief 复制完整 BMI323 诊断计数、配置和最近原始值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] diagnostics 调用方拥有的输出对象；NULL 时函数不执行复制。
 * @return 无返回值。
 * @note 调用方式与线程约束：诊断任务低频读取；不阻塞、不访问硬件。当前实现无锁复制，
 *       与采样并发时字段可能来自相邻操作，不得当作事务级一致快照。
 */
void bmi323_get_diagnostics(bmi323_diagnostics_t *diagnostics);
/**
 * @brief 复制当前初始化周期的一次性 WHO_AM_I 原始探针结果。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] diag 调用方拥有的输出对象；NULL 时函数不执行复制。
 * @return 无返回值；`diag->valid` 为 0 时其原始字节不能证明身份校验成功。
 * @note 调用方式与线程约束：初始化/诊断完成后读取；不阻塞、不访问 SPI，下一次初始化会
 *       重置并覆盖内部探针结果。
 */
void bmi323_get_diag(bmi323_diag_t *diag);
/**
 * @brief 将 BMI323 诊断枚举转换为静态英文日志字符串。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] status 待格式化的 `bmi323_diag_status_t`。
 * @return 指向只读静态字符串的指针；调用方不得修改或释放，未知值按当前实现
 *         返回 `"OK"`。
 * @note 调用方式与线程约束：日志格式化路径可直接调用；纯查询、不阻塞且可重入。
 */
const char *bmi323_diag_status_name(bmi323_diag_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* SMARTCAR_SENSOR_BMI323_H */
