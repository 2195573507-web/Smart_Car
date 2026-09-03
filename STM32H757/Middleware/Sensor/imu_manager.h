#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <stdint.h>

#include "bsp_status.h"
#include "BMI323/bmi323.h"
#include "imu_calibration.h"
#include "imu_leveling.h"

/*
 * 双 IMU 采集管理器。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 负责传感器驱动、采样快照和初始化状态，不负责最终姿态/电机安全决策。
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * BMI323 最新物理量发布值；manager 内部拥有缓存，读取接口按值复制给调用方。
 */
typedef struct
{
    float accel_x; /**< BMI323 传感器坐标系 X 轴加速度，单位 m/s^2。 */
    float accel_y; /**< BMI323 传感器坐标系 Y 轴加速度，单位 m/s^2。 */
    float accel_z; /**< BMI323 传感器坐标系 Z 轴加速度，单位 m/s^2。 */
    float gyro_x; /**< BMI323 传感器坐标系 X 轴角速度，单位 rad/s。 */
    float gyro_y; /**< BMI323 传感器坐标系 Y 轴角速度，单位 rad/s。 */
    float gyro_z; /**< BMI323 传感器坐标系 Z 轴角速度，单位 rad/s。 */
    uint32_t timestamp; /**< 捕获单调时间的低 32 位毫秒表示，允许自然回绕。 */
    uint64_t timestamp_us; /**< SPI 读取起止中点的单调捕获时间，单位 us。 */
    uint32_t sample_count; /**< 驱动成功入队样本累计数的最近快照。 */
    uint32_t invalid_count; /**< 真实读取失败或 manager 周期无待处理样本的累计次数。 */
    uint8_t valid; /**< 本次发布是否来自有效原始样本；当前与两路分项有效位一致。 */
    uint8_t accel_valid; /**< 本次发布的加速度分量有效时为 1。 */
    uint8_t gyro_valid; /**< 本次发布的角速度分量有效时为 1。 */
} bmi323_data_t;

#define BMI_RING_BUFFER_SIZE UINT16_C(512)

/**
 * BMI323 高频采样任务写入环形缓冲的原始样本；缓冲区按值持有，不保留外部指针。
 */
typedef struct
{
    uint64_t timestamp_us; /**< SPI 读取起止中点的单调捕获时间，单位 us。 */
    int16_t accel[3]; /**< X/Y/Z 三轴加速度原始寄存器值。 */
    int16_t gyro[3]; /**< X/Y/Z 三轴角速度原始寄存器值。 */
    uint8_t valid; /**< 两组原始寄存器均成功读取时为 1。 */
} bmi323_raw_sample_t;

/**
 * BMI323 单生产者/单消费者环形缓冲状态；仅由 IMU manager 在 BMI 数据锁下拥有和修改。
 */
typedef struct
{
    bmi323_raw_sample_t buffer[BMI_RING_BUFFER_SIZE]; /**< 按值存放的固定容量原始样本数组。 */
    uint16_t head; /**< 下一次生产者写入索引，范围 0..SIZE-1。 */
    uint16_t tail; /**< 当前最旧待处理样本索引，范围 0..SIZE-1。 */
    uint16_t count; /**< 当前待处理样本数，范围 0..SIZE。 */
    uint32_t overflow_count; /**< 缓冲满时覆盖最旧样本的累计次数，达到最大值后饱和。 */
    uint64_t last_capture_us; /**< 最近成功入队样本的单调捕获时间，单位 us。 */
} bmi323_ring_buffer_t;

/**
 * BMI323 高频捕获链路统计快照；内部计数由 manager 拥有，查询接口按值复制。
 */
typedef struct
{
    uint32_t sample_count; /**< 成功入队样本累计数，达到最大值后饱和。 */
    uint32_t overflow_count; /**< 环形缓冲满后覆盖最旧样本的累计次数。 */
    uint32_t last_timestamp; /**< 最近成功入队捕获时间的低 32 位毫秒表示。 */
    uint64_t first_timestamp_us; /**< 当前统计窗口首个成功样本的单调时间，单位 us。 */
    uint64_t last_timestamp_us; /**< 当前统计窗口最近成功样本的单调时间，单位 us。 */
    uint32_t max_latency_us; /**< 捕获至 10 ms manager 消费发布的历史最大延迟，单位 us。 */
    uint32_t read_fail_count; /**< 除 DATA_NOT_READY 外的真实传感器读取失败累计数。 */
    uint32_t contention_drop_count; /**< 高频任务零等待获取驱动锁或数据锁失败的累计次数。 */
    uint16_t pending_count; /**< 查询时环形缓冲内的待处理样本数。 */
    uint16_t configured_rate_hz; /**< BMI323 当前配置输出数据率，单位 Hz。 */
    uint16_t measured_rate_hz; /**< 依据首末时间和样本间隔计算的实测采样率，单位 Hz。 */
} bmi323_capture_stat_t;

/**
 * LSM303 最近加速度快照；manager 内部缓存按值复制，数值已经转换到车体坐标系。
 */
typedef struct
{
    float ax; /**< 车体坐标系 X 轴加速度，单位 m/s^2。 */
    float ay; /**< 车体坐标系 Y 轴加速度，单位 m/s^2。 */
    float az; /**< 车体坐标系 Z 轴加速度，单位 m/s^2。 */
    uint32_t timestamp; /**< 最近成功加速度采样的单调时间，单位 ms，允许回绕。 */
    uint64_t timestamp_us; /**< 最近成功加速度采样的单调时间，单位 us。 */
} lsm_accel_data_t;

/**
 * LSM303 最近磁场快照；manager 内部缓存按值复制，数值已经转换到车体坐标系。
 */
typedef struct
{
    float mx; /**< 车体坐标系 X 轴磁场强度，单位 uT。 */
    float my; /**< 车体坐标系 Y 轴磁场强度，单位 uT。 */
    float mz; /**< 车体坐标系 Z 轴磁场强度，单位 uT。 */
    uint32_t timestamp; /**< 最近成功磁场采样的单调时间，单位 ms，允许回绕。 */
    uint64_t timestamp_us; /**< 最近成功磁场采样的单调时间，单位 us。 */
} lsm_mag_data_t;

/**
 * 单路传感器 manager 运行统计；内部状态受对应数据锁保护，getter 按值复制。
 */
typedef struct
{
    uint32_t update_count; /**< 成功产生新数据的累计次数；BMI 按入队，LSM 按任一分路更新计数。 */
    uint32_t read_calls; /**< 已纳入统计的读取次数；BMI 计成功/真实失败，LSM 每轮联合读取计 1 次。 */
    uint32_t last_update_ms; /**< 最近成功更新的单调时间，单位 ms，允许回绕。 */
    bsp_status_t last_status; /**< 最近初始化、读取或发布操作的 BSP 状态。 */
} imu_sensor_stats_t;

/**
 * LSM303/BMI323 并行初始化 worker 的进度快照；manager 拥有状态并在互斥锁下按值复制。
 */
typedef struct
{
    uint8_t lsm_started; /**< LSM303 worker 启动请求已登记时为 1，不证明任务创建成功。 */
    uint8_t bmi_started; /**< BMI323 worker 启动请求已登记时为 1，不证明任务创建成功。 */
    uint8_t lsm_complete; /**< LSM303 worker 已结束或任务创建失败已收口时为 1。 */
    uint8_t bmi_complete; /**< BMI323 worker 已结束或任务创建失败已收口时为 1。 */
    uint8_t lsm_success; /**< LSM303 驱动初始化返回 OK 时为 1。 */
    uint8_t bmi_success; /**< BMI323 驱动初始化返回 true 时为 1。 */
    uint32_t lsm_start_time; /**< 本轮 LSM303 worker 创建尝试开始的单调时间，单位 ms。 */
    uint32_t bmi_start_time; /**< 本轮 BMI323 worker 创建尝试开始的单调时间，单位 ms。 */
    uint32_t lsm_end_time; /**< LSM303 worker 完成或创建失败收口时间，单位 ms。 */
    uint32_t bmi_end_time; /**< BMI323 worker 完成或创建失败收口时间，单位 ms。 */
} imu_dual_init_status_t;

/**
 * @brief 初始化 IMU 单调时基、数据锁和双 IMU 生命周期初始状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return `BSP_STATUS_OK` 表示生命周期资源准备成功；时基或锁创建失败返回错误。
 * @note 调用方式与线程约束：由 `imu_runtime_start()` 单次调用；正常镜像不在此直接初始化
 *       两个硬件驱动，后续由 boot manager 释放双 worker。可能分配 RTOS 对象并
 *       清空全局状态，禁止重复并发调用或从 ISR 调用。
 */
bsp_status_t imu_init(void);
/**
 * @brief 复位 IMU 数据、水平校准和双初始化生命周期，准备重新初始化。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return `BSP_STATUS_OK` 表示恢复状态已准备；时基/锁失败返回错误。
 * @note 调用方式与线程约束：仅 IMU 任务按既有恢复周期调用；正常镜像不会在返回前完成硬件
 *       re-init，ready 会保持关闭直至 boot manager 重新完成流程。函数可能阻塞
 *       多个 mutex，禁止从 ISR 或与初始化 worker 并发调用。
 */
bsp_status_t imu_recover(void);
/**
 * @brief 执行一次 LSM303 采样、BMI 最新样本消费和统一快照发布。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 未准备时返回 `BSP_STATUS_NOT_READY`；初始化 worker 尚在运行时正常镜像
 *         返回 OK；已初始化时返回本次 LSM303 更新状态，NOT_READY 可表示无新数据。
 * @note 调用方式与线程约束：仅 `imu_task` 单 writer 以 10 ms 周期调用；包含阻塞 I2C、多个
 *       mutex 和融合/标定调用，禁止重入或从 ISR 调用。
 */
bsp_status_t imu_update(void);

/* 启动管理器在 DUAL_IMU_BOOT/INIT 同时启动两条独立总线 worker，完成后作为一个生命周期屏障。 */
/**
 * @brief 创建并同时释放 LSM303 与 BMI323 两条独立总线初始化 worker。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return OK 表示 worker 已创建/已成功完成；NOT_READY 表示尚未准备或既有 worker
 *         未成功完成；任务创建失败返回 ERROR，诊断镜像/无 RTOS 返回 UNSUPPORTED。
 * @note 调用方式与线程约束：仅 boot manager INIT 阶段调用；会创建/删除任务、短暂挂起调度并
 *       清空采样状态，可能阻塞，禁止重复并发调用或从 ISR 调用。
 */
bsp_status_t imu_manager_start_dual_initialization(void);
/**
 * @brief 复制双初始化 worker 的启动、完成、成功和时间戳状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] status 调用方拥有的输出对象；NULL 时不执行复制。
 * @return 无返回值。
 * @note 调用方式与线程约束：boot manager 任务轮询；获取可阻塞 dual-init mutex，不保存
 *       输出指针，禁止从 ISR 调用。
 */
void imu_manager_get_dual_initialization_status(imu_dual_init_status_t *status);
/**
 * @brief 在两条 worker 成功后固定 BMI323 为 200 Hz、开放采集并创建采样任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 1 表示双初始化屏障和 BMI 采样任务均提交成功；任一 worker、ODR 配置
 *         或任务创建失败返回 0，并保持 manager 未初始化/采集关闭。
 * @note 调用方式与线程约束：仅 boot manager 在两个 worker 完成后调用一次；会阻塞 mutex、
 *       访问 SPI 并创建任务，禁止从 ISR 或与采样并发调用。
 */
uint8_t imu_manager_finalize_dual_initialization(void);

/* 静态窗口关闭时只提交一次水平状态；运行路径只读取已冻结矩阵。 */
/**
 * @brief 将 BMI323/LSM303 水平状态重置为单位矩阵并同步到 DualAHRS。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 无返回值。
 * @note 调用方式与线程约束：IMU 生命周期 reset、融合更新前调用；当前实现无独立锁，必须与
 *       leveling getter、commit 和 DualAHRS update 串行，禁止从 ISR 调用。
 */
void imu_manager_reset_leveling(void);
/**
 * @brief 根据已冻结静态统计计算两路水平矩阵并同步到 DualAHRS。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 无返回值；计算是否通过由两份 `imu_leveling_state_t.valid` 暴露。
 * @note 调用方式与线程约束：仅 boot manager 在静态窗口质量通过后调用一次；会获取 calibration
 *       mutex并执行浮点计算，当前 leveling 全局无锁，禁止与融合/读取并发或从 ISR 调用。
 */
void imu_manager_commit_leveling(void);
/**
 * @brief 分别复制 BMI323 和 LSM303 的当前水平校准状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] bmi 可选 BMI323 状态输出；NULL 表示跳过。
 * @param[out] lsm 可选 LSM303 状态输出；NULL 表示跳过。
 * @return 无返回值。
 * @note 调用方式与线程约束：boot manager 诊断/准入读取；不保存指针、不阻塞，但当前实现
 *       无锁复制，必须与 reset/commit 串行，调用方检查各自 `valid`。
 */
void imu_manager_get_leveling_states(imu_leveling_state_t *bmi,
                                     imu_leveling_state_t *lsm);

/**
 * @brief 互斥复制 manager 最近发布的 BMI323 物理量快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] data 调用方拥有的输出对象，不允许为 NULL；物理量单位为 m/s^2、
 *                  rad/s，时间戳单位见结构体字段。
 * @return OK 表示结构体已复制；NULL 返回 INVALID_ARG，manager 未初始化返回 NOT_READY。
 * @note 调用方式与线程约束：任务上下文诊断/遥测读取；获取可无限等待的 BMI data mutex，
 *       不保存指针，调用方仍须检查 `valid/accel_valid/gyro_valid`，禁止从 ISR 调用。
 */
bsp_status_t imu_get_bmi323_data(bmi323_data_t *data);
/**
 * @brief 设置生产 DualAHRS 路径的 BMI323 ODR 并清空原始样本环形缓冲。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] sample_rate 生产 manager 当前只接受 `BMI323_SAMPLE_RATE_200HZ`。
 * @return OK 表示驱动配置/缓存成功；其他值表示参数、锁资源或底层配置失败。
 * @note 调用方式与线程约束：仅初始化屏障提交或明确停止采样后的重配置路径调用；会创建/等待
 *       mutex、阻塞访问 SPI 并丢弃待处理 BMI 样本，禁止与采样并发或从 ISR 调用。
 */
bsp_status_t imu_manager_set_bmi323_sample_rate(bmi323_sample_rate_t sample_rate);
/**
 * @brief 读取 BMI323 驱动当前缓存的 ODR 配置。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 当前 `bmi323_sample_rate_t`；不证明硬件在线或配置回读成功。
 * @note 调用方式与线程约束：启动管理器/诊断任务读取；不阻塞、不访问 SPI，但与 setter 并发
 *       时不构成事务级快照。
 */
bmi323_sample_rate_t imu_manager_get_bmi323_sample_rate(void);
/**
 * @brief 互斥复制 BMI323 捕获、延迟、丢样、队列深度和实测 ODR 统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] stats 调用方拥有的输出对象，不允许为 NULL。
 * @return OK 表示快照有效；NULL 返回 INVALID_ARG，manager 未初始化返回 NOT_READY。
 * @note 调用方式与线程约束：低频诊断任务读取；获取可无限等待的 BMI mutex，不清零计数、
 *       不保存指针，禁止从 ISR 调用。
 */
bsp_status_t imu_manager_get_bmi323_capture_stats(bmi323_capture_stat_t *stats);
/**
 * @brief 创建唯一 BMI323 高频采样任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 已创建或创建成功返回 OK；manager 未初始化返回 NOT_READY，任务创建失败
 *         返回 ERROR，无 RTOS 构建返回 UNSUPPORTED。
 * @note 调用方式与线程约束：仅初始化屏障提交后调用；使用 `xTaskCreate()`，可能分配 RTOS
 *       资源，重复调用不创建第二任务，禁止从 ISR 调用。
 */
bsp_status_t imu_manager_start_bmi323_task(void);
/**
 * @brief 互斥复制最近一次 LSM303 加速度快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] data 调用方拥有的输出对象，单位 m/s^2，时间戳单位见字段。
 * @return OK 表示结构体已复制；NULL 返回 INVALID_ARG，manager 未初始化返回 NOT_READY。
 * @note 调用方式与线程约束：任务上下文诊断读取；获取可无限等待的 LSM mutex，不保存指针，
 *       复制成功不等于样本 freshness 有效，禁止从 ISR 调用。
 */
bsp_status_t imu_manager_get_lsm_accel(lsm_accel_data_t *data);
/**
 * @brief 互斥复制最近一次 LSM303 磁场快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] data 调用方拥有的输出对象，单位 uT，时间戳单位见字段。
 * @return OK 表示结构体已复制；NULL 返回 INVALID_ARG，manager 未初始化返回 NOT_READY。
 * @note 调用方式与线程约束：任务上下文诊断读取；获取可无限等待的 LSM mutex，不保存指针，
 *       复制成功不等于样本 freshness 有效，禁止从 ISR 调用。
 */
bsp_status_t imu_manager_get_lsm_mag(lsm_mag_data_t *data);
/**
 * @brief 互斥复制最近发布的双 IMU 统一原始快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] snapshot 调用方拥有的输出对象，不允许为 NULL。
 * @return OK 表示结构体已复制；NULL 返回 INVALID_ARG，manager 未初始化返回 NOT_READY。
 * @note 调用方式与线程约束：遥测/标定消费者在任务上下文读取；获取可无限等待的 snapshot
 *       mutex，不保存指针，调用方必须检查每路 valid 和独立时间戳，禁止从 ISR 调用。
 */
bsp_status_t imu_manager_get_snapshot(imu_raw_data_t *snapshot);
/**
 * @brief 互斥复制 BMI323 manager 更新/读取统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] stats 调用方拥有的输出对象，不允许为 NULL。
 * @return OK 表示统计已复制；NULL 返回 INVALID_ARG，manager 未初始化返回 NOT_READY。
 * @note 调用方式与线程约束：启动协调/诊断任务低频读取；获取可无限等待的 BMI mutex，
 *       读取不清零，禁止从 ISR 调用。
 */
bsp_status_t imu_get_bmi323_stats(imu_sensor_stats_t *stats);
/**
 * @brief 互斥复制 LSM303 manager 更新/读取统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] stats 调用方拥有的输出对象，不允许为 NULL。
 * @return OK 表示统计已复制；NULL 返回 INVALID_ARG，manager 未初始化返回 NOT_READY。
 * @note 调用方式与线程约束：启动协调/诊断任务低频读取；获取可无限等待的 LSM mutex，
 *       读取不清零，禁止从 ISR 调用。
 */
bsp_status_t imu_get_lsm303_stats(imu_sensor_stats_t *stats);

/* One scheduler iteration, useful for bare-metal integration and tests. */
/**
 * @brief 执行一次 IMU manager 调度迭代。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 完整透传 `imu_update()` 的 `bsp_status_t`。
 * @note 调用方式与线程约束：由 `imu_task()` 周期调用，也可供无 RTOS 集成；语义和阻塞特性
 *       与 `imu_update()` 相同，禁止重入或从 ISR 调用。
 */
bsp_status_t imu_task_step(void);

/* FreeRTOS-compatible task entry; uses the BSP tick in the no-RTOS fallback. */
/**
 * @brief 周期推进 boot manager、传感器更新、恢复和健康日志的永久任务入口。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] argument 任务参数，当前实现不解引用，通常传 NULL。
 * @return 无返回值；RTOS 和无 RTOS 分支均为永久循环。
 * @note 调用方式与线程约束：仅由 `imu_runtime_start()` 创建；包含阻塞总线、mutex、日志和
 *       周期延时，禁止手工并发调用或从 ISR 调用。
 */
void imu_task(void *argument);

/**
 * @brief 查询 manager 初始化屏障及当前生产在线条件。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 正常镜像中 manager 已初始化且 LSM303 加速度/磁场均在 freshness/失败
 *         门内返回 1；BMI-only 镜像中要求 BMI 初始化且 online；否则返回 0。
 * @note 调用方式与线程约束：启动协调/诊断任务调用；正常镜像内部获取 LSM mutex，可能阻塞，
 *       不等价于 DualAHRS 主分支 freshness，禁止从 ISR 调用。
 */
uint8_t imu_is_ready(void);
/* 硬件初始化状态与最近数据健康度相互独立。 */
/**
 * @brief 查询 LSM303 最近一次硬件初始化结果。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 初始化成功返回 1，否则返回 0；不检查近期样本 freshness。
 * @note 调用方式与线程约束：任务上下文诊断读取；获取可无限等待的 LSM mutex，禁止从 ISR 调用。
 */
uint8_t imu_manager_get_lsm303_init_success(void);
/**
 * @brief 查询 LSM303 初始化、连续失败计数及两路样本年龄组成的在线状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 初始化成功、加速度/磁场失败计数未超限且年龄均低于 1 秒时返回 1，
 *         否则返回 0。
 * @note 调用方式与线程约束：任务上下文健康/安全判断；读取单调时基并获取 LSM mutex，
 *       可能阻塞，结果不能跨周期缓存，禁止从 ISR 调用。
 */
uint8_t imu_manager_get_lsm303_online(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_MANAGER_H */
