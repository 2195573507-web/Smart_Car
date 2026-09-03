#ifndef IMU_BOOT_MANAGER_H
#define IMU_BOOT_MANAGER_H

#include <stdint.h>

#include "bsp_status.h"
#include "imu_calibration.h"

/*
 * 双 IMU 启动/标定生命周期管理器。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 它是 DUAL_IMU_BOOT 的唯一状态权威；雷达 PWM/STM-S3 通知只作为外部事件输入，
 * 不得被调用方用来跳过自检、静态窗口或故障状态。
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 双 IMU 启动、在线自检、静态标定和故障锁定的权威生命周期阶段。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @note 该枚举由 boot manager 独占写入；调用方只能通过快照读取。数值 4、5 为历史诊断
 *       接收端保留，`IMU_PHASE_COUNT` 同时作为阶段计时数组上界，禁止压缩或重排。
 */
typedef enum
{
    IMU_PHASE_IDLE = 0,               /**< 复位后的空闲阶段；下一次 step 才释放初始化 worker。 */
    IMU_PHASE_INIT = 1,               /**< LSM303 与 BMI323 初始化 worker 正在运行或等待收敛。 */
    IMU_PHASE_SELF_TEST = 2,          /**< 固定观察窗内等待两路 IMU 各自产生完整有效样本。 */
    IMU_PHASE_STATIC_CALIBRATION = 3, /**< 等待零 PWM、静置稳定、采样及质量/水平校准判定。 */
    /* 数值 4、5 为已持久化诊断读取端保留，不得复用。 */
    IMU_PHASE_READY = 6,  /**< 静态质量门与主 BMI 水平校准均通过，可参与运动准入。 */
    IMU_PHASE_FAILED = 7, /**< 已锁定首个启动/标定错误；显式 reset 前保持不可运动。 */
    IMU_PHASE_COUNT = 8   /**< 阶段编号空间和 `phase_timing[]` 元素数，不是有效阶段。 */
} imu_phase_t;

/**
 * @brief 双 IMU 生命周期失败原因编码，随兼容状态负载以单字节对外发送。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @note boot manager 锁定进入 FAILED 的首个错误；数值属于协议/诊断兼容契约，禁止重排。
 */
typedef enum
{
    IMU_ERROR_NONE = 0,             /**< 当前没有生命周期错误。 */
    IMU_ERROR_LSM_INIT,             /**< LSM303 初始化 worker 已结束但初始化失败。 */
    IMU_ERROR_BMI_INIT,             /**< BMI323 初始化 worker 已结束但初始化失败。 */
    IMU_ERROR_INIT_TIMEOUT,         /**< 双初始化未在规定期限内同时成功。 */
    IMU_ERROR_LSM_SELF_TEST,        /**< 自检观察窗内未看到完整有效的 LSM303 加速度/磁场。 */
    IMU_ERROR_BMI_SELF_TEST,        /**< 自检观察窗内未看到完整有效的 BMI323 加速度/陀螺。 */
    IMU_ERROR_RADAR_SYNC_TIMEOUT,   /**< 静态阶段超时前未收到已校验的零 PWM READY。 */
    IMU_ERROR_STATIC_WINDOW,        /**< 静态运动、样本质量或主水平校准未通过，且恢复失败。 */
    /* 保留状态负载使用的历史错误字节 12。 */
    IMU_ERROR_TASK_CREATE = 12      /**< 初始化 worker 创建或收尾同步失败。 */
} imu_error_t;

/**
 * @brief 单个权威生命周期阶段的单调毫秒起止时间快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @note 由 boot manager 持锁更新并按值复制给调用方；32 位单调时基允许自然回绕，
 *       当前阶段的结束时间保持 0，不能直接把 0 解释为启动时刻。
 */
typedef struct
{
    uint32_t start_timestamp; /**< 进入该阶段的 `imu_time_now_ms()` 值，单位 ms。 */
    uint32_t end_timestamp;   /**< 离开该阶段的单调时间，单位 ms；0 表示尚未离开/无记录。 */
} imu_phase_timing_t;

/**
 * @brief boot manager 持有的双 IMU 权威阶段、进度、错误及诊断计时值快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @note 实例由模块内部所有，getter 向调用方整体复制；进度是生命周期显示值，不等价于
 *       传感器 freshness、实际采样质量或车辆可运动许可。
 */
typedef struct
{
    imu_phase_t phase; /**< 当前权威生命周期阶段。 */

    uint8_t lsm_progress; /**< LSM303 当前阶段显示进度，范围 0..100。 */
    uint8_t bmi_progress; /**< BMI323 当前阶段显示进度，范围 0..100。 */

    uint8_t overall_progress; /**< 加权生命周期总进度，范围 0..100；FAILED 保留失败前值。 */

    uint32_t phase_start_time; /**< 当前阶段起点的单调时间，单位 ms。 */

    imu_error_t error; /**< 首个锁定的失败原因；未失败时为 `IMU_ERROR_NONE`。 */

    uint32_t phase_end_time; /**< 当前阶段结束时间，单位 ms；实现进入新阶段后将其复位为 0。 */
    uint8_t flags; /**< `DUAL_IMU_STATUS_FLAG_*` 位图，由阶段进度刷新逻辑整体重建。 */
    imu_phase_timing_t phase_timing[IMU_PHASE_COUNT]; /**< 按 `imu_phase_t` 数值索引的阶段历史。 */
} dual_imu_manager_t;

#define DUAL_IMU_STATUS_FLAG_LSM_PHASE_COMPLETE UINT8_C(0x01)
#define DUAL_IMU_STATUS_FLAG_BMI_PHASE_COMPLETE UINT8_C(0x02)
#define DUAL_IMU_STATUS_FLAG_PHASE_ACTIVE       UINT8_C(0x04)

/**
 * @brief 面向旧滤波、日志和 BOOT_READY 负载的只读兼容启动状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @note 权威状态仍是 `imu_phase_t`；本枚举由其和静态窗口子状态投影而来。保留值和别名
 *       属于跨模块兼容契约，禁止根据声明顺序推断状态必然可达。
 */
typedef enum
{
    IMU_BOOT_INIT = 0, /**< IDLE、INIT 或 SELF_TEST 的兼容汇总值。 */
    /* BOOT_READY 负载使用该稳定生命周期值。 */
    WAIT_SYNC, /**< 静态阶段尚未收到 S3 零 PWM READY，持续等待同步。 */
    /* 保留旧调用方使用的源码别名，数值与 WAIT_SYNC 相同。 */
    WAIT_RADAR_ZERO = WAIT_SYNC, /**< `WAIT_SYNC` 的历史名称，不是独立状态。 */
    STATIC_CAL_WAIT,   /**< 零 PWM 已确认，等待静置稳定定时到期。 */
    STATIC_CAL_SAMPLE, /**< 静态窗口已打开，正在累计三条传感器数据流。 */
    STATIC_CAL_DONE,   /**< 静态结果已冻结的兼容值；通常立即投影为 `IMU_READY`。 */
    /* 数值 5..9 为旧接收端保留，不得复用。 */
    IMU_READY = 10, /**< 权威阶段 READY 的兼容值。 */
    IMU_ERROR = 11, /**< 权威阶段 FAILED 的兼容值。 */
    /* 追加在历史值之后；当前权威阶段投影逻辑不会生成该值。 */
    SYNCED, /**< 兼容保留的瞬态同步值；不可作为当前状态机必经状态。 */
    /* 当前权威阶段投影逻辑不会生成该恢复值。 */
    CAL_SYNC_RECOVERY /**< 兼容保留的重新等待零 PWM 同步状态。 */
} imu_boot_state_t;

/**
 * @brief 启动/标定状态的跨模块发送回调类型。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] message_id SRP 消息 ID，由启动管理器选择。
 * @param[in] flags SRP 标志位。
 * @param[in] payload 只读 payload，通常位于调用栈，仅在回调期间有效。
 * @param[in] length payload 字节数。
 * @return 无返回值；调用方无法从该回调获知传输成功与否。
 * @note 调用方式与线程约束：由 `imu_boot_manager_step()` 所在任务在释放内部锁后调用；实现
 *       必须在返回前复制/入队，不得保存指针、阻塞或递归调用启动管理器。
 */
typedef void (*imu_boot_transport_callback_t)(uint16_t message_id,
                                              uint8_t flags,
                                              const uint8_t *payload,
                                              uint8_t length);

/**
 * @brief 面向安全协调、日志和 SRP 兼容状态帧的轻量启动状态摘要。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @note getter 将字段复制到调用方对象；其中样本计数是固定 600 点兼容视图的时间进度，
 *       不是 `imu_calibration_sample_counts_t` 中三条流的真实接受计数。
 */
typedef struct
{
    imu_boot_state_t state; /**< 从权威阶段和静态子状态投影出的旧版兼容状态。 */
    imu_phase_t phase; /**< 当前 `DUAL_IMU_BOOT` 权威阶段。 */
    uint8_t progress; /**< 加权生命周期总进度，范围 0..100。 */
    uint8_t lsm_progress; /**< LSM303 当前阶段显示进度，范围 0..100。 */
    uint8_t bmi_progress; /**< BMI323 当前阶段显示进度，范围 0..100。 */
    uint32_t sample_count; /**< 兼容静态窗口虚拟样本进度；非静态/READY 阶段通常为 0。 */
    uint32_t sample_total; /**< 兼容静态窗口固定目标数；当前为 600，非相关阶段为 0。 */
    uint8_t error; /**< `imu_error_t` 的单字节协议视图。 */
    const char *error_reason; /**< 借用的只读静态字符串；调用方不得释放或写入。 */
} imu_boot_status_t;

/**
 * @brief 初始化启动管理器互斥量、生命周期状态和阶段计时。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 无返回值。
 * @note 调用方式与线程约束：IMU manager 初始化阶段调用一次；内部可能创建 mutex、重置标定
 *       状态并写启动日志，可能阻塞，禁止从 ISR 或与状态机并发调用。
 */
void imu_boot_manager_init(void);
/* Call only during initial boot or an explicit user-requested recalibration. */
/**
 * @brief 复位到 `IMU_BOOT_INIT`，清除阶段/错误/一次性事件并重启标定状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 无返回值；已注册 transport 回调会被保留。
 * @note 调用方式与线程约束：仅初始启动或明确授权的重新标定路径调用；会获取可阻塞 mutex
 *       并清除当前结果，必须与 step/update/getter 串行，禁止从 ISR 调用。
 */
void imu_boot_manager_reset(void);
/**
 * @brief 注册或清除启动/标定状态的发送回调。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] callback 长期有效的回调函数；NULL 表示停用发送。
 * @return 无返回值。
 * @note 调用方式与线程约束：S3 服务初始化完成后、状态机发送消息前在任务上下文调用；函数
 *       获取可阻塞 mutex，不接管任何上下文对象，禁止从 ISR 调用。
 */
void imu_boot_manager_set_transport(imu_boot_transport_callback_t callback);
/**
 * @brief 推进一次双 IMU 初始化、自检、静态标定、超时和状态发布流程。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 无返回值；阶段和错误通过 getter/transport 暴露。
 * @note 调用方式与线程约束：仅由 IMU owner 任务周期调用；内部会获取多个 mutex，并可能创建
 *       worker、计算标定、记录日志和调用 transport，可能阻塞，禁止重入或从 ISR 调用。
 */
void imu_boot_manager_step(void);
/**
 * @brief 输入最新双 IMU 快照，推进自检观测并在静态窗口累计 LSM303 数据。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] raw_data 调用方拥有的只读快照，物理量/时间戳单位遵循
 *                     `imu_raw_data_t`；NULL 时不操作，指针不会被保存。
 * @return 无返回值；非自检/静态采样阶段只更新进度。
 * @note 调用方式与线程约束：IMU manager 在发布新统一快照后调用；会获取可阻塞 mutex，
 *       活跃静态窗口还会进入 calibration mutex，禁止从 ISR 或多个 writer 并发调用。
 */
void imu_boot_manager_update(const imu_raw_data_t *raw_data);
/**
 * @brief 处理已校验的雷达 PWM READY 事件并准入静态零 PWM 稳定阶段。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] speed 雷达 PWM 百分比；当前仅在静态标定阶段接受 0。
 * @return 1 表示事件被当前状态机接受，0 表示速度或阶段不允许，状态不会越级。
 * @note 调用方式与线程约束：由 CM7 S3 服务任务处理已校验 SRP payload 后调用；函数获取
 *       可阻塞 mutex，禁止从 ISR 调用，接受事件不等于整车运动就绪。
 */
uint8_t imu_boot_manager_on_radar_pwm_ready(uint8_t speed);

/**
 * @brief 读取当前兼容启动生命周期状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 当前 `imu_boot_state_t` 值；后续 step/reset 可立即使结果失效。
 * @note 调用方式与线程约束：任务上下文诊断/状态判断；获取可阻塞 mutex，禁止从 ISR 调用。
 */
imu_boot_state_t imu_boot_manager_get_state(void);
/**
 * @brief 复制兼容状态、阶段进度、虚拟样本计数和错误原因摘要。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] status 调用方拥有的输出对象；NULL 时不执行复制。
 * @return 无返回值；`error_reason` 指向模块静态字符串，调用方不得修改或释放。
 * @note 调用方式与线程约束：遥测/启动协调任务低频读取；获取可阻塞 mutex，不保存输出指针，
 *       禁止从 ISR 调用，返回仅是调用时刻的软件快照。
 */
void imu_boot_manager_get_status(imu_boot_status_t *status);
/**
 * @brief 复制双 IMU 原生阶段、进度、错误和计时状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] status 调用方拥有的输出对象；NULL 时不执行复制。
 * @return 无返回值。
 * @note 调用方式与线程约束：诊断任务低频读取；获取可阻塞 mutex，不保存输出指针，
 *       禁止从 ISR 调用。
 */
void imu_boot_manager_get_dual_status(dual_imu_manager_t *status);
/**
 * @brief 复制指定生命周期阶段的起止时间。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] phase 目标阶段，必须小于 `IMU_PHASE_COUNT`。
 * @param[out] timing 调用方拥有的时间输出，单位 ms，不允许为 NULL。
 * @return 1 表示输出已写入；参数为空或阶段越界返回 0，输出保持原值。
 * @note 调用方式与线程约束：任务上下文诊断读取；获取可阻塞 mutex，禁止从 ISR 调用。
 */
uint8_t imu_boot_manager_get_phase_timing(imu_phase_t phase,
                                          imu_phase_timing_t *timing);
/**
 * @brief 查询双 IMU 生命周期是否处于 READY 阶段。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return READY 返回 1，否则返回 0；不替代实时传感器 freshness 检查。
 * @note 调用方式与线程约束：启动协调/底盘门控在任务上下文调用；获取可阻塞 mutex，
 *       禁止从 ISR 调用，结果不能跨周期缓存。
 */
uint8_t imu_boot_manager_is_ready(void);
/**
 * @brief 查询双 IMU 生命周期是否处于 FAILED 阶段。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return FAILED 返回 1，否则返回 0。
 * @note 调用方式与线程约束：任务上下文安全/诊断判断；获取可阻塞 mutex，禁止从 ISR 调用。
 */
uint8_t imu_boot_manager_is_error(void);
/**
 * @brief 读取当前生命周期总体进度百分比。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 0..100 的软件进度值；FAILED 时可能保留失败前进度。
 * @note 调用方式与线程约束：遥测/诊断任务低频读取；获取可阻塞 mutex，禁止从 ISR 调用。
 */
uint8_t imu_boot_manager_get_progress(void);
/**
 * @brief 读取兼容状态摘要中的当前静态标定样本计数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 当前兼容样本数；非静态/READY 阶段通常为 0。
 * @note 调用方式与线程约束：任务上下文遥测读取；内部调用状态快照 getter 并可能阻塞 mutex，
 *       禁止从 ISR 调用。
 */
uint32_t imu_boot_manager_get_sample_count(void);
/**
 * @brief 读取兼容状态摘要中的静态标定目标样本总数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 当前阶段目标总数；非静态/READY 阶段通常为 0。
 * @note 调用方式与线程约束：任务上下文遥测读取；内部调用状态快照 getter 并可能阻塞 mutex，
 *       禁止从 ISR 调用。
 */
uint32_t imu_boot_manager_get_sample_total(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_BOOT_MANAGER_H */
