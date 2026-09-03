#ifndef RADAR_CONTROL_H
#define RADAR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 雷达 PWM 与校准状态机接口。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * S3 只控制雷达电机 PWM，不拥有 STM32 姿态/运动安全决策。
 */

#define RADAR_MIN_SPEED 0U
#define RADAR_MAX_SPEED 100U

/** S3 本地雷达 PWM 准入状态；只描述控制软件，不证明电机已物理旋转。 */
typedef enum {
    RADAR_CONTROL_WAIT_STM_QUERY = 0, /**< BOOT 门：等待已校验 STM 启动查询，PWM 保持安全值。 */
    RADAR_CONTROL_WAIT_IMU_CAL,       /**< 标定门：只允许 STM-owned 标定 PWM 覆盖。 */
    RADAR_CONTROL_SOFT_START,         /**< 历史保留状态；当前 App/标定路径不会进入。 */
    RADAR_CONTROL_RUNNING,            /**< 本地标定门已释放，可接受 App 运行占空比。 */
    RADAR_CONTROL_CAL_DONE,           /**< 标定完成的瞬时过渡值，随后立即进入 RUNNING。 */
} radar_control_state_t;

/**
 * @brief 初始化 PWM 控制 mutex 和 WAIT_STM_QUERY 状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 无；mutex/LEDC 更新失败通过日志和后续 false/安全默认值体现。
 * 调用方式：radar_pwm_init() 完成后、其他 radar_control_* 接口前调用一次。
 * 线程约束：仅启动任务调用，禁止从 ISR 调用；成功初始化后的重复调用直接返回，不重置状态。
 */
void radar_control_init(void);

/**
 * @brief 设置应用请求的雷达转速。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param percent 0..100 的占空比百分数。
 * @return true 表示 RUNNING/标定完成/无覆盖且 LEDC 已更新；参数、锁、状态或 LEDC
 *         失败返回 false，并尽力恢复原占空比。
 * 调用方式：仅任务上下文，内部获取控制互斥量；不可从 ISR 调用。
 * 线程约束：最多等待控制 mutex 20 ms；LEDC 失败时会尝试恢复旧值，但调用方仍须把 false
 *           当作硬件状态未确认，不能据此宣称占空比保持不变。
 */
bool radar_control_set_speed(uint8_t percent);

/**
 * @brief 报告 STM-owned IMU 标定状态并推进雷达状态机。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param done true 只在 WAIT_IMU_CAL 接受完成事件；false 在 BOOT 门已释放后回到等待标定并置零 PWM。
 * @return 无；乱序/重复事件或锁失败被拒绝并保持原状态，调用方需再查询状态确认。
 * 调用方式：仅由 S3 标定服务任务根据已校验 SRP 事件调用。
 * 线程约束：可能等待 mutex 20 ms 并执行 LEDC 写，禁止从 ISR/GATT 回调调用。
 */
void radar_control_set_imu_cal_done(bool done);

/**
 * @brief 接受 STM32 启动 PWM 查询并从 WAIT_STM_QUERY 进入 WAIT_IMU_CAL。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 无；LEDC 零占空比提交失败时保持 BOOT 门控，重复/乱序查询不改变状态。
 * 调用方式：仅由已验证的 STM 启动消息处理路径调用。
 * 线程约束：可能等待 mutex 20 ms 并访问 LEDC，禁止从 ISR 调用。
 */
void radar_control_handle_pwm_ready_query(void);

/* 标定 PWM 是明确的 STM32-owned 覆盖，不选择 App 运行速度或下一标定级别。 */
/**
 * @brief 应用 STM32-owned 标定 PWM 覆盖。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param percent 标定占空比百分数，范围 0..100。
 * @return true 表示 WAIT_IMU_CAL 中 LEDC 已应用并取得覆盖权；参数、锁、状态或 LEDC
 *         失败返回 false，并尽力恢复先前占空比。
 * 调用方式：仅服务任务；完成后需调用 radar_control_release_calibration_lock()。
 * 线程约束：最多等待 mutex 20 ms 并执行 LEDC 写；禁止从 ISR/GATT 回调调用。
 */
bool radar_control_set_calibration_pwm(uint8_t percent);
/**
 * @brief 释放标定 PWM 覆盖标志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 无；锁失败时保持覆盖状态。
 * 调用方式：标定事务完成/取消后由服务任务调用；本函数不自动改变当前 PWM。
 * 线程约束：可能等待 mutex 20 ms，禁止从 ISR 调用。
 */
void radar_control_release_calibration_lock(void);
/**
 * @brief 查询标定覆盖是否有效。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 取得 mutex 后返回初始化且覆盖有效状态；锁不可用/未初始化时保守返回 false。
 * 调用方式：服务任务状态诊断；false 可能表示锁超时，不能单独证明覆盖已安全释放。
 * 线程约束：只读但可能等待控制 mutex 最多 20 ms，禁止从 ISR 调用。
 */
bool radar_control_is_calibration_active(void);

/**
 * @brief 查询普通雷达状态机是否处于 RUNNING。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 锁失败或未初始化时返回 false；true 只表示本地雷达状态机 RUNNING，
 *         不代表 STM32 运动链路已同步或雷达物理旋转。
 * 调用方式：服务任务状态诊断；实际占空比和跨芯片健康需结合其他状态读取。
 * 线程约束：可能等待控制 mutex 最多 20 ms，禁止从 ISR 调用。
 */
bool radar_control_is_running(void);

/**
 * @brief 获取当前雷达控制状态机。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 当前状态；锁失败时返回安全默认 RADAR_CONTROL_WAIT_STM_QUERY。
 * 调用方式：服务任务诊断/状态上报；安全准入仍应调用控制接口而非自行比较快照。
 * 线程约束：可能等待 mutex 20 ms，禁止从 ISR 调用。
 */
radar_control_state_t radar_control_get_state(void);

/**
 * @brief 获取当前 s_speed_percent 百分比快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 0..100；锁失败时返回 RADAR_MIN_SPEED。标定覆盖期间该值也会随标定 PWM 更新，
 *         因此并非只代表 App 普通运行请求。
 * 调用方式：用于状态上报；是否处于标定覆盖需同时查询 is_calibration_active()。
 * 线程约束：可能等待 mutex 20 ms，禁止从 ISR 调用。
 */
uint8_t radar_control_get_speed(void);

/**
 * @brief 获取最近一次标定覆盖 PWM 百分比。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 最近一次成功提交的标定覆盖值 0..100；锁失败时返回 RADAR_MIN_SPEED。
 * 调用方式：仅用于标定诊断；释放覆盖不会自动清零该记录，也不证明当前 LEDC 仍使用该值。
 * 线程约束：可能等待 mutex 20 ms，禁止从 ISR 调用。
 */
uint8_t radar_control_get_calibration_pwm(void);

#endif /* RADAR_CONTROL_H */
