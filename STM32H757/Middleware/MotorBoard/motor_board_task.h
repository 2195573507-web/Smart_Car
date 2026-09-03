#ifndef MOTOR_BOARD_TASK_H
#define MOTOR_BOARD_TASK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* MotorBoard 控制任务接口；创建人：待确认（当前维护人：Zhiqin）。
 * 目标数组顺序固定为 [M1:RR, M2:RF, M3:LR, M4:LF]，最终输出仍受安全门控。 */

#ifdef __cplusplus
extern "C" {
#endif

/** 原子读取的一次四轮 MSPD 反馈及到达元数据。 */
typedef struct {
    float speed_mm_s[4]; /**< 已修正编码器极性的四轮速度，单位 mm/s，顺序 [RR,RF,LR,LF]。 */
    uint32_t timestamp_ms; /**< CM7 接受该 MSPD 帧时的 HAL 单调 tick，单位 ms。 */
    uint32_t sequence; /**< 每接受一条 MSPD 自增一次的序号，允许自然回绕。 */
    bool valid; /**< 自任务初始化后是否至少接受过一条 MSPD。 */
} motor_board_wheel_speed_snapshot_t;

/**
 * @brief 创建并启动唯一的 MotorBoard 控制任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * 传入参数：无。
 * @return 无；transport 未就绪、任务已存在或创建失败时直接返回，创建失败会记错误日志。
 * 调用方式：完成 MB_Transport_Init() 和 MB_Protocol_Init() 后，由普通启动任务调用。
 * 线程约束：内部调用 xTaskCreate()，禁止从 ISR 调用；重复调用不会创建第二个任务。
 */
void motor_board_task_start(void);
/**
 * @brief 执行 MotorBoard 配置序列、反馈解析、四轮 PID、遥测和链路恢复。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * @param argument FreeRTOS 任务参数，当前忽略，允许为 NULL。
 * @return 无；任务进入 1 ms 周期的永久循环，不应返回。
 * 调用方式：仅由 motor_board_task_start() 作为 xTaskCreate() 入口注册。
 * 线程约束：该任务是 MotorBoard parser 和控制循环的唯一 owner；不得直接并行调用。
 */
void motor_board_task(void *argument);
/**
 * @brief 向 MotorBoard 控制循环提交四路目标轮速。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * @param speeds 四元素只读数组，单位 mm/s，顺序固定为 [RR, RF, LR, LF]；
 *               调用期间复制，不保留指针，禁止 NULL、NaN、Inf 或超出轮速上限的值。
 * @return 非零目标返回 true 仅表示目标已复制到 RAM；非法输入返回 false；
 *         四路全零时转入 motor_board_force_stop()，返回值表示零 PWM 是否成功排队。
 * 调用方式：MotorBoard 任务启动后，由通过同步、姿态和安全门的底盘/服务任务调用。
 * 线程约束：使用 FreeRTOS 临界区，禁止从 ISR 调用；早于任务初始化的目标可能被清零。
 */
bool motor_board_set_target_wheel_speeds(const float speeds[4]);
/**
 * @brief 原子更新四轮 PID 增益和目标轮速斜坡的最大加速度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * @param kp 比例增益，必须有限且位于 SRP_PID_KP_MIN..SRP_PID_KP_MAX。
 * @param ki 积分增益，必须有限且位于 SRP_PID_KI_MIN..SRP_PID_KI_MAX。
 * @param kd 微分增益，必须有限且位于 SRP_PID_KD_MIN..SRP_PID_KD_MAX。
 * @param max_accel 轮速斜坡上限，单位 mm/s^2，必须位于 SRP_PID_ACCEL_MIN..MAX。
 * @return true 表示四轮参数已更新；false 表示至少一个参数非法，原配置保持不变。
 * 调用方式：由已校验的 SRP PID 参数命令在普通任务上下文调用。
 * 线程约束：使用 FreeRTOS 临界区，禁止从 ISR 调用；不会清除已有 PID 积分/反馈历史。
 */
bool motor_board_update_pid_params(float kp, float ki, float kd,
                                   float max_accel);
/**
 * @brief 清零四轮目标与 PID/斜坡状态，并排队一条四路零 PWM 命令。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * 传入参数：无。
 * @return true 仅表示零 PWM 文本已进入 USART6 TX ring；false 表示排队失败。
 * 调用方式：由急停、链路失效、安全门丢失或零目标路径调用；false 时上层须保留故障状态。
 * 线程约束：在 FreeRTOS 临界区串行化目标/PID 与排队动作，禁止从 ISR 调用；
 *           返回 true 不代表字节已发送、电机板已执行或车辆已经物理停止。
 */
bool motor_board_force_stop(void);
/**
 * @brief 复制最近一次 MotorBoard 反馈换算后的四路实际轮速。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * @param speeds 四元素可写数组，单位 mm/s，顺序固定为 [RR, RF, LR, LF]；NULL 时不写入。
 * @return 无。
 * 调用方式：普通任务按需读取速度兼容视图；内部取得完整 snapshot 后复制 speed，
 *           不向调用方暴露 valid/timestamp/sequence，也不因 snapshot invalid 拒绝写入。
 * 线程约束：snapshot 在短 FreeRTOS 临界区原子复制；禁止 ISR 调用，输出数组归调用方所有。
 */
void motor_board_get_actual_wheel_speeds(float speeds[4]);
/**
 * @brief 原子复制最近一次已校正 MSPD 四轮速度及其到达元数据。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param snapshot 调用方拥有的可写输出；不得为 NULL。即使返回 false，也会写入当前完整快照。
 * @return 至少处理过一条有效 MSPD 帧时返回 true，否则 false；不包含 freshness 判断。
 * 调用方式：底盘状态任务读取后结合 `timestamp_ms` 与当前 tick 判断 200 ms 新鲜度，
 *           `sequence` 每接受一条 MSPD 自然递增并允许 uint32 回绕。
 * 线程约束：在短 FreeRTOS 临界区同时复制四轮、时间戳、序号和 valid；禁止 ISR 调用。
 */
bool motor_board_get_actual_wheel_speed_snapshot(
    motor_board_wheel_speed_snapshot_t *snapshot);
/**
 * @brief 读取最近一次有效 MotorBoard 电池电压反馈。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * @param voltage 可写输出，单位 V；不得为 NULL，失败时内容保持不变。
 * @return true 表示已写入有限的缓存电压；尚无反馈、缓存非有限或参数无效时返回 false。
 * 调用方式：普通任务在收到电池响应后按需读取；返回值不表示反馈仍然新鲜。
 * 线程约束：无锁快照，禁止从 ISR 调用；需要时效保证的调用方还须结合上层时间戳/状态机。
 */
bool motor_board_get_battery_voltage(float *voltage);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_BOARD_TASK_H */
