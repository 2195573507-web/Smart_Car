#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 轮速斜坡/PID 公共接口。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 契约补充日期：2026-08-31。
 * 所有速度、加速度和输出单位由 wheel_control_params.h 约定；该模块不访问 UART、
 * 传感器或电机硬件，调用方负责提供有效的动态 dt。
 */

/** 目标斜坡的持久状态；单位由实际目标量决定。 */
typedef struct {
    /** 上一次斜坡输出；轮速路径中单位为 mm/s。 */
    float current_target;
    /** 每秒最大变化量；轮速路径中单位为 mm/s^2。 */
    float max_accel;
} Ramp_Profile_t;

/** 单轮控制器的持久状态；同一实例不允许并发读写。 */
typedef struct {
    float kp; /**< 比例增益。 */
    float ki; /**< 积分增益。 */
    float kd; /**< 预留微分增益；当前步进实现不计算 D 项。 */
    float max_out; /**< 总 PWM 输出绝对值上限。 */
    float max_iout; /**< 积分项绝对值上限。 */
    float deadband; /**< 轮速误差死区，单位 mm/s。 */
    float integral; /**< 当前积分累积状态。 */
    float previous_error; /**< 上一次轮速误差，当前兼容状态保留。 */
    float filtered_feedback; /**< 当前低通后的实际轮速，单位 mm/s。 */
    float previous_filtered_feedback; /**< 上一次低通反馈，供状态连续性使用。 */
    float feedback_alpha; /**< 反馈低通旧值权重，范围由配置保证。 */
    float output; /**< 最近一次有符号 PWM 输出。 */
    unsigned char initialized; /**< 已建立反馈历史的布尔标志。 */
} pid_controller_t;

typedef pid_controller_t PID_Controller_t;

/**
 * @brief 按最大加速度和 dt 推进目标斜坡。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param ramp 可写斜坡状态；调用前必须初始化 current_target 和 max_accel。
 * @param final_target 最终目标，单位必须与 ramp->current_target 一致。
 * @param dt_seconds 本次更新间隔，单位秒，必须为正的有限值。
 * @return ramp 为 NULL 时返回 0；其他参数无效时保持并返回
 *         ramp->current_target；成功时返回本次限速后目标。
 * 调用方式：由 MotorBoard 控制任务按反馈周期调用。
 * 线程约束：函数不阻塞、不内部加锁；同一 ramp 必须由调用方串行化，不在 ISR 中调用。
 */
float Ramp_Update(Ramp_Profile_t *ramp, float final_target, float dt_seconds);

/**
 * @brief 更新目标斜坡的最大加速度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param ramp 可写斜坡状态；NULL 时不执行操作。
 * @param max_accel 新的最大加速度，必须为大于等于 0 的有限值。
 * @return 无。参数无效时原配置保持不变。
 * 调用方式：由参数下发路径在调用方排他保护下调用。
 * 线程约束：函数不加锁、不阻塞，不在 ISR 中调用。
 */
void Ramp_Update_Max_Accel(Ramp_Profile_t *ramp, float max_accel);

/**
 * @brief 初始化单轮控制器的增益、限幅、死区和反馈历史。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param pid 可写控制器实例；NULL 时不执行操作。
 * @param kp 比例增益，由调用方保证为有限值。
 * @param ki 积分增益，由调用方保证为有限值。
 * @param kd 微分增益保留字段；当前步进实现不计算 D 项。
 * @param max_out 有限的总输出绝对值上限；内部保存其绝对值。
 * @param max_iout 有限的积分输出绝对值上限；内部保存其绝对值。
 * @param deadband 有限的误差死区绝对值；内部保存其绝对值。
 * @return 无。
 * 调用方式：创建 MotorBoard 控制任务前对每轮调用一次。本函数会清空
 * 历史状态。
 * 线程约束：不内部加锁；同一 pid 必须由调用方排他访问，不在 ISR 中调用。
 */
void pid_controller_init(pid_controller_t *pid, float kp, float ki, float kd,
                         float max_out, float max_iout, float deadband);

/**
 * @brief 在线更新控制器增益，保留积分和反馈历史。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param pid 可写控制器实例；NULL 时不执行操作。
 * @param kp 新比例增益，必须为有限值。
 * @param ki 新积分增益，必须为有限值。
 * @param kd 新微分增益保留值，必须为有限值；当前步进不使用 D 项。
 * @return 无。任一参数无效时三个增益均保持不变。
 * 调用方式：由参数下发路径在调用方排他保护下调用。
 * 线程约束：函数本身不加锁、不阻塞，不在 ISR 中调用。
 */
void PID_Update_Gains(PID_Controller_t *pid, float kp, float ki, float kd);

/**
 * @brief 清除积分、误差、反馈滤波和输出历史。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param pid 可写控制器实例；NULL 时不执行操作。
 * @return 无。
 * 调用方式：初始化、急停或闭环重锚时调用。
 * 线程约束：函数不内部加锁，同一 pid 实例必须由调用方串行化，不在 ISR 中调用。
 */
void pid_controller_reset(pid_controller_t *pid);

/**
 * @brief 执行一次内置轮速前馈、P/I 调节、死区和抗积分饱和步进。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param pid 可写控制器状态；NULL 时返回 0。
 * @param target 斜坡后目标轮速，单位 mm/s，必须为有限值。
 * @param actual 编码器实际轮速，单位 mm/s，必须为有限值。
 * @param dt_seconds 本次控制间隔，单位秒，必须为正的有限值。
 * @return 限制在 +/-pid->max_out 内的有符号 PWM 命令；参数无效或
 *         |target| < 1 mm/s 时返回 0。
 * 调用方式：由 MotorBoard 反馈处理路径每轮调用一次。函数会修改 pid
 * 内部状态。
 * 线程约束：不加锁、不分配内存，同一 pid 单 owner，不在 ISR 中调用。
 */
float pid_controller_step_with_feedforward(pid_controller_t *pid,
                                            float target, float actual,
                                            float dt_seconds);

/**
 * @brief 兼容步进入口，当前行为与 pid_controller_step_with_feedforward() 完全一致。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param pid 可写控制器状态；NULL 时返回 0。
 * @param target 斜坡后目标轮速，单位 mm/s。
 * @param actual 编码器实际轮速，单位 mm/s。
 * @param dt_seconds 本次控制间隔，单位秒。
 * @return 与 pid_controller_step_with_feedforward() 相同的有符号 PWM 命令或 0。
 * 调用方式：MotorBoard 任务使用的现有入口；保留该符号用于源码兼容。
 * 线程约束：不内部加锁或阻塞，同一 pid 单 owner，不在 ISR 中调用。
 */
float pid_controller_step(pid_controller_t *pid, float target, float actual,
                          float dt_seconds);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
