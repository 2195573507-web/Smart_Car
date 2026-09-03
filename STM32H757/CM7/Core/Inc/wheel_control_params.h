#ifndef WHEEL_CONTROL_PARAMS_H
#define WHEEL_CONTROL_PARAMS_H

/*
 * 四轮底盘速度闭环参数。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 契约补充日期：2026-08-31。
 * 修改参数前必须通过主机回放和受控台架验证，不能以调试日志替代安全门。
 */

/*
 * 速度统一使用 mm/s，加速度统一使用 mm/s^2，PWM 输出使用电机板协议的
 * 原始有符号量纲。修改本文件后，斜坡、反馈滤波、死区和四路 PI 参数会
 * 同步生效，避免控制任务与 PID 模块出现参数不一致。
 */

/* 目标速度斜坡的最大加速度，控制周期内据此限制目标变化量。 */
#define WHEEL_RAMP_MAX_ACCEL 800.0f

/* 编码器速度反馈一阶低通滤波的新样本权重，取值范围为 0.0f 到 1.0f；
 * 值越大跟随越快，值越小平滑越强。 */
#define WHEEL_FEEDBACK_LPF_ALPHA 0.35f

/* 低于此值的非零目标在进入斜坡发生器前截断为零，避开不可控微速区。 */
#define WHEEL_SPEED_MIN_TARGET_LIMIT 30.0f

/* 目标与滤波反馈的绝对误差小于此值时按零误差处理，抑制低速量化抖动。 */
#define WHEEL_SPEED_DEADBAND 6.0f

/* 线性速度前馈系数，单位为 PWM / (mm/s)；总前馈另叠加平滑摩擦项。 */
#define WHEEL_FEEDFORWARD_KV 1.40f

/* 库仑静摩擦补偿峰值及其随目标速度平滑过渡的速度范围。 */
#define WHEEL_FRICTION_COMP_PWM 260.0f
#define WHEEL_FRICTION_RAMP_VELOCITY 80.0f

/* 平稳型 PI(D) 控制器比例、积分和微分增益。当前步进实现只计算
 * P/I 项，KD 仅保留配置兼容性，修改 KD 不会改变当前 PWM 输出。 */
#define WHEEL_PID_KP 1.10f
#define WHEEL_PID_KI 0.060f
#define WHEEL_PID_KD 0.00f

/* 单轮输出幅值校正，用于补偿制造差异。轮序固定为
 * M1=RR、M2=RF、M3=LR、M4=LF；只缩放幅值，不改变方向。 */
#define WHEEL_TRIM_M1 1.08f
#define WHEEL_TRIM_M2 1.00f
#define WHEEL_TRIM_M3 1.00f
#define WHEEL_TRIM_M4 1.00f

/* 积分出力与总 PWM 出力限幅。积分限幅覆盖完整闭环调节范围。 */
#define WHEEL_PID_MAX_IOUT 700.0f
#define WHEEL_PID_MAX_OUT 2500.0f

/* 单轮目标速度绝对值上限，超过该值的上位机命令会被拒绝。 */
#define WHEEL_SPEED_MAX_TARGET_LIMIT 1000.0f

#endif /* WHEEL_CONTROL_PARAMS_H */
