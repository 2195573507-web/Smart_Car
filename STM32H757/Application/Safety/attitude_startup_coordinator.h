#ifndef ATTITUDE_STARTUP_COORDINATOR_H
#define ATTITUDE_STARTUP_COORDINATOR_H

#include <stdint.h>

/* 姿态启动安全协调器；创建人：待确认（当前维护人：Zhiqin）。 */

#ifdef __cplusplus
extern "C" {
#endif

/** 仅在 IMU 生命周期、零位、双传感器更新和主姿态 freshness 稳定通过后置 1；
 * 该 volatile 标志是无锁快照，其他时间必须视为 0，不能单独替代全部运动安全门。 */
extern volatile uint8_t g_attitude_is_ready;

/**
 * @brief 创建唯一姿态启动安全协调任务；创建失败时请求 MotorBoard 强停。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 无；任务已存在时直接返回；MotorBoard-only 镜像只保持 ready=0，不创建任务。
 * 调用方式：IMU/姿态生命周期任务可运行后由启动路径调用一次。
 * 线程约束：内部调用 xTaskCreate()，失败路径可能排队零 PWM；禁止从 ISR 调用。
 */
void attitude_startup_coordinator_start(void);
/**
 * @brief 监视 IMU/姿态生命周期与 freshness，稳定后启动 MotorBoard，失鲜后重新锁定并强停。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param argument FreeRTOS 任务参数，当前忽略，允许为 NULL。
 * @return 不返回；按 20 ms 周期永久运行。
 * 调用方式：仅由 attitude_startup_coordinator_start() 注册；连续 5 个稳定周期后才置 ready。
 * 线程约束：唯一协调 owner；会查询多个带锁状态、启动 MotorBoard 任务并调用强停，禁止 ISR/并发调用。
 */
void attitude_startup_coordinator_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_STARTUP_COORDINATOR_H */
