#ifndef CHASSIS_STATE_TASK_H
#define CHASSIS_STATE_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* 底盘状态/里程计遥测任务；创建人：待确认（当前维护人：Zhiqin）。
 * 只消费 MotorBoard 轮速和 DualAHRS 航向快照，不参与运动命令或执行器输出。 */

/**
 * @brief 创建唯一的 50 ms 底盘状态发布任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 无；任务已存在时直接返回，创建失败记录错误并保持空 handle。
 * 调用方式：姿态启动协调器确认姿态稳定并成功启动 MotorBoard 后调用。
 * 线程约束：内部调用 xTaskCreate()，仅启动/安全协调任务调用，禁止 ISR 和并发创建。
 */
void chassis_state_task_start(void);
/**
 * @brief 融合最新四轮速度与主 AHRS 航向，更新里程计并发送 CHASSIS_STATE。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param argument FreeRTOS 任务参数，当前忽略，允许 NULL。
 * @return 不返回；任务永久按 50 ms 周期运行。
 * 调用方式：仅由 chassis_state_task_start() 注册；新 wheel sequence 才积分，轮速/姿态
 *           失鲜时使里程计无效并每周期发布无效状态，fresh 但 sequence 未变时不发布。
 * 线程约束：唯一状态 owner；读取原子 wheel snapshot 和无锁 AHRS heading，发送路径可能等待
 *           S3 service mutex/UART，禁止 ISR 或手工并行调用。
 */
void chassis_state_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_STATE_TASK_H */
