#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include <stdbool.h>

/*
 * CM7 底盘控制任务公共接口。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 所有输出仍受同步、姿态、急停、BUS_OFF 和 MotorBoard 就绪门控约束。
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建并启动唯一的 10 ms 底盘控制任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 无；任务已存在时直接返回，创建失败只记录错误日志并保持空 handle。
 * 调用方式：完成 RTOS 基础资源和相关安全服务初始化后，由启动路径调用一次。
 * 线程约束：内部调用 xTaskCreate()，仅普通启动任务调用，禁止从 ISR 调用。
 */
void chassis_task_start(void);
/**
 * @brief 周期执行安全门、线速度斜坡、航向控制、运动学和 MotorBoard 目标提交。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param argument FreeRTOS 任务参数，当前忽略，允许为 NULL。
 * @return 不返回；任务永久按 CHASSIS_TASK_PERIOD_MS 周期运行。
 * 调用方式：只由 chassis_task_start() 注册为 FreeRTOS 任务入口，不得直接调用。
 * 线程约束：唯一底盘控制 owner；会读取姿态/同步门并调用 MotorBoard 任务接口，禁止 ISR 调用。
 */
void chassis_task(void *argument);

/**
 * @brief 提交差速底盘速度目标。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param v_mm_s 车体前向线速度，单位 mm/s，必须有限且运动学结果不超单轮上限。
 * @param w_rad_s 绕 Z 轴角速度，单位 rad/s，正值左转，必须有限。
 * @return true 表示目标已复制到本地状态；false 表示参数/运动学无效，或非零命令的
 *         SRP 同步、IMU、姿态 freshness、ready 标志或 MotorBoard 任务门未打开。
 * 调用方式：服务任务或受控应用任务调用；不得绕过本接口直接写 MotorBoard。
 * 线程约束：使用 FreeRTOS 临界区更新目标，禁止从 ISR 调用；成功不代表轮速已发送或执行。
 */
bool chassis_task_set_velocity(float v_mm_s, float w_rad_s);

/**
 * @brief 原子清除速度/航向/前馈/斜坡状态并递增本地停机序号。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 无。
 * 调用方式：失联、急停和故障路径调用；需要立即排队零 PWM 时，外层安全路径还必须调用
 *           motor_board_force_stop()，本函数本身不访问 USART6 或 MotorBoard TX ring。
 * 线程约束：仅使用短 FreeRTOS 临界区，可与控制任务串行化状态；禁止从 ISR 调用。
 */
void chassis_task_force_stop(void);

/**
 * @brief 提交线速度加绝对目标航向的巡航目标，不直接写 MotorBoard。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param v_mm_s 线速度，单位 mm/s；绝对值不得超过单轮速度上限。
 * @param target_yaw_deg 目标航向，单位 deg，输入范围 -180..180；180 会规范化为 -180。
 * @return 无；不满足 admissible 条件时静默保持原目标；绝对值不超过 0.001 mm/s 的
 *         已准入命令会转为 chassis_task_force_stop()。
 * 调用方式：协议服务先调用 chassis_task_heading_target_is_admissible() 获取可确认结果，
 *           再调用本函数；重复刷新不会重置已有航向积分和角速度 slew 历史。
 * 线程约束：以短 FreeRTOS 临界区更新目标，禁止从 ISR 调用；最终输出仍受实时安全门。
 */
void chassis_task_set_heading_target(float v_mm_s, float target_yaw_deg);
/**
 * @brief 设置目标航向控制的角速度前馈，传入 0 恢复无前馈。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param w_ff_rad_s 角速度前馈，单位 rad/s，必须有限且绝对值不超过 2.0。
 * @return 无；非法值静默保持原配置。
 * 调用方式：受控任务在航向命令前后按需要更新；停机路径会把前馈清零。
 * 线程约束：短临界区写入，不检查安全门且不直接产生输出；禁止从 ISR 调用。
 */
void chassis_task_set_heading_feedforward(float w_ff_rad_s);

/**
 * @brief 仅检查航向命令是否满足有限值、角度范围和安全 flags 要求。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param v_mm_s 线速度，单位 mm/s；必须有限且绝对值不超过单轮速度上限。
 * @param target_yaw_deg 目标航向，单位 deg，必须有限且位于 -180..180。
 * 返回值：true 表示数值有效，且 v_mm_s 精确为 0 或当前完整输出门已打开；false 应拒绝。
 * 调用方式：协议服务和主机测试可调用；本函数不修改目标状态。
 * 线程约束：普通任务调用；会读取同步/IMU/姿态/MotorBoard 状态，不提供跨状态原子快照，禁止 ISR。
 */
bool chassis_task_heading_target_is_admissible(float v_mm_s,
                                               float target_yaw_deg);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_TASK_H */
