#ifndef IMU_RUNTIME_H
#define IMU_RUNTIME_H

#include <stdint.h>

/* CM7 IMU 运行任务接口；创建人：待确认（当前维护人：Zhiqin）。 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 IMU 运行链并创建采样任务与低频诊断/遥测任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 无返回值；初始化和任务创建失败通过启动日志及模块状态暴露。
 * @note 调用方式与线程约束：CM7 启动流程在调度器启动前后按既有顺序调用一次；内部会执行
 *       传感器初始化、日志和 `xTaskCreate()`，可能阻塞，禁止从 ISR 或重复调用。
 */
void imu_runtime_start(void);
/**
 * @brief 低频输出 IMU 状态、资源水位、DualAHRS 日志并发送遥测。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] argument FreeRTOS 任务参数，当前实现不解引用，通常传 NULL。
 * @return 无返回值；这是永久 FreeRTOS 任务入口。
 * @note 调用方式与线程约束：仅由 `imu_runtime_start()` 通过 `xTaskCreate()` 创建；函数包含
 *       周期延时和日志/发送操作，禁止直接调用、并发创建或从 ISR 调用。
 */
void imu_debug_task(void *argument);
/**
 * @brief 读取 IMU runtime 记录的日志写入失败累计值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 自启动以来的累计失败次数；读取不会清零，计数回绕按 `uint32_t` 处理。
 * @note 调用方式与线程约束：诊断任务低频读取；不阻塞、不拥有外部资源，该计数不证明 UART
 *       或 BLE 已实际接收日志。
 */
uint32_t imu_runtime_get_log_fail_count(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_RUNTIME_H */
