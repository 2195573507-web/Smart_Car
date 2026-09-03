#ifndef MAG_FILTER_H
#define MAG_FILTER_H

#include <stdbool.h>

#include "imu_manager.h"

/* 磁力计滤波接口；创建人：待确认（当前维护人：Zhiqin）。
 * 契约补充日期：2026-08-31。
 * 只消费已成功读取的 LSM303 快照并产出滤波结果，不负责总线访问、
 * 样本有效性检查或姿态安全判定。 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * LSM303 磁场低通滤波输出；模块内部保存一份状态，getter 按值复制给调用方。
 */
typedef struct
{
    float mx; /**< 车体坐标系 X 轴滤波磁场强度，单位 uT。 */
    float my; /**< 车体坐标系 Y 轴滤波磁场强度，单位 uT。 */
    float mz; /**< 车体坐标系 Z 轴滤波磁场强度，单位 uT。 */
} mag_filter_data_t;

/**
 * @brief 清空磁力计低通滤波历史并置为未初始化状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 无。
 * @note 调用方式与线程约束：IMU 初始化或恢复路径在首个磁场样本前调用；FreeRTOS 构建中
 *       使用短临界区，不阻塞等待资源，禁止从 ISR 调用。
 */
void mag_filter_init(void);
/**
 * @brief 消费一条 LSM303 磁场快照并更新一阶低通输出。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] raw 调用方拥有的只读磁场快照，单位 uT；NULL 时不更新。当前实现
 *                不校验 NaN/有效位，调用方必须先完成数据有效性检查。
 * @return 无。
 * @note 调用方式与线程约束：由 IMU manager 在成功读取新磁场样本后调用；函数在短临界区
 *       内复制数值，不保存指针、不阻塞，禁止从 ISR 或多个 writer 并发调用。
 */
void mag_filter_update(const lsm_mag_data_t *raw);
/**
 * @brief 复制最近一次已初始化的磁场滤波结果。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] out 调用方拥有的输出对象，单位 uT，不允许为 NULL。
 * @return true 表示 `out` 已写入有效滤波值；false 表示参数为 NULL 或尚无样本，
 *         false 时非 NULL 输出保持原值。
 * @note 调用方式与线程约束：任务上下文读取；FreeRTOS 构建中使用短临界区，不保存输出指针，
 *       不阻塞等待资源，禁止从 ISR 调用。
 */
bool mag_filter_get(mag_filter_data_t *out);

#ifdef __cplusplus
}
#endif

#endif /* MAG_FILTER_H */
