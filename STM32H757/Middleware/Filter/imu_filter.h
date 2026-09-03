#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include <stdint.h>

#include "imu_calibration.h"

/*
 * IMU 滤波层。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 契约补充日期：2026-08-31。
 * 只消费已标定快照并产出只读结果，不负责传感器初始化或姿态安全判定。
 * 本模块不检查 online 或浮点有限性；有效性/新鲜度门控由调用方完成。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 三轴加速度/磁场独立使用的中值窗口长度。 */
#define IMU_FILTER_MEDIAN_WINDOW 5U
/* 上一次滤波输出的权重；当前新中值权重为 1-IMU_FILTER_ALPHA。 */
#define IMU_FILTER_ALPHA 0.95f

/** LSM303 已标定加速度与磁场经过中值/低通后的输出快照。 */
typedef struct
{
    /** 车体坐标系加速度，单位 m/s^2。 */
    float ax; /**< X 轴加速度，单位 m/s^2。 */
    float ay; /**< Y 轴加速度，单位 m/s^2。 */
    float az; /**< Z 轴加速度，单位 m/s^2。 */
    /** 车体坐标系磁场强度，单位 uT。 */
    float mx; /**< X 轴磁场，单位 uT。 */
    float my; /**< Y 轴磁场，单位 uT。 */
    float mz; /**< Z 轴磁场，单位 uT。 */
    /** 来源快照的毫秒时间戳。 */
    uint32_t timestamp;
    /** 来源快照的在线标志；本模块只复制，不重新判定。 */
    uint8_t online;
} imu_filtered_data_t;

/* Compatibility name for existing runtime consumers. */
typedef imu_filtered_data_t imu_filter_output_t;

/**
 * @brief 创建必要的互斥资源，清空中值/低通历史并置为未就绪。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 无。当前接口不上报互斥量创建失败。
 * 调用方式：在 IMU 生命周期开始、标定重启或滤波重锚时调用。
 * 线程约束：RTOS 构建下可创建/获取互斥量，只允许启动上下文或普通任务调用，禁止 ISR 调用。
 */
void imu_filter_init(void);

/**
 * @brief 消费一组已标定快照，更新五点中值窗和一阶低通输出。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param calibrated_data 已完成标定和坐标变换的快照；只在调用期间读取。
 *        NULL 时不更新；调用方必须在调用前验证 online、新鲜度和浮点有限性。
 * @return 无。
 * 调用方式：imu_manager 在双 IMU 启动/标定门已通过且有新 LSM303 样本时调用。
 * 线程约束：RTOS 构建下会以 portMAX_DELAY 等待互斥量，只允许普通任务上下文，禁止 ISR 调用。
 */
void imu_filter_update(const imu_calibrated_data_t *calibrated_data);

/**
 * @brief 按值复制最新滤波快照，不暴露内部存储。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 最新 imu_filtered_data_t 快照；首个非 NULL 更新前为初始零值/未在线状态。
 * 调用方式：姿态层和调试任务在普通任务上下文读取；调用方仍需检查 online/时间戳。
 * 线程约束：RTOS 构建下可阻塞等待互斥量，禁止 ISR 调用。
 */
imu_filtered_data_t imu_filter_get_output(void);

/**
 * @brief 查询滤波器是否已接受至少一个非 NULL 样本。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 1 表示已建立滤波历史，0 表示尚未更新；该值不等价于 output.online。
 * 调用方式：姿态启动门在普通任务上下文读取。
 * 线程约束：RTOS 构建下可等待互斥量，禁止 ISR 调用。
 */
uint8_t imu_filter_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FILTER_H */
