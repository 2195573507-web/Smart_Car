#ifndef S3_RADAR_TELEMETRY_AGE_H
#define S3_RADAR_TELEMETRY_AGE_H

#include <stdbool.h>
#include <stdint.h>

/* S3 telemetry 时效判定纯函数；创建人：待确认（当前维护人：Zhiqin）。 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 使用同源 uint32_t 单调毫秒计数判断一条 telemetry 是否超过允许时效。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31。
 * @param now_ms consumer 当前单调时间，必须与 ingress_timestamp_ms 来自同一时钟和 epoch。
 * @param ingress_timestamp_ms producer 入队时的单调时间，单位 ms。
 * @param max_age_ms 最大允许年龄；等于该值仍有效，严格大于才判 stale。
 * @return 超过 max_age_ms 返回 true，否则返回 false。
 * 调用方式：S3RD type-2 准备路径在消费队列项后、编码前调用；host test 直接传入合成计数。
 * 失败语义：函数不检查跨 epoch 或超过一个完整 uint32 周期的输入，调用方必须保证同源时钟。
 * 线程约束：header-only 纯计算、无共享状态、无阻塞、可重入，可在任务或 ISR 计算路径使用。
 */
static inline bool radar_telemetry_age_is_stale(uint32_t now_ms,
                                                 uint32_t ingress_timestamp_ms,
                                                 uint32_t max_age_ms)
{
    return (uint32_t)(now_ms - ingress_timestamp_ms) > max_age_ms;
}

#ifdef __cplusplus
}
#endif

#endif
