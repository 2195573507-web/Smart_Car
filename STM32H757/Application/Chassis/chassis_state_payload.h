#ifndef CHASSIS_STATE_PAYLOAD_H
#define CHASSIS_STATE_PAYLOAD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "chassis_odometry.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CHASSIS_STATE 纯 payload/序号门工具；创建人：待确认（当前维护人：Zhiqin）。 */

/**
 * @brief 判断一次 MotorBoard wheel sequence 是否需要被状态任务消费。
 * @param sequence 当前快照序号。
 * @param last_sequence 上一次已消费序号。
 * @param have_last_sequence 是否已经建立消费基线。
 * @return 首次消费或序号变化返回 true；序号停滞返回 false。
 * @note 序号由单一 MotorBoard producer 自然递增并允许 uint32 回绕；这里只拒绝
 *       完全相同的重复快照，回绕由 producer 的 32 位自然运算保留。
 */
bool chassis_state_sequence_is_new(uint32_t sequence,
                                   uint32_t last_sequence,
                                   bool have_last_sequence);

/**
 * @brief 将里程计状态编码为固定 24 字节 SRP CHASSIS_STATE payload。
 * @param[out] payload 输出缓冲区。
 * @param[in] capacity 输出缓冲区容量，至少为 SRP_PAYLOAD_CHASSIS_STATE_SIZE。
 * @param[in] state 待读取的有限里程计状态。
 * @param[in] flags 状态位；未知高位会被清除。
 * @param[in] timestamp_ms 轮速源样本的 CM7 单调毫秒时间戳。
 * @return 参数和状态合法且完成编码时返回 true，否则返回 false。
 */
bool chassis_state_pack_payload(
    uint8_t *payload,
    size_t capacity,
    const chassis_odometry_state_t *state,
    uint8_t flags,
    uint32_t timestamp_ms);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_STATE_PAYLOAD_H */
