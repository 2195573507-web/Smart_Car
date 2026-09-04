#ifndef CHASSIS_ODOMETRY_H
#define CHASSIS_ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 平面底盘里程计纯计算接口；创建人：待确认（当前维护人：Zhiqin）。
 * 轮速顺序沿用 [RR, RF, LR, LF]，航向由外部主 AHRS 提供，本模块不自行积分角速度。 */

#define CHASSIS_ODOMETRY_WHEEL_COUNT 4U
#define CHASSIS_ODOMETRY_MAX_SAMPLE_INTERVAL_MS UINT32_C(200)

/** 单次里程计输入的处理结果。 */
typedef enum {
    CHASSIS_ODOMETRY_RESULT_INVALID = 0, /**< 输入、状态、计算或采样间隔无效。 */
    CHASSIS_ODOMETRY_RESULT_ANCHORED, /**< 仅建立时间/航向基线，本次不积分位移。 */
    CHASSIS_ODOMETRY_RESULT_UPDATED /**< 已完成一次有效平面位移积分。 */
} chassis_odometry_result_t;

/** 状态任务单 owner 持有的平面里程计累计状态。 */
typedef struct {
    float x_mm; /**< 世界平面 X 累计位置，单位 mm。 */
    float y_mm; /**< 世界平面 Y 累计位置，单位 mm。 */
    float yaw_rad; /**< 最近主 AHRS 航向，单位 rad。 */
    float total_distance_m; /**< 按位移绝对值累计的路程，单位 m。 */
    uint32_t last_sample_timestamp_ms; /**< 最近消费轮速的 CM7 单调时间戳，单位 ms。 */
    bool has_time_anchor; /**< 已有可用于下一次差分的时间基线。 */
    bool valid; /**< 当前累计状态是否可作为有效 odometry 发布。 */
} chassis_odometry_state_t;

/**
 * @brief 把里程计位置、航向、累计距离、时间 anchor 和有效位全部清零。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param state 调用方拥有的可写状态；NULL 时不动作。
 * @return 无。
 * 调用方式：状态任务启动或检测到内部累计状态非有限/负距离时调用。
 * 线程约束：纯内存写入、不加锁；同一 state 必须由单 owner 串行访问，禁止与读取并发。
 */
void chassis_odometry_init(chassis_odometry_state_t *state);
/**
 * @brief 清除时间 anchor 和 valid 位，保留累计位置、航向、距离及上次时间戳。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param state 调用方拥有的可写状态；NULL 时不动作。
 * @return 无。
 * 调用方式：轮速/姿态失鲜或输入/计算无效时调用；下一次 update 会先重新 anchor 而不积分。
 * 线程约束：纯内存写入、不加锁；同一 state 必须由状态任务单 owner 串行访问。
 */
void chassis_odometry_invalidate(chassis_odometry_state_t *state);
/**
 * @brief 用四轮平均线速度和外部主航向对平面位置与绝对路程做一次欧拉积分。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param state 可写里程计状态；NULL 返回 INVALID。
 * @param wheel_speed_mm_s 四个有限轮速，单位 mm/s，顺序 [RR,RF,LR,LF]；不得为 NULL。
 * @param primary_yaw_rad 主 AHRS 航向，单位 rad，必须有限；本模块直接用于本周期 cos/sin。
 * @param sample_timestamp_ms 轮速到达的 CM7 单调毫秒时间戳，按 uint32 无符号差值处理回绕。
 * @return 首次/失效后建立时间基线返回 ANCHORED；有效 1..200 ms 间隔积分返回 UPDATED；
 *         参数、状态或间隔无效返回 INVALID。间隔为 0/过大时清除时间 anchor/valid，
 *         下一条样本只能重新锚定；其他输入/计算失败通常也清 anchor。
 * 调用方式：状态任务只对新的 wheel sequence 调用；返回非 UPDATED 时不得宣称产生位移增量。
 * 线程约束：纯 float/math 状态更新、不加锁；同一 state 单 owner，禁止 ISR 或并发读写。
 */
chassis_odometry_result_t chassis_odometry_update(
    chassis_odometry_state_t *state,
    const float wheel_speed_mm_s[CHASSIS_ODOMETRY_WHEEL_COUNT],
    float primary_yaw_rad,
    uint32_t sample_timestamp_ms);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_ODOMETRY_H */
