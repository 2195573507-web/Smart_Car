#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "chassis_odometry.h"
#include "chassis_state_payload.h"
#include "srp_registry.h"
#include "srp_wire.h"

#define TEST_PI_F 3.14159265358979323846f

/**
 * @brief 断言两个有限浮点值的绝对误差不超过给定容差。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param actual 被测实际值。
 * @param expected 期望值。
 * @param tolerance 允许的非负绝对误差上限；调用点使用固定小正数。
 * @return 返回值：无（void）；误差超限或比较产生非真结果时 assert 终止测试进程。
 * 调用方式：各里程计场景在 update 后同步调用，不修改被测状态。
 * 线程约束：单线程 host 浮点断言，无共享状态、RTOS、ISR 或硬件访问。
 */
static void assert_near(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

/**
 * @brief 验证四轮同速在 0/90 度航向的平面投影、反向位移及绝对累计路程。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；锚定/更新结果或位置、距离容差断言失败时终止测试进程。
 * 调用方式：由 main() 调用；按 50 ms 时间步先前进两段，再把四轮改为 -100 mm/s 验证反向。
 * 线程约束：单线程纯计算测试，state/speeds 为本函数所有；不验证轮序采集、AHRS 时效或车辆运动。
 */
static void test_projection_and_reverse_distance(void)
{
    chassis_odometry_state_t state;
    float speeds[CHASSIS_ODOMETRY_WHEEL_COUNT] = {100.0f, 100.0f, 100.0f,
                                                   100.0f};

    chassis_odometry_init(&state);
    assert(chassis_odometry_update(&state, speeds, 0.0f, 1000U) ==
           CHASSIS_ODOMETRY_RESULT_ANCHORED);
    assert(chassis_odometry_update(&state, speeds, 0.0f, 1050U) ==
           CHASSIS_ODOMETRY_RESULT_UPDATED);
    assert_near(state.x_mm, 5.0f, 0.0001f);
    assert_near(state.y_mm, 0.0f, 0.0001f);

    assert(chassis_odometry_update(&state, speeds, TEST_PI_F / 2.0f, 1100U) ==
           CHASSIS_ODOMETRY_RESULT_UPDATED);
    assert_near(state.x_mm, 5.0f, 0.001f);
    assert_near(state.y_mm, 5.0f, 0.001f);

    for (size_t index = 0U; index < CHASSIS_ODOMETRY_WHEEL_COUNT; ++index) {
        speeds[index] = -100.0f;
    }
    assert(chassis_odometry_update(&state, speeds, 0.0f, 1150U) ==
           CHASSIS_ODOMETRY_RESULT_UPDATED);
    assert_near(state.x_mm, 0.0f, 0.001f);
    assert_near(state.total_distance_m, 0.015f, 0.00001f);
}

/**
 * @brief 验证超过 200 ms 的样本间隔使状态失效，以及恢复样本只重新锚定。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；失效、保留位置、后续积分或重新锚定语义不符时 assert 终止。
 * 调用方式：由 main() 调用；构造 250 ms 间隔，再检查下一条样本不跨失效区间积分。
 * 线程约束：单线程 host 状态机测试，无锁且不覆盖状态任务并发或真实时间源抖动。
 */
static void test_stale_gap_and_reanchor(void)
{
    chassis_odometry_state_t state;
    const float speeds[CHASSIS_ODOMETRY_WHEEL_COUNT] = {100.0f, 100.0f,
                                                        100.0f, 100.0f};

    chassis_odometry_init(&state);
    (void)chassis_odometry_update(&state, speeds, 0.0f, 0U);
    (void)chassis_odometry_update(&state, speeds, 0.0f, 50U);
    assert(chassis_odometry_update(&state, speeds, 0.0f, 300U) ==
           CHASSIS_ODOMETRY_RESULT_INVALID);
    assert(!state.valid);
    assert_near(state.x_mm, 5.0f, 0.0001f);
    assert(chassis_odometry_update(&state, speeds, 0.0f, 350U) ==
           CHASSIS_ODOMETRY_RESULT_ANCHORED);
    assert_near(state.x_mm, 5.0f, 0.0001f);

    assert(chassis_odometry_update(&state, speeds, 0.0f, 340U) ==
           CHASSIS_ODOMETRY_RESULT_INVALID);
    assert(!state.valid);
    assert(chassis_odometry_update(&state, speeds, 0.0f, 390U) ==
           CHASSIS_ODOMETRY_RESULT_ANCHORED);
    assert(chassis_odometry_update(&state, speeds, 0.0f, 440U) ==
           CHASSIS_ODOMETRY_RESULT_UPDATED);
    assert_near(state.x_mm, 10.0f, 0.0001f);

    chassis_odometry_invalidate(&state);
    assert(chassis_odometry_update(&state, speeds, 0.0f, 1000U) ==
           CHASSIS_ODOMETRY_RESULT_ANCHORED);
    assert_near(state.x_mm, 10.0f, 0.0001f);
}

/**
 * @brief 验证左右轮等幅反向时四轮平均速度为零，不产生平移或累计距离。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；更新结果、零平移、零路程或外部 yaw 快照不符时 assert 终止。
 * 调用方式：由 main() 调用；先建立时间 anchor，再用 `[100,100,-100,-100]` 和 yaw=1 rad 更新。
 * 线程约束：单线程纯计算；只验证平均速度模型，不验证轮胎打滑、旋转运动学或 AHRS 物理精度。
 */
static void test_in_place_turn_has_zero_translation(void)
{
    chassis_odometry_state_t state;
    const float speeds[CHASSIS_ODOMETRY_WHEEL_COUNT] = {100.0f, 100.0f,
                                                        -100.0f, -100.0f};

    chassis_odometry_init(&state);
    (void)chassis_odometry_update(&state, speeds, 0.0f, 100U);
    assert(chassis_odometry_update(&state, speeds, 1.0f, 150U) ==
           CHASSIS_ODOMETRY_RESULT_UPDATED);
    assert_near(state.x_mm, 0.0f, 0.0001f);
    assert_near(state.y_mm, 0.0f, 0.0001f);
    assert_near(state.total_distance_m, 0.0f, 0.0001f);
    assert_near(state.yaw_rad, 1.0f, 0.0001f);
}

/**
 * @brief 验证 uint32 毫秒时间戳回绕仍可积分，并拒绝含 NaN 的轮速输入。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；回绕位移、非法结果或 valid 清除语义不符时 assert 终止。
 * 调用方式：由 main() 调用；锚点设在 UINT32_MAX-20，下一样本为 10，再将第三轮速度改为 NAN。
 * 线程约束：单线程 host 浮点/整数边界测试；不模拟硬件计时器采样或并发更新。
 */
static void test_timestamp_wrap_and_nonfinite_rejection(void)
{
    chassis_odometry_state_t state;
    float speeds[CHASSIS_ODOMETRY_WHEEL_COUNT] = {100.0f, 100.0f, 100.0f,
                                                   100.0f};

    chassis_odometry_init(&state);
    (void)chassis_odometry_update(&state, speeds, 0.0f,
                                  UINT32_MAX - UINT32_C(20));
    assert(chassis_odometry_update(&state, speeds, 0.0f, 10U) ==
           CHASSIS_ODOMETRY_RESULT_UPDATED);
    assert_near(state.x_mm, 3.1f, 0.0001f);

    speeds[2] = NAN;
    assert(chassis_odometry_update(&state, speeds, 0.0f, 60U) ==
           CHASSIS_ODOMETRY_RESULT_INVALID);
    assert(!state.valid);
}

/**
 * @brief 验证重复 MotorBoard sequence 不会触发第二次消费，并允许自然回绕。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-09-03（host 覆盖补充）。
 * @return 无；序号消费门语义不符时 assert 终止测试进程。
 */
static void test_wheel_sequence_stall_gate(void)
{
    uint32_t last_sequence = 0U;
    bool have_last_sequence = false;

    assert(chassis_state_sequence_is_new(7U, last_sequence,
                                         have_last_sequence));
    last_sequence = 7U;
    have_last_sequence = true;
    assert(!chassis_state_sequence_is_new(7U, last_sequence,
                                          have_last_sequence));
    assert(chassis_state_sequence_is_new(8U, last_sequence,
                                         have_last_sequence));
    assert(chassis_state_sequence_is_new(0U, UINT32_MAX, true));
}

/**
 * @brief 验证 CHASSIS_STATE 24 字节布局、有效位掩码、保留字段和有限值拒绝。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-09-03（host 覆盖补充）。
 * @return 无；字段、位或非法状态不符时 assert 终止测试进程。
 */
static void test_chassis_state_payload_validity(void)
{
    chassis_odometry_state_t state = {
        .x_mm = 1000.0f,
        .y_mm = -500.0f,
        .yaw_rad = TEST_PI_F,
        .total_distance_m = 12.5f,
        .last_sample_timestamp_ms = 0U,
        .has_time_anchor = true,
        .valid = true,
    };
    uint8_t payload[SRP_PAYLOAD_CHASSIS_STATE_SIZE];
    uint8_t flags = (uint8_t)(SRP_CHASSIS_STATE_FLAG_ODOMETRY_VALID | 0x80U);

    (void)memset(payload, 0xA5, sizeof(payload));
    assert(chassis_state_pack_payload(payload, sizeof(payload), &state, flags,
                                      1000U));
    assert(payload[0] == SRP_CHASSIS_STATE_SCHEMA);
    assert(payload[1] == SRP_CHASSIS_STATE_FLAG_ODOMETRY_VALID);
    assert(payload[2] == 0U && payload[3] == 0U);
    assert(srp_wire_read_u32_le(&payload[4]) == 1000U);
    assert(srp_wire_read_f32_le(&payload[8]) == 1000.0f);
    assert(srp_wire_read_f32_le(&payload[12]) == -500.0f);
    assert_near(srp_wire_read_f32_le(&payload[16]), 180.0f, 0.0001f);
    assert(srp_wire_read_f32_le(&payload[20]) == 12.5f);
    assert(chassis_state_pack_payload(payload, sizeof(payload), &state, 0U,
                                      0U));
    assert(payload[1] == 0U);

    state.total_distance_m = -1.0f;
    assert(!chassis_state_pack_payload(payload, sizeof(payload), &state, 0U,
                                       0U));
}

/**
 * @brief 顺序执行底盘里程计投影、失鲜、原地旋转、回绕和非有限输入测试。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 全部断言通过返回 0；任一 assert 失败会终止测试进程。
 * 调用方式：由 Odometry/tests/run_host_tests.sh 编译并直接执行，无外部夹具。
 * 线程约束：单进程单线程 host 测试；未运行 CM7 RTOS、传感器、轮速链路或车辆。
 */
int main(void)
{
    test_projection_and_reverse_distance();
    test_stale_gap_and_reanchor();
    test_in_place_turn_has_zero_translation();
    test_timestamp_wrap_and_nonfinite_rejection();
    test_wheel_sequence_stall_gate();
    test_chassis_state_payload_validity();
    return 0;
}
