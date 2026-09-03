#include "chassis_state_task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal.h"

#include "attitude_startup_coordinator.h"
#include "chassis_odometry.h"
#include "dual_ahrs.h"
#include "log_service.h"
#include "motor_board_task.h"
#include "s3_service.h"
#include "srp_registry.h"
#include "srp_wire.h"

/* 底盘状态与里程计发布实现；创建人：待确认（当前维护人：Zhiqin）。 */

#define CHASSIS_STATE_TASK_STACK_WORDS UINT16_C(256)
#define CHASSIS_STATE_TASK_PRIORITY (tskIDLE_PRIORITY + 1U)
#define CHASSIS_STATE_TASK_PERIOD_MS UINT32_C(50)
#define CHASSIS_STATE_RAD_TO_DEG 57.2957795130823208768f

static TaskHandle_t s_task_handle;

/**
 * @brief 将里程计快照、状态标志和采样时间显式编码为固定长度底盘状态 payload。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] payload 调用方拥有且恰可写 `SRP_PAYLOAD_CHASSIS_STATE_SIZE` 字节的缓冲区；
 *                     函数先清零再按小端布局写入，不可为 NULL。
 * @param[in] state 调用期间只读的里程计状态；不可为 NULL，yaw 从 rad 转换为 deg 写出。
 * @param[in] flags 待发布状态位，仅保留 `SRP_CHASSIS_STATE_FLAGS_MASK` 范围。
 * @param[in] timestamp_ms 轮速快照时间戳，单位 ms；无有效轮速时上层传 0。
 * @return 无返回值；有效参数时写满 payload，NULL/容量不足没有防御性失败输出。
 * 调用方式：仅 `chassis_state_task()` 在里程计更新或失效事件需要发布时同步调用。
 * 线程约束：纯内存序列化、不阻塞、不获取 mutex，禁止 ISR 调用；不保存任何指针，
 *           源状态和目标缓冲区所有权均归调用方且调用期间不得并发修改。
 */
static void chassis_state_pack_payload(
    uint8_t payload[SRP_PAYLOAD_CHASSIS_STATE_SIZE],
    const chassis_odometry_state_t *state,
    uint8_t flags,
    uint32_t timestamp_ms)
{
    (void)memset(payload, 0, SRP_PAYLOAD_CHASSIS_STATE_SIZE);
    payload[0] = SRP_CHASSIS_STATE_SCHEMA;
    payload[1] = flags & SRP_CHASSIS_STATE_FLAGS_MASK;
    srp_wire_write_u32_le(&payload[4], timestamp_ms);
    srp_wire_write_f32_le(&payload[8], state->x_mm);
    srp_wire_write_f32_le(&payload[12], state->y_mm);
    srp_wire_write_f32_le(&payload[16],
                          state->yaw_rad * CHASSIS_STATE_RAD_TO_DEG);
    srp_wire_write_f32_le(&payload[20], state->total_distance_m);
}

void chassis_state_task(void *argument)
{
    TickType_t last_wake;
    chassis_odometry_state_t odometry;
    uint32_t last_processed_sequence = 0U;
    bool have_processed_sequence = false;

    (void)argument;
    chassis_odometry_init(&odometry);
    last_wake = xTaskGetTickCount();

    for (;;) {
        motor_board_wheel_speed_snapshot_t wheel = {0};
        uint8_t payload[SRP_PAYLOAD_CHASSIS_STATE_SIZE];
        uint8_t flags = 0U;
        float primary_yaw_rad = odometry.yaw_rad;
        float gyro_z_rad_s = 0.0f;
        const uint32_t now_ms = HAL_GetTick();
        const bool have_wheel =
            motor_board_get_actual_wheel_speed_snapshot(&wheel);
        const bool wheel_fresh =
            have_wheel &&
            (uint32_t)(now_ms - wheel.timestamp_ms) <=
                CHASSIS_ODOMETRY_MAX_SAMPLE_INTERVAL_MS;
        const bool attitude_fresh =
            g_attitude_is_ready != 0U &&
            dual_ahrs_get_heading_state(&primary_yaw_rad,
                                        &gyro_z_rad_s) != 0U;
        bool publish = false;

        (void)gyro_z_rad_s;
        if (attitude_fresh) {
            flags |= SRP_CHASSIS_STATE_FLAG_ATTITUDE_READY;
        }

        if (wheel_fresh && attitude_fresh &&
            (!have_processed_sequence ||
             wheel.sequence != last_processed_sequence)) {
            const chassis_odometry_result_t result = chassis_odometry_update(
                &odometry, wheel.speed_mm_s, primary_yaw_rad,
                wheel.timestamp_ms);

            last_processed_sequence = wheel.sequence;
            have_processed_sequence = true;
            if (result != CHASSIS_ODOMETRY_RESULT_INVALID && odometry.valid) {
                flags |= SRP_CHASSIS_STATE_FLAG_ODOMETRY_VALID;
            }
            publish = true;
        } else if (!wheel_fresh || !attitude_fresh) {
            chassis_odometry_invalidate(&odometry);
            have_processed_sequence = false;
            publish = true;
        }

        if (publish) {
            chassis_state_pack_payload(payload, &odometry, flags,
                                       have_wheel ? wheel.timestamp_ms : 0U);
            s3_service_send_chassis_state(payload, sizeof(payload));
        }
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(CHASSIS_STATE_TASK_PERIOD_MS));
    }
}

void chassis_state_task_start(void)
{
    if (s_task_handle != NULL) {
        return;
    }
    if (xTaskCreate(chassis_state_task, "chassis_state",
                    CHASSIS_STATE_TASK_STACK_WORDS, NULL,
                    CHASSIS_STATE_TASK_PRIORITY, &s_task_handle) != pdPASS) {
        s_task_handle = NULL;
        LOG_ERROR("[CHASSIS_STATE] task create failed");
    } else {
        LOG_INFO("[CHASSIS_STATE] task started");
    }
}
