#include "radar_calibration_manager.h"

/* S3 雷达标定握手实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "radar_control.h"
#include "s3_ble.h"

static const char *TAG = "RADAR_CAL";

#define RADAR_CAL_STATIC_EVENT_TIMEOUT_MS UINT32_C(75000)
#define STM_BOOT_STATE_WAIT_RADAR_ZERO UINT8_C(1)

static radar_calibration_state_t s_state;
static uint8_t s_pwm;
static bool s_initialized;
static bool s_done;
static bool s_stm_boot_ready;
static uint64_t s_static_event_deadline_us;
static radar_calibration_send_ready_t s_send_ready;
static void *s_transport_context;

/**
 * @brief 读取 ESP 单调微秒计时，用于标定事务截止时间计算。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return esp_timer_get_time() 的非负微秒值，单调性由 ESP-IDF 定时器实现保证。
 * 调用方式：仅由本文件状态机设置或比较 RADAR_CAL_STATIC_EVENT_TIMEOUT_MS 截止时间时调用。
 * 线程约束：不访问本模块可变状态；当前仅服务任务调用，禁止将其返回值解释为墙上时间或硬件响应证据。
 */
static uint64_t now_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

/**
 * @brief 清除标定握手进度并回到 RADAR_WAIT_SYNC，不改写 transport 注册信息。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）。
 * 调用方式：由初始化和失败回退路径调用；只重置 manager 静态状态，不直接操作 LEDC/PWM。
 * 线程约束：无锁静态写入，仅允许 smartcar_service 单任务 owner，禁止 ISR 或与 step/事件处理并发调用。
 */
static void reset_tracking(void)
{
    s_state = RADAR_WAIT_SYNC;
    s_pwm = 0U;
    s_done = false;
    s_stm_boot_ready = false;
    s_static_event_deadline_us = 0U;
}

/**
 * @brief 将标定流程回退到同步等待，并尽力恢复雷达零 PWM/释放标定覆盖。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param reason 只读失败原因字符串，仅在本次调用日志中使用；允许 NULL，NULL 时记录 unspecified。
 * @return 返回值：无（void）；控制锁或 LEDC 更新失败只由下层返回/日志体现，manager 仍保留 WAIT_SYNC 状态。
 * 调用方式：由 step() 的提交/超时失败及事务完成回调失败路径同步调用。
 * 线程约束：仅服务任务调用；会多次获取 radar_control mutex（每次最多 20 ms）并调用日志，禁止 ISR、GATT 回调和并发调用。
 */
static void enter_sync_wait(const char *reason)
{
    const radar_calibration_state_t from_state = s_state;

    reset_tracking();
    if (radar_control_get_state() != RADAR_CONTROL_WAIT_STM_QUERY) {
        radar_control_set_imu_cal_done(false);
        (void)radar_control_set_calibration_pwm(0U);
    }
    radar_control_release_calibration_lock();
    ESP_LOGW(TAG, "RADAR_WAIT_SYNC from=%u reason=%s", (unsigned)from_state,
             reason != NULL ? reason : "unspecified");
    (void)s3_log_info("RADAR CAL WAIT_SYNC");
}

/**
 * @brief 应用 STM-owned 零 PWM，并提交一笔 RADAR_PWM_READY SRP 事务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return true 表示本地零 PWM 已提交且 transport 接受事务；缺少回调、控制门失败或 transport 非零返回时为 false。
 * 调用方式：仅由 step() 在 RADAR_SET_PWM 状态调用；失败后由调用方执行 enter_sync_wait()。
 * 线程约束：服务任务单 owner；会等待 radar_control mutex，并同步调用借用的 transport/context，不保留新指针且禁止 ISR/并发调用。
 */
static bool start_zero_pwm(void)
{
    if (s_send_ready == NULL || !radar_control_set_calibration_pwm(0U)) {
        return false;
    }
    s_pwm = 0U;
    s_state = RADAR_WAIT_ACK;
    s_static_event_deadline_us = now_us() +
        ((uint64_t)RADAR_CAL_STATIC_EVENT_TIMEOUT_MS * UINT64_C(1000));
    if (s_send_ready(s_pwm, s_transport_context) != 0) {
        return false;
    }
    ESP_LOGI(TAG, "RADAR_PWM_READY TX speed=0");
    return true;
}

/**
 * @brief 清空标定跟踪状态并进入 RADAR_WAIT_SYNC。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）。
 * 调用方式：radar_control_init() 后、服务循环和所有 manager 事件前调用；重复调用会重置进度，但不会清除已注册 transport 或主动改变 PWM。
 * 线程约束：仅启动/服务任务调用；模块使用无锁静态状态，不可并发或从 ISR/GATT 回调调用。
 */
void radar_calibration_manager_init(void)
{
    reset_tracking();
    s_initialized = true;
    (void)s3_log_info("RADAR CAL WAIT STM_BOOT_READY");
}

/**
 * @brief 注册 RADAR_PWM_READY 事务提交回调及其上下文。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param send_ready 回调；可传 NULL 取消 transport，此时下一次提交将失败并回到同步等待。
 * @param context 原样传给 send_ready，不转移所有权；生命周期必须覆盖注册使用期，允许 NULL。
 * @return 返回值：无（void）。
 * 调用方式：初始化后、收到 BOOT_READY 前注册；不要在事务等待期间替换回调。
 * 线程约束：仅服务任务调用；无锁写入，不可与 step/事件处理并发，也不得从 ISR/GATT 回调调用。
 */
void radar_calibration_manager_set_transport(radar_calibration_send_ready_t send_ready,
                                             void *context)
{
    s_send_ready = send_ready;
    s_transport_context = context;
}

/**
 * @brief 推进一次 PWM READY 提交或静态标定超时处理。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；未初始化、未收到 BOOT_READY 或已完成时不动作。
 * 调用方式：由 smartcar_service 主循环周期调用；可能同步调用 transport 和 radar_control 接口。
 * 线程约束：仅服务任务；可能等待 radar_control mutex 并产生日志，禁止 ISR/GATT 回调和并发调用。
 */
void radar_calibration_manager_step(void)
{
    if (!s_initialized || s_done || !s_stm_boot_ready) {
        return;
    }
    if (s_state == RADAR_SET_PWM) {
        if (!start_zero_pwm()) {
            enter_sync_wait("RADAR_PWM_READY send failed");
        }
        return;
    }
    if ((s_state == RADAR_WAIT_ACK || s_state == RADAR_WAIT_STATIC_EVENT) &&
        now_us() >= s_static_event_deadline_us) {
        enter_sync_wait("STATIC_CAL_DONE timeout");
    }
}

/**
 * @brief 校验并消费 STM 的 BOOT_READY 标定门事件。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param payload 非 NULL、长度为 SRP_PAYLOAD_BOOT_READY_SIZE 的借用 payload。
 * @param length payload 字节数；状态必须为 WAIT_RADAR_ZERO 且保留字节为 0。
 * @return true 表示首次事件或合法重传被接受；未初始化、状态、长度或内容不符返回 false。
 * 调用方式：仅在 SRP 帧 CRC、类型和会话已由 command_bridge 校验后调用；函数不保留 payload。
 * 线程约束：服务任务上下文；会调用 radar_control 并可能等待 mutex，禁止 ISR、GATT 回调或并发调用。
 */
bool radar_calibration_manager_on_boot_ready(const uint8_t *payload, uint8_t length)
{
    if (!s_initialized || payload == NULL ||
        length != SRP_PAYLOAD_BOOT_READY_SIZE ||
        payload[0] != STM_BOOT_STATE_WAIT_RADAR_ZERO || payload[1] != 0U) {
        return false;
    }
    if (s_stm_boot_ready) {
        /* BOOT_READY is retransmitted until its transport ACK arrives. */
        return true;
    }
    if (s_state != RADAR_WAIT_SYNC) {
        return false;
    }
    s_stm_boot_ready = true;
    radar_control_handle_pwm_ready_query();
    s_state = RADAR_SET_PWM;
    (void)s3_log_info("RADAR CAL START");
    return true;
}

/**
 * @brief 消费 STM 的 STATIC_CAL_DONE 事件并尝试释放雷达运行门。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param payload 非 NULL、长度为 SRP_PAYLOAD_CAL_EVENT_SIZE 的借用 payload。
 * @param length payload 字节数；事件 ID 必须为 SRP_CAL_EVENT_STATIC_DONE。
 * @return true 表示完成事件被接受或为完成后的合法重传；乱序、内容错误或控制门失败返回 false。
 * 调用方式：仅在已校验 SRP 事件路径调用；函数不保留 payload，成功后 manager 进入 RADAR_CAL_DONE。
 * 线程约束：服务任务上下文；会操作 radar_control mutex/LEDC，禁止 ISR、GATT 回调和并发调用。
 */
bool radar_calibration_manager_on_cal_event(const uint8_t *payload, uint8_t length)
{
    if (!s_initialized || payload == NULL ||
        length != SRP_PAYLOAD_CAL_EVENT_SIZE ||
        payload[0] != SRP_CAL_EVENT_STATIC_DONE) {
        return false;
    }
    if (s_done) {
        return true;
    }
    if (s_state != RADAR_WAIT_STATIC_EVENT || s_pwm != 0U) {
        return false;
    }
    radar_control_release_calibration_lock();
    radar_control_set_imu_cal_done(true);
    if (!radar_control_is_running()) {
        enter_sync_wait("radar control completion failed");
        return false;
    }
    s_state = RADAR_CAL_DONE;
    s_done = true;
    (void)s3_log_info("STATIC_CAL_DONE accepted; radar control released");
    return true;
}

/**
 * @brief 处理 RADAR_PWM_READY 事务的 ACK、拒绝或超时结果。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param result SRP 链路事务结果。
 * @param status_code 快速响应状态；仅 SRP_LINK_TX_OK 加 SRP_FAST_RESP_OK 进入 WAIT_STATIC_EVENT。
 * @return 返回值：无（void）；非 WAIT_ACK 状态的迟到/重复回调被忽略，失败会回到 WAIT_SYNC 并请求 PWM 安全复位。
 * 调用方式：作为对应 SRP 事务完成回调，由 command_bridge 转发；不要手工伪造 ACK。
 * 线程约束：必须与 manager 主状态机在同一服务任务串行执行；可能等待控制 mutex，禁止 ISR 和并发调用。
 */
void radar_calibration_manager_on_ready_response(srp_link_tx_result_t result,
                                                 uint8_t status_code)
{
    if (!s_initialized || s_state != RADAR_WAIT_ACK) {
        return;
    }
    if (result == SRP_LINK_TX_OK && status_code == SRP_FAST_RESP_OK) {
        s_state = RADAR_WAIT_STATIC_EVENT;
        (void)s3_log_info("RADAR_PWM_READY ACK; WAIT STATIC_CAL_DONE");
        return;
    }
    enter_sync_wait("RADAR_PWM_READY transaction failed");
}

/**
 * @brief 返回当前雷达标定握手状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return radar_calibration_state_t 当前快照；不会附带一致性版本号。
 * 调用方式：服务任务诊断/状态上报读取；不能单独据此绕过 radar_control 安全门。
 * 线程约束：无锁读取；只在 manager 所属服务任务内使用，跨任务读取需由调用方同步，禁止 ISR 依赖该快照作控制决策。
 */
radar_calibration_state_t radar_calibration_manager_get_state(void)
{
    return s_state;
}

/**
 * @brief 返回最近一次 manager 请求的标定 PWM 百分数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 当前请求值；现有流程为 0，不代表 LEDC 写入或 STM ACK 已成功。
 * 调用方式：只用于服务任务诊断/状态显示，实际 PWM 需由 radar_control 状态机确认。
 * 线程约束：无锁快照；仅服务任务读取，跨任务或 ISR 读取不能获得一致性保证。
 */
uint8_t radar_calibration_manager_get_pwm(void)
{
    return s_pwm;
}

/**
 * @brief 查询本次 S3/STM 静态标定握手是否完成。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return true 仅表示 manager 已接受 STATIC_CAL_DONE 且 radar_control 已进入 RUNNING；否则 false。
 * 调用方式：服务层状态上报读取；不等同于 UART、BLE、传感器或车辆硬件验收结果。
 * 线程约束：无锁快照；仅服务任务读取，禁止跨任务/ISR 据此绕过控制门。
 */
bool radar_calibration_manager_is_done(void)
{
    return s_done;
}
