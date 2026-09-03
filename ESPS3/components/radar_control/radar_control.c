#include "radar_control.h"

#include <stdbool.h>
#include <stdio.h>

#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "s3_ble.h"

/* 雷达 PWM/状态机实现；创建人：待确认（当前维护人：Zhiqin）。 */

/* These values match the existing M_CTR setup in main/radar/radar_uart.c. */
#define RADAR_CONTROL_PWM_MODE LEDC_LOW_SPEED_MODE
#define RADAR_CONTROL_PWM_CHANNEL LEDC_CHANNEL_0
#define RADAR_CONTROL_PWM_DUTY_RESOLUTION_BITS 10U
#define RADAR_CONTROL_PWM_DUTY_MAX \
    ((1U << RADAR_CONTROL_PWM_DUTY_RESOLUTION_BITS) - 1U)
#define RADAR_CONTROL_SPEED_PERCENT_BASE 100U
static const char *TAG = "RADAR_CONTROL";

static uint8_t s_speed_percent;
static uint8_t s_calibration_pwm;
static volatile bool s_imu_cal_done;
static volatile radar_control_state_t s_state = RADAR_CONTROL_WAIT_STM_QUERY;
static bool s_calibration_active;
static bool s_initialized;
static SemaphoreHandle_t s_lock;

/**
 * @brief 尝试在 20 ms 内获取雷达控制互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return mutex 已创建且在超时前取得时返回 true；未初始化或超时返回 false。
 * 调用方式：所有访问控制状态或 LEDC 的任务路径先调用，成功后必须配对 radar_control_unlock()。
 * 线程约束：会阻塞最多 20 ms，只能在 FreeRTOS 任务上下文调用；禁止 ISR/GATT 回调，且同任务不可递归获取非递归 mutex。
 */
static bool radar_control_lock(void)
{
    return s_lock != NULL &&
           xSemaphoreTake(s_lock, pdMS_TO_TICKS(20U)) == pdTRUE;
}

/**
 * @brief 释放当前任务已持有的雷达控制互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；mutex 尚未创建时不动作。
 * 调用方式：仅在 radar_control_lock() 成功后由同一任务配对调用。
 * 线程约束：不验证当前 owner；错误释放会破坏互斥语义，因此禁止 ISR、跨任务释放或未持锁调用。
 */
static void radar_control_unlock(void)
{
    if (s_lock != NULL) {
        (void)xSemaphoreGive(s_lock);
    }
}

/**
 * @brief 将雷达速度百分数按 10 位 LEDC 满量程换算为 duty 计数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param percent 待换算百分数；本函数不裁剪，调用方应先保证范围为 0..100。
 * @return 使用整数除法向下取整后的 duty；0 对应 0，100 对应 RADAR_CONTROL_PWM_DUTY_MAX。
 * 调用方式：由日志计算和 apply_speed() 调用，不直接写硬件。
 * 线程约束：纯计算、可重入、不阻塞，可在任务/回调使用；当前控制链禁止从 ISR 发起后续 LEDC 操作。
 */
static uint32_t speed_to_duty(uint8_t percent)
{
    return (RADAR_CONTROL_PWM_DUTY_MAX * (uint32_t)percent) /
           RADAR_CONTROL_SPEED_PERCENT_BASE;
}

/**
 * @brief 将 s_speed_percent 换算并提交到既有 M_CTR LEDC 通道。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return ESP_OK 表示 set/update 两步均成功；否则返回首个 ESP-IDF LEDC 错误码。
 * 调用方式：radar_pwm_init() 已配置通道后，由持有 s_lock 的初始化/状态迁移路径调用。
 * 线程约束：读取无原子的全局速度并访问 LEDC，调用者必须持有控制 mutex；禁止 ISR、并发或无锁调用，成功也不证明雷达物理转动。
 */
static esp_err_t apply_speed(void)
{
    const uint32_t duty = speed_to_duty(s_speed_percent);
    esp_err_t ret = ledc_set_duty(RADAR_CONTROL_PWM_MODE,
                                  RADAR_CONTROL_PWM_CHANNEL,
                                  duty);
    if (ret == ESP_OK) {
        ret = ledc_update_duty(RADAR_CONTROL_PWM_MODE,
                               RADAR_CONTROL_PWM_CHANNEL);
    }
    return ret;
}

/**
 * @brief 初始化雷达状态机、控制 mutex 和零占空比默认状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；mutex 分配失败时保持未初始化，LEDC 更新失败只记录日志且实际输出未确认。
 * 调用方式：radar_pwm_init() 完成后、其他 radar_control_* 接口前由启动路径调用；成功初始化后的重复调用直接返回。
 * 线程约束：创建 mutex 后以 portMAX_DELAY 首次获取，只允许启动任务串行调用；会分配 FreeRTOS 对象并访问 LEDC，禁止 ISR/GATT 回调。
 */
void radar_control_init(void)
{
    if (s_initialized) {
        return;
    }

    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        ESP_LOGE(TAG, "RADAR CONTROL mutex creation failed");
        return;
    }
    (void)xSemaphoreTake(s_lock, portMAX_DELAY);
    s_speed_percent = RADAR_MIN_SPEED;
    s_calibration_pwm = RADAR_MIN_SPEED;
    s_imu_cal_done = false;
    s_state = RADAR_CONTROL_WAIT_STM_QUERY;
    s_calibration_active = false;
    s_initialized = true;
    if (apply_speed() != ESP_OK) {
        ESP_LOGE(TAG, "M_CTR initial duty update failed");
    }
    (void)xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "RADAR CONTROL READY state=WAIT_STM_QUERY speed=%u",
             (unsigned)s_speed_percent);
}

/**
 * @brief 在 BOOT、IMU 标定和覆盖门均放行后应用 App 请求的雷达转速。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param percent 目标占空比百分数，范围为 0..100。
 * @return true 表示 RUNNING/标定完成/无覆盖且 LEDC 已更新；参数、锁、状态或 LEDC 失败返回 false，并尽力恢复原速度。
 * 调用方式：由服务任务处理已校验 App 雷达调速命令时调用；调用方必须把 false 当作硬件状态未确认。
 * 线程约束：最多等待控制 mutex 20 ms，并调用日志与 LEDC；禁止 ISR/GATT 回调，同一任务不得已持锁再调用。
 */
bool radar_control_set_speed(uint8_t percent)
{
    bool accepted = false;

    ESP_LOGI(TAG, "input speed=%u", (unsigned)percent);
    ESP_LOGI(TAG, "calculated duty=%u",
             (unsigned)(percent <= RADAR_MAX_SPEED ? speed_to_duty(percent) : 0U));
    if (percent > RADAR_MAX_SPEED || !radar_control_lock()) {
        return false;
    }
    if (s_initialized && s_state == RADAR_CONTROL_RUNNING &&
        s_imu_cal_done && !s_calibration_active) {
        const uint8_t previous_speed = s_speed_percent;
        s_speed_percent = percent;
        if (apply_speed() == ESP_OK) {
            accepted = true;
        } else {
            s_speed_percent = previous_speed;
            (void)apply_speed();
        }
    }
    radar_control_unlock();
    return accepted;
}

/**
 * @brief 报告 STM-owned IMU 标定状态并推进或回退雷达运行门。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param done true 仅在 WAIT_IMU_CAL 接受完成事件；false 在 BOOT 门释放后回到等待标定并请求零 PWM。
 * @return 返回值：无（void）；乱序/重复事件、锁失败或 LEDC 失败时不进入 RUNNING，调用方需查询状态确认。
 * 调用方式：仅由 S3 标定服务根据已校验 SRP 事件或同步回退路径调用。
 * 线程约束：可能等待 mutex 20 ms 并访问 LEDC；禁止 ISR/GATT 回调或已持有控制锁时递归调用。
 */
void radar_control_set_imu_cal_done(bool done)
{
    if (!radar_control_lock()) {
        return;
    }
    if (!s_initialized) {
        radar_control_unlock();
        return;
    }

    if (!done) {
        if (s_state == RADAR_CONTROL_WAIT_STM_QUERY) {
            radar_control_unlock();
            return;
        }
        s_speed_percent = RADAR_MIN_SPEED;
        s_calibration_active = false;
        s_calibration_pwm = RADAR_MIN_SPEED;
        if (apply_speed() != ESP_OK) {
            ESP_LOGE(TAG, "IMU calibration reset PWM update failed");
        }
        s_imu_cal_done = false;
        s_state = RADAR_CONTROL_WAIT_IMU_CAL;
        radar_control_unlock();
        return;
    }

    /* A completion event is valid only after the accepted STM boot query has
     * released BOOT/WAIT_STM_QUERY. This rejects stale or out-of-order events. */
    if (s_state != RADAR_CONTROL_WAIT_IMU_CAL || s_imu_cal_done) {
        radar_control_unlock();
        return;
    }

    /* Calibration completion releases the App speed-control gate. */
    s_calibration_active = false;
    s_calibration_pwm = RADAR_MIN_SPEED;
    s_speed_percent = RADAR_MIN_SPEED;
    if (apply_speed() != ESP_OK) {
        /* Keep the controller non-running when the requested safe zero duty
         * could not be committed. */
        ESP_LOGE(TAG, "IMU calibration completion PWM update failed");
        radar_control_unlock();
        return;
    }
    s_imu_cal_done = true;
    s_state = RADAR_CONTROL_CAL_DONE;
    ESP_LOGI(TAG, "CAL_DONE_RECEIVED");
    /* Do not reintroduce the old automatic 0-to-100 percent ramp here. The
     * next accepted App command owns the first nonzero duty. */
    s_state = RADAR_CONTROL_RUNNING;
    ESP_LOGI(TAG, "RADAR CONTROL RUNNING speed=%u",
             (unsigned)s_speed_percent);
    radar_control_unlock();
}

/**
 * @brief 在 WAIT_IMU_CAL 中应用 STM32-owned 标定 PWM 覆盖。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param percent 标定占空比百分数，范围为 0..100。
 * @return true 表示 LEDC 已应用且覆盖标志已建立；参数、锁、状态或 LEDC 失败返回 false，并尽力恢复先前软件值/占空比。
 * 调用方式：由标定 manager 的服务任务路径调用；完成或取消后另行调用 radar_control_release_calibration_lock()。
 * 线程约束：最多等待 mutex 20 ms，并执行日志、格式化和 LEDC 写；禁止 ISR/GATT 回调或锁内递归调用，失败后的物理占空比仍未确认。
 */
bool radar_control_set_calibration_pwm(uint8_t percent)
{
    char line[48];
    uint8_t previous_speed;
    uint8_t previous_calibration_pwm;
    bool previous_calibration_active;

    (void)snprintf(line, sizeof(line), "RADAR SET PWM speed=%u",
                   (unsigned)percent);
    (void)s3_log_info(line);
    if (percent > RADAR_MAX_SPEED || !radar_control_lock()) {
        (void)s3_log_error("PWM RESULT FAIL");
        return false;
    }
    if (!s_initialized || s_state != RADAR_CONTROL_WAIT_IMU_CAL ||
        s_imu_cal_done) {
        radar_control_unlock();
        (void)s3_log_error("PWM RESULT FAIL");
        return false;
    }

    previous_speed = s_speed_percent;
    previous_calibration_pwm = s_calibration_pwm;
    previous_calibration_active = s_calibration_active;
    s_calibration_pwm = percent;
    s_speed_percent = percent;
    if (apply_speed() != ESP_OK) {
        s_speed_percent = previous_speed;
        s_calibration_pwm = previous_calibration_pwm;
        s_calibration_active = previous_calibration_active;
        (void)apply_speed();
        (void)s3_log_error("PWM RESULT FAIL");
        radar_control_unlock();
        return false;
    }
    s_calibration_active = true;
    (void)s3_log_info("PWM RESULT OK");
    radar_control_unlock();
    return true;
}

/**
 * @brief 清除标定 PWM 覆盖标志，但不改写当前速度或 LEDC duty。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；锁失败时保持原覆盖状态。
 * 调用方式：标定事务完成、取消或回退后由服务任务调用；本函数不是零 PWM 命令。
 * 线程约束：可能等待控制 mutex 20 ms；禁止 ISR/GATT 回调或持锁递归调用。
 */
void radar_control_release_calibration_lock(void)
{
    if (!radar_control_lock()) {
        return;
    }
    s_calibration_active = false;
    radar_control_unlock();
}

/**
 * @brief 获取本地标定 PWM 覆盖是否有效的受锁快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 取得 mutex 后返回“已初始化且覆盖有效”；锁不可用、超时或未初始化时保守返回 false。
 * 调用方式：供服务任务诊断/状态判断；false 可能只是锁超时，不能单独证明覆盖已安全释放。
 * 线程约束：只读但最多阻塞 20 ms；禁止 ISR/GATT 回调或控制锁内调用。
 */
bool radar_control_is_calibration_active(void)
{
    bool active = false;
    if (radar_control_lock()) {
        active = s_initialized && s_calibration_active;
        radar_control_unlock();
    }
    return active;
}

/**
 * @brief 获取本地雷达状态机是否处于 RADAR_CONTROL_RUNNING 的受锁快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 锁失败或未初始化时返回 false；true 只表示本地状态机 RUNNING，不证明雷达物理旋转或 STM 链路健康。
 * 调用方式：服务任务在标定完成校验或状态上报时调用。
 * 线程约束：只读但最多阻塞 20 ms；禁止 ISR/GATT 回调或控制锁内递归调用。
 */
bool radar_control_is_running(void)
{
    bool running = false;
    if (radar_control_lock()) {
        running = s_initialized && s_state == RADAR_CONTROL_RUNNING;
        radar_control_unlock();
    }
    return running;
}

/**
 * @brief 获取当前雷达控制状态机的受锁快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 当前状态；锁不可用或超时时返回安全默认 RADAR_CONTROL_WAIT_STM_QUERY。
 * 调用方式：供服务任务诊断和状态上报；安全准入仍应调用控制接口，而不是自行比较一次快照。
 * 线程约束：最多等待 mutex 20 ms；禁止 ISR/GATT 回调或控制锁内调用。
 */
radar_control_state_t radar_control_get_state(void)
{
    radar_control_state_t state = RADAR_CONTROL_WAIT_STM_QUERY;
    if (radar_control_lock()) {
        state = s_state;
        radar_control_unlock();
    }
    return state;
}

/**
 * @brief 获取当前 s_speed_percent 百分比的受锁快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 0..100 的软件值；锁失败时返回 RADAR_MIN_SPEED，标定覆盖期间也可能反映标定 PWM。
 * 调用方式：仅用于状态上报；是否处于标定覆盖需结合 radar_control_is_calibration_active() 判断。
 * 线程约束：最多等待 mutex 20 ms；禁止 ISR/GATT 回调或控制锁内调用，返回值不证明 LEDC/物理转速。
 */
uint8_t radar_control_get_speed(void)
{
    uint8_t speed = RADAR_MIN_SPEED;
    if (radar_control_lock()) {
        speed = s_speed_percent;
        radar_control_unlock();
    }
    return speed;
}

/**
 * @brief 获取最近一次成功提交的标定覆盖 PWM 百分比快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 0..100 的记录值；锁失败时返回 RADAR_MIN_SPEED，释放覆盖不会自动清零该记录。
 * 调用方式：仅用于服务任务标定诊断；不能据此证明当前 LEDC 仍使用该 duty。
 * 线程约束：最多等待 mutex 20 ms；禁止 ISR/GATT 回调或控制锁内调用。
 */
uint8_t radar_control_get_calibration_pwm(void)
{
    uint8_t pwm = RADAR_MIN_SPEED;
    if (radar_control_lock()) {
        pwm = s_calibration_pwm;
        radar_control_unlock();
    }
    return pwm;
}

/**
 * @brief 接受 STM32 启动 PWM 查询，并在零 duty 提交成功后释放 BOOT 门控。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；锁或 LEDC 失败时保持 BOOT 门控，重复/乱序查询不改变状态。
 * 调用方式：仅由标定 manager 处理已校验 STM BOOT_READY 消息时调用。
 * 线程约束：最多等待 mutex 20 ms 并访问 LEDC；禁止 ISR/GATT 回调或控制锁内递归调用。
 */
void radar_control_handle_pwm_ready_query(void)
{
    if (!radar_control_lock()) {
        return;
    }
    if (s_initialized && s_state == RADAR_CONTROL_WAIT_STM_QUERY) {
        s_speed_percent = RADAR_MIN_SPEED;
        if (apply_speed() != ESP_OK) {
            ESP_LOGE(TAG, "BOOT zero-duty update failed");
            radar_control_unlock();
            return;
        }
        s_state = RADAR_CONTROL_WAIT_IMU_CAL;
        ESP_LOGI(TAG, "PWM_READY");
        ESP_LOGI(TAG, "WAIT_IMU_CAL");
    }
    radar_control_unlock();
}
