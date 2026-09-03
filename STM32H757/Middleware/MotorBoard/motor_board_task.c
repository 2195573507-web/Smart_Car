#include "motor_board_task.h"

/* MotorBoard 控制/配置任务实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal.h"

#include "log_service.h"
#include "motor_board_protocol.h"
#include "motor_board_transport_uart.h"
#include "pid_controller.h"
#include "srp_registry.h"
#include "srp_wire.h"
#include "s3_service.h"
#include "wheel_control_params.h"

#define MB_TASK_STACK_WORDS UINT16_C(384)
#define MB_TASK_PRIORITY (tskIDLE_PRIORITY + 2U)
#define MB_TASK_POLL_PERIOD_MS UINT32_C(1)
#define MB_TASK_SEQUENCE_GAP_MS UINT32_C(100)
#define MB_TASK_RESPONSE_TIMEOUT_MS UINT32_C(1000)
#define MB_TASK_REPORT_PERIOD_MS UINT32_C(1000)
#define MB_TASK_STATS_PERIOD_MS UINT32_C(5000)
#define MB_TASK_WHEEL_STATUS_PERIOD_MS UINT32_C(50)
#define MB_TASK_POWER_STATUS_PERIOD_MS UINT32_C(500)
#define MB_PID_DT_SECONDS 0.05f
#define MB_WHEEL_COUNT 4U
#define MB_LOG_RAW_CHUNK_LENGTH UINT16_C(64)

#define MB_520_MOTOR_TYPE UINT8_C(1)
#define MB_520_MAGNETIC_LINE_COUNT UINT16_C(11)
#define MB_520_GEAR_RATIO UINT16_C(30)
#define MB_520_WHEEL_DIAMETER_MM UINT16_C(65)

typedef enum {
    MB_SEQUENCE_LINK_PROBE = 0U,
    MB_SEQUENCE_MTYPE,
    MB_SEQUENCE_MLINE,
    MB_SEQUENCE_MPHASE,
    MB_SEQUENCE_WDIAMETER,
    MB_SEQUENCE_READ_FLASH,
    MB_SEQUENCE_READ_VOLTAGE,
    MB_SEQUENCE_UPLOAD,
    MB_SEQUENCE_RUNNING,
    MB_SEQUENCE_FAILED
} mb_sequence_step_t;

typedef enum {
    MB_WAIT_NONE = 0U,
    MB_WAIT_CONFIG_OK,
    MB_WAIT_FLASH_RESPONSE,
    MB_WAIT_BATTERY_RESPONSE,
    MB_WAIT_UPLOAD_RESPONSE
} mb_wait_response_t;

/* Motor-board order is fixed as M1=RR, M2=RF, M3=LR, M4=LF. The RF encoder
 * is electrically inverted and is calibrated exactly once at the PID input. */
static const char *const MOTOR_POSITION_NAME[4] = { "RR", "RF", "LR", "LF" };
static const int8_t ENCODER_DIR_SIGN[4] = { 1, -1, 1, 1 };
static const float s_wheel_trim[4] = {
    WHEEL_TRIM_M1, WHEEL_TRIM_M2, WHEEL_TRIM_M3, WHEEL_TRIM_M4
};

static TaskHandle_t s_task_handle;
static mb_sequence_step_t s_sequence_step;
static mb_wait_response_t s_wait_response;
static TickType_t s_next_sequence_tick;
static TickType_t s_response_deadline;
static TickType_t s_next_report_tick;
static TickType_t s_next_stats_tick;
static TickType_t s_next_wheel_status_tick;
static TickType_t s_next_power_status_tick;
static mb_protocol_frame_t s_latest_battery;
static mb_protocol_frame_t s_latest_mtep;
static mb_protocol_frame_t s_latest_mspd;
static mb_protocol_frame_t s_latest_mall;
static bool s_have_battery;
static bool s_have_mtep;
static bool s_have_mspd;
static bool s_have_mall;
static float s_target_wheel_speed[MB_WHEEL_COUNT];
static float s_actual_wheel_speed[MB_WHEEL_COUNT];
static uint32_t s_actual_wheel_speed_timestamp_ms;
static uint32_t s_actual_wheel_speed_sequence;
static bool s_actual_wheel_speed_valid;
static pid_controller_t s_wheel_pid[MB_WHEEL_COUNT];
static Ramp_Profile_t s_wheel_ramp[MB_WHEEL_COUNT];
static bool s_motion_forced_stop;
static char s_latest_raw[MB_PROTOCOL_MAX_FRAME_LEN];
static uint32_t s_link_probe_attempts;

/**
 * @brief 用有符号差值判断 FreeRTOS tick deadline 是否到期。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param now 当前 tick。
 * @param due 目标 deadline tick。
 * @return now 位于 due 或其后时 true，否则 false。
 * 调用方式：MotorBoard 周期、响应超时和状态机间隔判断调用；两时刻差必须小于 2^31 tick。
 * 线程约束：纯数值计算、可重入、不阻塞。
 */
static bool tick_due(TickType_t now, TickType_t due)
{
    return (int32_t)(now - due) >= 0;
}

/**
 * @brief 把有符号轮 PWM 限制到 +/-WHEEL_PID_MAX_OUT。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param value PID 与 trim 相乘后的 PWM；调用方应保证为有限值。
 * @return 超上/下限时返回边界，否则原样返回；NaN 不会在此被修复。
 * 调用方式：四轮 PID 输出转换为 int16_t 前逐轮调用。
 * 线程约束：纯数值计算、可重入、不阻塞。
 */
static float clamp_wheel_pwm(float value)
{
    if (value > WHEEL_PID_MAX_OUT) {
        return WHEEL_PID_MAX_OUT;
    }
    if (value < -WHEEL_PID_MAX_OUT) {
        return -WHEEL_PID_MAX_OUT;
    }
    return value;
}

/**
 * @brief 把 MotorBoard 启动序列枚举映射为稳定诊断名称。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param step 当前启动序列步骤。
 * @return 静态字符串常量；未知值与 FAILED 均返回 `failed`。
 * 调用方式：配置失败日志保存失败步骤时调用，返回指针无需释放。
 * 线程约束：纯 switch、可重入、不阻塞。
 */
static const char *motor_board_step_name(mb_sequence_step_t step)
{
    switch (step) {
    case MB_SEQUENCE_MTYPE:
        return "mtype";
    case MB_SEQUENCE_MLINE:
        return "mline";
    case MB_SEQUENCE_MPHASE:
        return "mphase";
    case MB_SEQUENCE_WDIAMETER:
        return "wdiameter";
    case MB_SEQUENCE_READ_FLASH:
        return "read_flash";
    case MB_SEQUENCE_READ_VOLTAGE:
        return "read_vol";
    case MB_SEQUENCE_UPLOAD:
        return "upload";
    case MB_SEQUENCE_LINK_PROBE:
        return "link_probe";
    case MB_SEQUENCE_RUNNING:
        return "running";
    case MB_SEQUENCE_FAILED:
    default:
        return "failed";
    }
}

/**
 * @brief 把非 NULL MotorBoard 文本作为 INFO 日志提交到有界日志服务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param text 只读零结尾文本；NULL 时忽略，日志服务返回前复制。
 * @return 无；队列满时由 log_service 计数，本函数不报告失败。
 * 调用方式：本模块格式化普通诊断后统一调用。
 * 线程约束：普通任务上下文；LOG_INFO 使用零等待队列提交，禁止 ISR 调用。
 */
static void motor_board_log(const char *text)
{
    if (text != NULL) {
        LOG_INFO(text);
    }
}

/**
 * @brief 记录一条 MotorBoard 命令的 TX ring 排队结果。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param command 只读命令文本；NULL 在日志中显示 `?`。
 * @param sent true 表示完整文本进入 TX ring，false 表示 DROP。
 * @return 无；`QUEUED` 不代表 USART6 已发送或电机板已执行。
 * 调用方式：启动配置状态机每次发送后调用。
 * 线程约束：使用栈上日志缓冲和日志队列，只在 MotorBoard 任务调用。
 */
static void motor_board_log_send(const char *command, bool sent)
{
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    (void)snprintf(line, sizeof(line),
                   "[MOTOR_BOARD] tx=%s status=%s",
                   command == NULL ? "?" : command,
                   sent ? "QUEUED" : "DROP");
    if (sent) {
        motor_board_log(line);
    } else {
        LOG_ERROR(line);
    }
}

/**
 * @brief 把长文本响应按最多 64 字符分片提交 INFO 日志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param category 非空零结尾类别名；NULL 时忽略。
 * @param raw 非空零结尾文本；NULL 或空串时忽略。
 * @return 无；每片独立入队，队列满时可能只保留部分片段。
 * 调用方式：read_flash 文本响应记录调用，不改变 parser 状态。
 * 线程约束：调用 strlen/snprintf 和日志队列，仅 MotorBoard 普通任务调用。
 */
static void motor_board_log_raw(const char *category, const char *raw)
{
    const char *cursor = raw;
    uint32_t fragment = 0U;

    if (category == NULL || raw == NULL || raw[0] == '\0') {
        return;
    }
    while (*cursor != '\0') {
        char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
        size_t length = strlen(cursor);

        if (length > MB_LOG_RAW_CHUNK_LENGTH) {
            length = MB_LOG_RAW_CHUNK_LENGTH;
        }
        (void)snprintf(line, sizeof(line), "[MOTOR_BOARD] %s[%lu]=%.*s",
                       category, (unsigned long)fragment, (int)length, cursor);
        motor_board_log(line);
        cursor += length;
        ++fragment;
    }
}

/**
 * @brief 把未知文本按最多 16 字节分片为 hex 与可打印 ASCII 的 WARN 日志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param category 非空零结尾类别名；NULL 时忽略。
 * @param raw 非空零结尾字节串；NULL 或首字节为零时忽略。
 * @return 无；格式化失败时提前停止，输入中的首个 NUL 结束全部处理。
 * 调用方式：INVALID 非 `hex=` 和 UNKNOWN frame 的诊断路径调用。
 * 线程约束：使用栈缓冲、snprintf 和日志队列，仅 MotorBoard 任务调用。
 */
static void motor_board_log_unrecognized_raw(const char *category,
                                             const char *raw)
{
    const uint8_t *bytes = (const uint8_t *)raw;
    size_t offset = 0U;

    if (category == NULL || raw == NULL || raw[0] == '\0') {
        return;
    }
    while (raw[offset] != '\0') {
        char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
        char ascii[17];
        size_t length = 0U;
        size_t line_offset;

        while (length < 16U && raw[offset + length] != '\0') {
            const uint8_t byte = bytes[offset + length];
            ascii[length] = (byte >= 0x20U && byte <= 0x7EU) ?
                                (char)byte : '.';
            ++length;
        }
        ascii[length] = '\0';
        line_offset = (size_t)snprintf(line, sizeof(line),
                                       "[MOTOR_BOARD] %s hex=", category);
        if (line_offset >= sizeof(line)) {
            return;
        }
        for (size_t index = 0U;
             index < length && line_offset + 4U < sizeof(line); ++index) {
            const int written = snprintf(&line[line_offset],
                                         sizeof(line) - line_offset,
                                         "%02X%s", bytes[offset + index],
                                         index + 1U == length ? "" : " ");
            if (written < 0) {
                return;
            }
            line_offset += (size_t)written;
        }
        if (line_offset < sizeof(line)) {
            (void)snprintf(&line[line_offset], sizeof(line) - line_offset,
                           " ascii=%s", ascii);
        }
        LOG_WARN(line);
        offset += length;
    }
}

/**
 * @brief 尝试排队四路零 PWM，并把启动序列锁定为 FAILED。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param reason 只在错误日志中借用的原因；NULL 显示 `?`。
 * @return 无；零 PWM 排队失败只记录 DROP，不证明电机已经停止。
 * 调用方式：配置命令排队失败或电机板返回 NACK 时调用。
 * 线程约束：修改无锁序列状态并调用 USART6 TX ring API，仅 MotorBoard task owner 调用。
 */
static void motor_board_enter_failed(const char *reason)
{
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
    const char *failed_step = motor_board_step_name(s_sequence_step);
    const bool stop_queued = MB_Protocol_SendPwm(0, 0, 0, 0);

    s_wait_response = MB_WAIT_NONE;
    s_sequence_step = MB_SEQUENCE_FAILED;
    (void)snprintf(line, sizeof(line),
                   "[MOTOR_BOARD] init failed step=%s reason=%s stop=%s",
                   failed_step,
                   reason == NULL ? "?" : reason,
                   stop_queued ? "QUEUED" : "DROP");
    LOG_ERROR(line);
}

/**
 * @brief 判断帧类型能否作为 `$upload` 配置已生效的响应证据。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param type 已解析的 MotorBoard 帧类型。
 * @return OK_ACK、MTEP、MSPD、BATTERY 或 MALL 返回 true；其他类型返回 false。
 * 调用方式：等待 MB_WAIT_UPLOAD_RESPONSE 时选择是否推进到 RUNNING。
 * 线程约束：纯 switch、可重入、不阻塞。
 */
static bool motor_board_upload_response_is_success(mb_frame_type_t type)
{
    switch (type) {
    case MB_FRAME_OK_ACK:
    case MB_FRAME_MTEP:
    case MB_FRAME_MSPD:
    case MB_FRAME_BATTERY:
    case MB_FRAME_MALL:
        return true;
    case MB_FRAME_INVALID:
    case MB_FRAME_NACK:
    case MB_FRAME_UNKNOWN:
    case MB_FRAME_FLASH_RAW:
    default:
        return false;
    }
}

/**
 * @brief 尝试恢复 USART6 RX，并记录寄存器与 transport 统计摘要。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param reason 最多显示前 8 字符的原因；NULL 显示 `?`。
 * @return 无；RX active 时也以 WARN 记录，失败时记录 ERROR。
 * 调用方式：link probe 重启清理 RX/parser 后调用，不改变控制目标。
 * 线程约束：会直接检查 USART6、进入 transport 临界区并提交日志，禁止 ISR，仅任务 owner 调用。
 */
static void motor_board_log_rx_health(const char *reason)
{
    mb_transport_stats_t stats;
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
    const bool active = MB_Transport_Ensure_Rx_Active();
    const uint32_t isr = USART6->ISR;
    const uint32_t cr1 = USART6->CR1;

    MB_Transport_GetStats(&stats);

    (void)snprintf(line, sizeof(line),
                   "[MB6] %.8s a=%u i=%08lX c=%08lX r=%lu b=%u e=%lu",
                   reason == NULL ? "?" : reason,
                   active ? 1U : 0U,
                   (unsigned long)isr,
                   (unsigned long)cr1,
                   (unsigned long)stats.rx_bytes,
                   (unsigned int)stats.rx_buffered,
                   (unsigned long)stats.uart_errors);
    if (active) {
        LOG_WARN(line);
    } else {
        LOG_ERROR(line);
    }
}

/**
 * @brief 丢弃 USART6 RX 与 parser 半帧，并立即回到零 PWM link probe 步骤。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param now 当前 FreeRTOS tick，作为下一次 probe 的立即 deadline。
 * @param reason 只在健康/重试日志中借用；允许 NULL。
 * @return 无；不会清 TX ring、轮速目标、PID 状态或累计 probe 次数。
 * 调用方式：等待响应超时后由启动序列调用，下一轮 run_sequence 重新排队零 PWM。
 * 线程约束：修改 transport/parser/任务全局状态，只允许 MotorBoard task owner 调用。
 */
static void motor_board_restart_link_probe(TickType_t now, const char *reason)
{
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    MB_Transport_ClearRx();
    MB_Protocol_ResetRx();
    s_wait_response = MB_WAIT_NONE;
    s_sequence_step = MB_SEQUENCE_LINK_PROBE;
    s_next_sequence_tick = now;
    motor_board_log_rx_health(reason);
    (void)snprintf(line, sizeof(line),
                   "[MOTOR_BOARD] link retry=%lu reason=%s rx/parser reset",
                   (unsigned long)(s_link_probe_attempts + 1U),
                   reason == NULL ? "?" : reason);
    LOG_WARN(line);
}

/**
 * @brief 清除当前响应等待，按枚举顺序推进一个配置步骤并安排 100 ms 间隔。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param now 当前 FreeRTOS tick。
 * @return 无；依赖 mb_sequence_step_t 的配置步骤保持连续有序。
 * 调用方式：handle_response() 收到与当前等待匹配的成功响应后调用。
 * 线程约束：无锁修改状态机字段，仅 MotorBoard task owner 调用。
 */
static void motor_board_advance_sequence(TickType_t now)
{
    s_wait_response = MB_WAIT_NONE;
    s_response_deadline = now;
    s_sequence_step = (mb_sequence_step_t)(s_sequence_step + 1U);
    s_next_sequence_tick = now + pdMS_TO_TICKS(MB_TASK_SEQUENCE_GAP_MS);
}

/**
 * @brief 记录待等响应类型并设置固定 1000 ms deadline。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param response 期望响应类别；MB_WAIT_NONE 不修改任何状态。
 * @param now 当前 FreeRTOS tick。
 * @return 当前实现始终返回 true，表示状态记录操作本身不会失败。
 * 调用方式：配置文本成功进入 TX ring 后由 run_sequence() 调用。
 * 线程约束：无锁修改等待状态，只允许 MotorBoard task owner 调用。
 */
static bool motor_board_wait_for_response(mb_wait_response_t response,
                                          TickType_t now)
{
    if (response == MB_WAIT_NONE) {
        return true;
    }
    s_wait_response = response;
    s_response_deadline = now + pdMS_TO_TICKS(MB_TASK_RESPONSE_TIMEOUT_MS);
    return true;
}

/**
 * @brief 缓存最近响应，并按 frame type 更新业务快照或输出有界诊断。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param frame parser 产出的只读完整帧；NULL 时返回，内容在本函数内按值复制。
 * @return 无；BATTERY/MTEP/MSPD/MALL 更新对应 have 标志，其他类型只记录日志。
 * 调用方式：MotorBoard task 每次 Poll 成功后、状态机响应处理和 PID 更新前调用。
 * 线程约束：无锁修改模块快照并可能提交多条日志，只允许 MotorBoard task owner 调用。
 */
static void motor_board_log_frame_event(const mb_protocol_frame_t *frame)
{
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    if (frame == NULL) {
        return;
    }
    (void)snprintf(s_latest_raw, sizeof(s_latest_raw), "%s", frame->raw);
    switch (frame->type) {
    case MB_FRAME_BATTERY:
        s_latest_battery = *frame;
        s_have_battery = true;
        break;
    case MB_FRAME_MTEP:
        s_latest_mtep = *frame;
        s_have_mtep = true;
        break;
    case MB_FRAME_MSPD:
        s_latest_mspd = *frame;
        s_have_mspd = true;
        break;
    case MB_FRAME_MALL:
        s_latest_mall = *frame;
        s_have_mall = true;
        break;
    case MB_FRAME_OK_ACK:
        (void)snprintf(line, sizeof(line), "[MOTOR_BOARD] ok_ack=%.64s", frame->raw);
        motor_board_log(line);
        break;
    case MB_FRAME_FLASH_RAW:
        motor_board_log_raw("flash", frame->raw);
        break;
    case MB_FRAME_NACK:
        (void)snprintf(line, sizeof(line), "[MOTOR_BOARD] nack=%.75s", frame->raw);
        LOG_ERROR(line);
        break;
    case MB_FRAME_INVALID:
        if (strncmp(frame->raw, "hex=", 4U) == 0) {
            (void)snprintf(line, sizeof(line),
                           "[MOTOR_BOARD] raw %.76s", frame->raw);
            LOG_WARN(line);
        } else {
            motor_board_log_unrecognized_raw("invalid", frame->raw);
        }
        break;
    case MB_FRAME_UNKNOWN:
    default:
        motor_board_log_unrecognized_raw("unknown", frame->raw);
        break;
    }
}

/**
 * @brief 按当前等待类型消费 MotorBoard 响应，并推进配置状态机或进入 FAILED。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（关键静态函数契约补充）。
 * @param frame parser 产出的只读帧；NULL 或当前无等待时忽略。
 * @param now 当前 FreeRTOS tick，用于计算下一步/超时期限。
 * @return 无；不匹配当前等待类型的帧仅由日志/缓存路径处理，不推进状态。
 * 调用方式：仅 MotorBoard 任务在 Poll 成功后调用。
 * 线程约束：直接修改无锁状态机字段，单任务 owner，禁止 ISR 或并发调用。
 */
static void motor_board_handle_response(const mb_protocol_frame_t *frame,
                                        TickType_t now)
{
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    if (frame == NULL || s_wait_response == MB_WAIT_NONE) {
        return;
    }
    if (frame->type == MB_FRAME_NACK) {
        motor_board_enter_failed("board_nack");
        return;
    }
    switch (s_wait_response) {
    case MB_WAIT_CONFIG_OK:
        if (frame->type == MB_FRAME_OK_ACK) {
            (void)snprintf(line, sizeof(line),
                           "[MOTOR_BOARD] 520 %s=OK status=%.48s",
                           motor_board_step_name(s_sequence_step),
                           frame->response_status[0] == '\0' ? frame->raw :
                                                               frame->response_status);
            motor_board_log(line);
            motor_board_advance_sequence(now);
        }
        break;
    case MB_WAIT_FLASH_RESPONSE:
        /* read_flash may return several plain-text configuration lines before
         * the final OK acknowledgement. Keep each line and wait for OK. */
        if (frame->type == MB_FRAME_OK_ACK) {
            (void)snprintf(line, sizeof(line),
                           "[MOTOR_BOARD] read_flash response=OK status=%.48s",
                           frame->response_status[0] == '\0' ? frame->raw :
                                                               frame->response_status);
            motor_board_log(line);
            motor_board_advance_sequence(now);
        }
        break;
    case MB_WAIT_BATTERY_RESPONSE:
        if (frame->type == MB_FRAME_BATTERY) {
            (void)snprintf(line, sizeof(line),
                           "[MOTOR_BOARD] read_vol=%.2fV OK",
                           (double)frame->battery_voltage);
            motor_board_log(line);
            motor_board_advance_sequence(now);
        }
        break;
    case MB_WAIT_UPLOAD_RESPONSE:
        if (motor_board_upload_response_is_success(frame->type)) {
            (void)snprintf(line, sizeof(line),
                           "[MOTOR_BOARD] upload=OK source=%s data=%.48s",
                           frame->type == MB_FRAME_OK_ACK ? "ACK" : "STREAM",
                           frame->response_status[0] == '\0' ? frame->raw :
                                                               frame->response_status);
            motor_board_log(line);
            motor_board_advance_sequence(now);
        }
        break;
    case MB_WAIT_NONE:
    default:
        break;
    }
}

/** 提交四轮目标速度；数组顺序固定为 RR/RF/LR/LF。 */
bool motor_board_set_target_wheel_speeds(const float speeds[MB_WHEEL_COUNT])
{
    bool any_nonzero = false;

    if (speeds == NULL) {
        return false;
    }
    for (size_t index = 0U; index < MB_WHEEL_COUNT; ++index) {
        if (!isfinite(speeds[index]) ||
            fabsf(speeds[index]) > WHEEL_SPEED_MAX_TARGET_LIMIT) {
            return false;
        }
        any_nonzero = any_nonzero || speeds[index] != 0.0f;
    }
    if (!any_nonzero) {
        return motor_board_force_stop();
    }
    taskENTER_CRITICAL();
    (void)memcpy(s_target_wheel_speed, speeds, sizeof(s_target_wheel_speed));
    s_motion_forced_stop = false;
    taskEXIT_CRITICAL();
    return true;
}

/** 更新 PID 与加速度限幅，拒绝非有限或越界参数。 */
bool motor_board_update_pid_params(float kp, float ki, float kd,
                                   float max_accel)
{
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
        !isfinite(max_accel) || kp < SRP_PID_KP_MIN || kp > SRP_PID_KP_MAX ||
        ki < SRP_PID_KI_MIN || ki > SRP_PID_KI_MAX ||
        kd < SRP_PID_KD_MIN || kd > SRP_PID_KD_MAX ||
        max_accel < SRP_PID_ACCEL_MIN || max_accel > SRP_PID_ACCEL_MAX) {
        return false;
    }

    taskENTER_CRITICAL();
    for (size_t index = 0U; index < MB_WHEEL_COUNT; ++index) {
        PID_Update_Gains(&s_wheel_pid[index], kp, ki, kd);
        Ramp_Update_Max_Accel(&s_wheel_ramp[index], max_accel);
    }
    taskEXIT_CRITICAL();
    return true;
}

/** 清零目标并发送 MotorBoard 安全停机序列。 */
bool motor_board_force_stop(void)
{
    bool sent;

    taskENTER_CRITICAL();
    (void)memset(s_target_wheel_speed, 0, sizeof(s_target_wheel_speed));
    for (size_t index = 0U; index < MB_WHEEL_COUNT; ++index) {
        pid_controller_reset(&s_wheel_pid[index]);
        s_wheel_ramp[index].current_target = 0.0f;
    }
    s_motion_forced_stop = true;
    /* Serialize the physical stop with a PID update that may already have
     * copied a target, so a stale nonzero PWM cannot follow this stop. */
    sent = MB_Protocol_SendPwm(0, 0, 0, 0);
    taskEXIT_CRITICAL();
    if (!sent) {
        LOG_ERROR("[MOTOR_BOARD] forced stop PWM queue drop");
    }
    return sent;
}

/** 复制最近一次实际轮速反馈。 */
void motor_board_get_actual_wheel_speeds(float speeds[MB_WHEEL_COUNT])
{
    motor_board_wheel_speed_snapshot_t snapshot;

    if (speeds != NULL) {
        (void)motor_board_get_actual_wheel_speed_snapshot(&snapshot);
        (void)memcpy(speeds, snapshot.speed_mm_s,
                     sizeof(snapshot.speed_mm_s));
    }
}

bool motor_board_get_actual_wheel_speed_snapshot(
    motor_board_wheel_speed_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    (void)memcpy(snapshot->speed_mm_s, s_actual_wheel_speed,
                 sizeof(snapshot->speed_mm_s));
    snapshot->timestamp_ms = s_actual_wheel_speed_timestamp_ms;
    snapshot->sequence = s_actual_wheel_speed_sequence;
    snapshot->valid = s_actual_wheel_speed_valid;
    taskEXIT_CRITICAL();
    return snapshot->valid;
}

/** 读取最近一次电池电压反馈。 */
bool motor_board_get_battery_voltage(float *voltage)
{
    if (voltage == NULL || !s_have_battery || !isfinite(s_latest_battery.battery_voltage)) {
        return false;
    }
    *voltage = s_latest_battery.battery_voltage;
    return true;
}

/**
 * @brief 用 MSPD 反馈执行四轮斜坡/PID/trim，并把 [RR,RF,LR,LF] PWM 排入 USART6 ring。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（关键静态函数契约补充）。
 * @param frame 只读 MotorBoard 帧；仅 MB_FRAME_MSPD 会处理，RF 编码器极性在此修正一次。
 * @return 无；强停 latch 生效时只更新实际轮速，不产生新 PWM；排队失败记录警告。
 * 调用方式：MotorBoard 单一任务每收到速度反馈调用，控制步长固定为 MB_PID_DT_SECONDS。
 * 线程约束：四轮计算、状态更新和 TX 排队均位于 FreeRTOS 临界区，期间会屏蔽调度/IRQ；
 *           禁止 ISR/并发调用，扩展计算前必须评估最坏临界区时长。
 */
static void motor_board_update_pid(const mb_protocol_frame_t *frame)
{
    int16_t pwm[MB_WHEEL_COUNT];
    float target_speed[MB_WHEEL_COUNT];
    bool sent;
    const uint32_t sample_timestamp_ms = HAL_GetTick();

    if (frame == NULL || frame->type != MB_FRAME_MSPD) {
        return;
    }
    taskENTER_CRITICAL();
    if (s_motion_forced_stop) {
        for (size_t index = 0U; index < MB_WHEEL_COUNT; ++index) {
            s_actual_wheel_speed[index] =
                frame->speed[index] * (float)ENCODER_DIR_SIGN[index];
        }
        s_actual_wheel_speed_timestamp_ms = sample_timestamp_ms;
        ++s_actual_wheel_speed_sequence;
        s_actual_wheel_speed_valid = true;
        taskEXIT_CRITICAL();
        return;
    }
    (void)memcpy(target_speed, s_target_wheel_speed, sizeof(target_speed));
    for (size_t index = 0U; index < MB_WHEEL_COUNT; ++index) {
        const float raw_speed = frame->speed[index];
        const float actual_speed = raw_speed * (float)ENCODER_DIR_SIGN[index];
        if (fabsf(target_speed[index]) > 0.0f &&
            fabsf(target_speed[index]) < WHEEL_SPEED_MIN_TARGET_LIMIT) {
            target_speed[index] = 0.0f;
        }
        const float smooth_target = Ramp_Update(&s_wheel_ramp[index],
                                                target_speed[index],
                                                MB_PID_DT_SECONDS);

        /* PID output is already signed. Trim changes magnitude only; it
         * neither changes direction nor bypasses the final output limit. */
        const float raw_pwm = pid_controller_step(
            &s_wheel_pid[index], smooth_target,
            actual_speed, MB_PID_DT_SECONDS);
        s_actual_wheel_speed[index] = actual_speed;
        pwm[index] = (int16_t)clamp_wheel_pwm(
            raw_pwm * s_wheel_trim[index]);
    }
    s_actual_wheel_speed_timestamp_ms = sample_timestamp_ms;
    ++s_actual_wheel_speed_sequence;
    s_actual_wheel_speed_valid = true;
    sent = MB_Protocol_SendPwm(pwm[0], pwm[1], pwm[2], pwm[3]);
    taskEXIT_CRITICAL();
    if (!sent) {
        LOG_WARN("[MOTOR_BOARD] PID PWM queue drop");
    }
}

/**
 * @brief 把最近四轮实际速度按小端 float 编码为 SRP WHEEL_SPEED_STATUS。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无；忽略服务发送返回值，失败由 SRP/UART 统计体现。
 * 调用方式：MotorBoard task 每 50 ms 调用，数组顺序固定为 [RR,RF,LR,LF]。
 * 线程约束：读取同一 task owner 更新的实际速度并可能等待服务 mutex/UART，禁止 ISR。
 */
static void motor_board_send_wheel_status(void)
{
    uint8_t payload[SRP_PAYLOAD_WHEEL_SPEED_STATUS_SIZE];

    srp_wire_write_f32_array_le(payload, s_actual_wheel_speed, MB_WHEEL_COUNT);
    (void)s3_service_send_message(SRP_PRIORITY_TELEMETRY,
                                  SRP_MSG_ID_WHEEL_SPEED_STATUS,
                                  SRP_FLAG_STREAM_DATA, payload,
                                  sizeof(payload));
}

/**
 * @brief 把最近有效电池电压编码为 SRP POWER_STATUS 遥测。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无；尚无有限电压时不发送，服务发送失败不向本函数调用者返回。
 * 调用方式：MotorBoard task 每 500 ms 调用；缓存无独立 freshness 时间戳。
 * 线程约束：同一 task owner 读取缓存，发送路径可能等待 mutex/UART，禁止 ISR。
 */
static void motor_board_send_power_status(void)
{
    float voltage;
    uint8_t payload[SRP_PAYLOAD_POWER_STATUS_SIZE];

    if (!motor_board_get_battery_voltage(&voltage)) {
        return;
    }
    srp_wire_write_f32_le(payload, voltage);
    (void)s3_service_send_message(SRP_PRIORITY_TELEMETRY,
                                  SRP_MSG_ID_POWER_STATUS,
                                  SRP_FLAG_STREAM_DATA, payload,
                                  sizeof(payload));
}

/**
 * @brief 按 have 标志输出最近 Battery/MTEP/MSPD/MAll 和原始响应快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无；读取后不清 have 标志或缓存，因此后续周期可重复同一值。
 * 调用方式：MotorBoard task 每 1000 ms 调用；日志轮序显示 M1=RR,M2=RF,M3=LR,M4=LF。
 * 线程约束：使用共享静态日志缓冲与模块快照，只允许 MotorBoard task owner 调用。
 */
static void motor_board_log_snapshot(void)
{
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    if (s_have_battery) {
        (void)snprintf(line, sizeof(line), "[MOTOR_BOARD] Battery=%.2fV",
                       (double)s_latest_battery.battery_voltage);
        motor_board_log(line);
    }
    if (s_have_mtep) {
        (void)snprintf(line, sizeof(line),
                       "[MOTOR_BOARD] MTEP M1(%s)=%ld M2(%s)=%ld "
                       "M3(%s)=%ld M4(%s)=%ld",
                       MOTOR_POSITION_NAME[0],
                       (long)s_latest_mtep.pulse[0],
                       MOTOR_POSITION_NAME[1],
                       (long)s_latest_mtep.pulse[1],
                       MOTOR_POSITION_NAME[2],
                       (long)s_latest_mtep.pulse[2],
                       MOTOR_POSITION_NAME[3],
                       (long)s_latest_mtep.pulse[3]);
        motor_board_log(line);
    }
    if (s_have_mspd) {
        (void)snprintf(line, sizeof(line),
                       "[MOTOR_BOARD] MSPD M1(%s)=%.2f M2(%s)=%.2f "
                       "M3(%s)=%.2f M4(%s)=%.2f",
                       MOTOR_POSITION_NAME[0],
                       (double)s_latest_mspd.speed[0],
                       MOTOR_POSITION_NAME[1],
                       (double)s_latest_mspd.speed[1],
                       MOTOR_POSITION_NAME[2],
                       (double)s_latest_mspd.speed[2],
                       MOTOR_POSITION_NAME[3],
                       (double)s_latest_mspd.speed[3]);
        motor_board_log(line);
    }
    if (s_have_mall) {
        (void)snprintf(line, sizeof(line),
                       "[MOTOR_BOARD] MAll M1=%ld M2=%ld M3=%ld M4=%ld",
                       (long)s_latest_mall.pulse[0],
                       (long)s_latest_mall.pulse[1],
                       (long)s_latest_mall.pulse[2],
                       (long)s_latest_mall.pulse[3]);
        motor_board_log(line);
    }
    if (s_latest_raw[0] != '\0') {
        (void)snprintf(line, sizeof(line), "[MOTOR_BOARD] raw=%.76s",
                       s_latest_raw);
        motor_board_log(line);
    }
}

/**
 * @brief 汇总 USART6 transport 与文本 parser 累计统计并输出一条 INFO 日志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无；快照读取不清零，日志排队失败不反馈。
 * 调用方式：MotorBoard task 每 5000 ms 调用，用于诊断而非链路验收。
 * 线程约束：transport 快照使用短临界区，protocol 统计由同一 task owner 读取；禁止 ISR。
 */
static void motor_board_log_stats(void)
{
    mb_transport_stats_t transport_stats;
    mb_protocol_stats_t protocol_stats;
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    MB_Transport_GetStats(&transport_stats);
    MB_Protocol_GetStats(&protocol_stats);
    (void)snprintf(line, sizeof(line),
                   "[MOTOR_BOARD] stats rx=%lu tx=%lu ro=%lu to=%lu ue=%lu "
                   "fr=%lu inv=%lu",
                   (unsigned long)transport_stats.rx_bytes,
                   (unsigned long)transport_stats.tx_bytes,
                   (unsigned long)transport_stats.rx_overflow,
                   (unsigned long)transport_stats.tx_overflow,
                   (unsigned long)transport_stats.uart_errors,
                   (unsigned long)protocol_stats.frames,
                   (unsigned long)protocol_stats.invalid_frames);
    motor_board_log(line);
}

/**
 * @brief 推进零 PWM 探测、520 参数配置、Flash/电压查询和上传使能启动序列。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（关键静态函数契约补充）。
 * @param now 当前 FreeRTOS tick，所有 deadline 用有符号差值判断回绕。
 * @return 无；响应超时会清 transport/parser 并回到零 PWM link probe，配置排队失败进入 FAILED。
 * 调用方式：MotorBoard 任务每 1 ms 调用；只有匹配响应通过 handle_response 后才推进下一步。
 * 线程约束：无锁状态机、单任务 owner；会短临界区排队 USART6 文本，禁止 ISR/并发调用。
 */
static void motor_board_run_sequence(TickType_t now)
{
    bool sent;

    if (s_wait_response != MB_WAIT_NONE) {
        if (tick_due(now, s_response_deadline)) {
            motor_board_restart_link_probe(now, "response_timeout");
        }
        return;
    }
    if (!tick_due(now, s_next_sequence_tick)) {
        return;
    }
    switch (s_sequence_step) {
    case MB_SEQUENCE_LINK_PROBE:
        ++s_link_probe_attempts;
        sent = MB_Protocol_SendPwm(0, 0, 0, 0);
        motor_board_log_send("$pwm:0,0,0,0#", sent);
        if (sent) {
            s_sequence_step = MB_SEQUENCE_MTYPE;
            s_next_sequence_tick = now + pdMS_TO_TICKS(MB_TASK_SEQUENCE_GAP_MS);
        } else {
            s_next_sequence_tick =
                now + pdMS_TO_TICKS(MB_TASK_RESPONSE_TIMEOUT_MS);
        }
        break;
    case MB_SEQUENCE_MTYPE:
        sent = MB_Protocol_SendMotorType(MB_520_MOTOR_TYPE);
        motor_board_log_send("$mtype:1#", sent);
        if (sent) {
            (void)motor_board_wait_for_response(MB_WAIT_CONFIG_OK, now);
        } else {
            motor_board_enter_failed("tx_drop");
        }
        break;
    case MB_SEQUENCE_MLINE:
        sent = MB_Protocol_SendMagneticLine(MB_520_MAGNETIC_LINE_COUNT);
        motor_board_log_send("$mline:11#", sent);
        if (sent) {
            (void)motor_board_wait_for_response(MB_WAIT_CONFIG_OK, now);
        } else {
            motor_board_enter_failed("tx_drop");
        }
        break;
    case MB_SEQUENCE_MPHASE:
        sent = MB_Protocol_SendGearRatio(MB_520_GEAR_RATIO);
        motor_board_log_send("$mphase:30#", sent);
        if (sent) {
            (void)motor_board_wait_for_response(MB_WAIT_CONFIG_OK, now);
        } else {
            motor_board_enter_failed("tx_drop");
        }
        break;
    case MB_SEQUENCE_WDIAMETER:
        sent = MB_Protocol_SendWheelDiameter(MB_520_WHEEL_DIAMETER_MM);
        motor_board_log_send("$wdiameter:65#", sent);
        if (sent) {
            (void)motor_board_wait_for_response(MB_WAIT_CONFIG_OK, now);
        } else {
            motor_board_enter_failed("tx_drop");
        }
        break;
    case MB_SEQUENCE_READ_FLASH:
        sent = MB_Protocol_SendReadFlash();
        motor_board_log_send("$read_flash#", sent);
        if (sent) {
            (void)motor_board_wait_for_response(MB_WAIT_FLASH_RESPONSE, now);
        } else {
            motor_board_enter_failed("tx_drop");
        }
        break;
    case MB_SEQUENCE_READ_VOLTAGE:
        sent = MB_Protocol_SendReadVoltage();
        motor_board_log_send("$read_vol#", sent);
        if (sent) {
            (void)motor_board_wait_for_response(MB_WAIT_BATTERY_RESPONSE, now);
        } else {
            motor_board_enter_failed("tx_drop");
        }
        break;
    case MB_SEQUENCE_UPLOAD:
        sent = MB_Protocol_SendUpload(false, true, true);
        motor_board_log_send("$upload:0,1,1#", sent);
        if (sent) {
            (void)motor_board_wait_for_response(MB_WAIT_UPLOAD_RESPONSE, now);
        } else {
            motor_board_enter_failed("tx_drop");
        }
        break;
    case MB_SEQUENCE_RUNNING:
    case MB_SEQUENCE_FAILED:
    default:
        break;
    }
}

/** MotorBoard FreeRTOS 任务：配置序列、周期目标、反馈解析和恢复。 */
void motor_board_task(void *argument)
{
    TickType_t now;
    mb_protocol_frame_t frame;

    (void)argument;
    MB_Transport_ClearRx();
    s_sequence_step = MB_SEQUENCE_LINK_PROBE;
    s_wait_response = MB_WAIT_NONE;
    s_have_battery = false;
    s_have_mtep = false;
    s_have_mspd = false;
    s_have_mall = false;
    s_latest_raw[0] = '\0';
    s_link_probe_attempts = 0U;
    s_motion_forced_stop = false;
    now = xTaskGetTickCount();
    s_next_sequence_tick = now;
    s_next_report_tick = now + pdMS_TO_TICKS(MB_TASK_REPORT_PERIOD_MS);
    s_next_stats_tick = now + pdMS_TO_TICKS(MB_TASK_STATS_PERIOD_MS);
    s_next_wheel_status_tick = now + pdMS_TO_TICKS(MB_TASK_WHEEL_STATUS_PERIOD_MS);
    s_next_power_status_tick = now + pdMS_TO_TICKS(MB_TASK_POWER_STATUS_PERIOD_MS);
    (void)memset(s_target_wheel_speed, 0, sizeof(s_target_wheel_speed));
    (void)memset(s_actual_wheel_speed, 0, sizeof(s_actual_wheel_speed));
    s_actual_wheel_speed_timestamp_ms = 0U;
    s_actual_wheel_speed_sequence = 0U;
    s_actual_wheel_speed_valid = false;
    for (size_t index = 0U; index < MB_WHEEL_COUNT; ++index) {
        s_wheel_ramp[index].current_target = 0.0f;
        s_wheel_ramp[index].max_accel = WHEEL_RAMP_MAX_ACCEL;
        pid_controller_init(&s_wheel_pid[index], WHEEL_PID_KP, WHEEL_PID_KI,
                            WHEEL_PID_KD, WHEEL_PID_MAX_OUT,
                            WHEEL_PID_MAX_IOUT, WHEEL_SPEED_DEADBAND);
    }
    for (;;) {
        now = xTaskGetTickCount();
        while (MB_Protocol_Poll(&frame)) {
            motor_board_log_frame_event(&frame);
            motor_board_handle_response(&frame, now);
            motor_board_update_pid(&frame);
            now = xTaskGetTickCount();
        }
        motor_board_run_sequence(now);
        if (tick_due(now, s_next_wheel_status_tick)) {
            s_next_wheel_status_tick = now + pdMS_TO_TICKS(MB_TASK_WHEEL_STATUS_PERIOD_MS);
            motor_board_send_wheel_status();
        }
        if (tick_due(now, s_next_power_status_tick)) {
            s_next_power_status_tick = now + pdMS_TO_TICKS(MB_TASK_POWER_STATUS_PERIOD_MS);
            motor_board_send_power_status();
        }
        if (tick_due(now, s_next_report_tick)) {
            s_next_report_tick = now + pdMS_TO_TICKS(MB_TASK_REPORT_PERIOD_MS);
            motor_board_log_snapshot();
        }
        if (tick_due(now, s_next_stats_tick)) {
            s_next_stats_tick = now + pdMS_TO_TICKS(MB_TASK_STATS_PERIOD_MS);
            motor_board_log_stats();
        }
        vTaskDelay(pdMS_TO_TICKS(MB_TASK_POLL_PERIOD_MS));
    }
}

/** 创建唯一 MotorBoard 任务。 */
void motor_board_task_start(void)
{
    if (!MB_Transport_IsReady() || s_task_handle != NULL) {
        return;
    }
    if (xTaskCreate(motor_board_task, "motor_board", MB_TASK_STACK_WORDS,
                    NULL, MB_TASK_PRIORITY, &s_task_handle) != pdPASS) {
        s_task_handle = NULL;
        LOG_ERROR("[MOTOR_BOARD] task create failed");
    }
}
