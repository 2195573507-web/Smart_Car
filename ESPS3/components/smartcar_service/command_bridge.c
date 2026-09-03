#include "smartcar_service.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "app_parser.h"
#include "log_bridge.h"
#include "radar_calibration_manager.h"
#include "radar_control.h"
#include "s3_ble.h"
#include "srp_crc.h"
#include "srp_link.h"
#include "srp_codec.h"
#include "srp_wire.h"
#include "smartcar_debug_config.h"
#include "stm_uart.h"

#define SMARTCAR_SERVICE_TASK_NAME "smartcar_svc"
/*
 * ESP-IDF measures xTaskCreate() stack depth in bytes. The service task is
 * the serialized consumer for BLE RX, SRP parsing, UART forwarding, and
 * reconnect/stop handling, so reserve 16 KB for nested protocol callbacks.
 *
 * menuconfig recommendation: FreeRTOS stack overflow check = Method 3
 * (ISR + MPU), when this check is available for the selected target/IDF.
 * Real-vehicle check: run the `tasks` CLI command during BLE reconnect and
 * radar-burst tests; keep `smartcar_svc` min_free_words > 4096.
 */
#define SMARTCAR_SERVICE_TASK_STACK 16384U
#define SMARTCAR_SERVICE_TASK_PRIORITY 8U
#define SMARTCAR_SERVICE_TASK_DELAY_TICKS 1U
#define SMARTCAR_SERVICE_BLE_RX_QUEUE_DEPTH 8U
#define SMARTCAR_SERVICE_BLE_RX_BUDGET 4U
#define SMARTCAR_SERVICE_BUS_OFF_RECOVERY_MS UINT32_C(100)
#define APP_RADAR_STATUS_PERIOD_MS UINT32_C(1000)
#define APP_V2_HEARTBEAT_PERIOD_MS UINT32_C(500)
#define APP_V2_SESSION_TTL_MS UINT32_C(3000)
#define SMARTCAR_SERVICE_BAUD_SWITCH_GUARD_MS UINT32_C(20)
#define SMARTCAR_SERVICE_SYNC_RETRY_PERIOD_MS UINT32_C(500)
#define SMARTCAR_SERVICE_SYNC_HEARTBEAT_PERIOD_MS UINT32_C(100)
#define SMARTCAR_SERVICE_SYNC_MAX_ATTEMPTS UINT8_C(10)
#define SMARTCAR_SERVICE_SYNC_TIMEOUT_RETRY_MS UINT32_C(500)
#define SMARTCAR_SERVICE_STM_FRAME_TIMEOUT_MS UINT32_C(1500)
#define SMARTCAR_SERVICE_APP_BLE_DIAG_PERIOD_MS UINT32_C(1000)
#define SC_APP_SYNC_TIMEOUT_ERROR UINT16_C(0x0201)

typedef struct {
    uint16_t length;
    uint32_t connection_epoch;
    uint8_t bytes[SC_APP_FRAME_MAX_SIZE];
} smartcar_ble_rx_item_t;

typedef struct {
    bool valid;
    uint8_t app_type;
    uint8_t app_version;
    uint32_t app_session_id;
    uint32_t app_sequence;
    uint16_t message_id;
    uint8_t length;
    uint32_t generation;
    uint64_t valid_until_us;
    uint8_t payload[SRP_PAYLOAD_CHASSIS_SPEED_CMD_SIZE];
} smartcar_motion_command_t;

typedef struct {
    uint8_t version;
    uint32_t session_id;
    uint32_t sequence;
    uint64_t valid_until_us;
} smartcar_app_command_meta_t;

typedef struct {
    uint8_t app_type;
    uint8_t app_version;
    uint32_t session_id;
    uint32_t sequence;
} smartcar_app_tx_context_t;

typedef struct {
    bool active;
    uint32_t session_id;
    uint32_t last_sequence;
    uint64_t last_activity_us;
    bool have_sequence;
    bool have_last_ack;
    uint32_t last_ack_sequence;
    uint8_t last_ack_type;
    uint8_t last_ack_result;
    uint8_t last_ack_stage;
} smartcar_app_v2_session_t;

typedef enum {
    SMARTCAR_SYNC_UART_READY = 0U,
    SMARTCAR_SYNC_SYNCING = 1U,
    SMARTCAR_SYNC_SYNCED = 2U,
    SMARTCAR_SYNC_TIMEOUT = 3U
} smartcar_sync_state_t;

static const char *TAG = "SRP";
static srp_parser_t s_parser;
static srp_link_t s_link;
static sc_app_parser_t s_app_parser;
static QueueHandle_t s_ble_rx_queue;
static StaticQueue_t s_ble_rx_queue_storage;
static TaskHandle_t s_task;
static uint8_t s_link_ready;
static bool s_bus_off_recovery_pending;
static bool s_bus_off_latched;
static uint64_t s_bus_off_recovery_at_us;
static volatile uint32_t s_ble_rx_dropped;
static volatile uint32_t s_ble_rx_received;
static volatile uint32_t s_ble_rx_protocol_errors;
static volatile UBaseType_t s_stack_min_free_bytes;
static volatile bool s_stack_hwm_valid;
static uint32_t s_parser_errors;
static uint32_t s_dual_len_reject;
static uint32_t s_dual_schema_reject;
static uint32_t s_dual_crc_reject;
static uint32_t s_dual_notify_drop;
static uint32_t s_dual_ble_not_ready;
static volatile bool s_ble_disconnect_stop_pending;
static volatile uint32_t s_ble_connection_epoch;
static bool s_ble_stop_cleanup_active;
static bool s_baud_change_pending;
static uint32_t s_baud_change_value;
static uint64_t s_baud_change_due_us;
static smartcar_service_telemetry_sink_t s_telemetry_sink;
static void *s_telemetry_sink_context;

static uint8_t s_ble_rx_queue_buffer[
    SMARTCAR_SERVICE_BLE_RX_QUEUE_DEPTH * sizeof(smartcar_ble_rx_item_t)]
    __attribute__((aligned(4)));
static uint8_t s_uart_rx_buffer[256U];
static smartcar_ble_rx_item_t s_ble_rx_item;
static smartcar_motion_command_t s_motion_inflight;
static smartcar_motion_command_t s_motion_pending_target;
static smartcar_motion_command_t s_motion_pending_scale;
static bool s_motion_tx_in_flight;
static uint32_t s_motion_generation;
static uint8_t s_app_tx_frame[SC_APP_FRAME_MAX_SIZE];
static smartcar_app_v2_session_t s_app_v2_session;
static uint32_t s_app_v2_next_session_id = 0x51A70001U;
static smartcar_app_tx_context_t s_pid_tx_context;
static smartcar_app_tx_context_t s_baud_tx_context;
static smartcar_sync_state_t s_sync_state;
static uint8_t s_sync_attempts;
static uint8_t s_sync_tx_sequence;
static uint64_t s_next_sync_us;
static uint64_t s_last_sync_timeout_notify_us;
static uint64_t s_last_stm_frame_us;
static uint64_t s_last_uart_diag_us;
static uint64_t s_last_rx_reset_log_us;
static uint64_t s_last_header_fail_log_us;
static uint64_t s_last_app_ble_diag_us;
static uint32_t s_app_valid_frames;
static uint32_t s_app_command_frames;
static uint32_t s_app_ack_ok;
static uint32_t s_app_ack_rejected;
static uint32_t s_app_ack_dropped;
static uint8_t s_app_last_ack_type;
static uint8_t s_app_last_ack_result;
static uint8_t s_app_last_ack_stage;
static uint32_t s_app_last_ack_sequence;

static void cancel_motion_transactions(void);
static void app_command_on_frame(const sc_app_frame_view_t *input_frame,
                                 void *context);
static void app_command_on_error(int error, const uint8_t *data,
                                 size_t length, void *context);

/**
 * @brief 对 App/S3 诊断计数执行饱和加一，避免长期运行回绕。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-09-03（BLE 指令链路诊断）。
 * @param value 非 NULL 诊断计数指针；调用方负责并发上下文约束。
 * @return 无。
 * 调用方式：FFE1 GATT 入队、App parser 和 ACK 路径记录有界累计值。
 * 线程约束：不加锁、不阻塞；调用点仅允许使用自身已建立的 owner/volatile 边界。
 */
static void command_bridge_saturating_increment(volatile uint32_t *value)
{
    if (value != NULL && *value != UINT32_MAX) {
        ++(*value);
    }
}

/**
 * @brief 识别当前 App V1 业务命令类型，不把遥测/握手帧计入命令入口。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-09-03（BLE 指令链路诊断）。
 * @param type App V1 type 字段。
 * @return true 表示会进入命令仲裁或配置路径；false 表示状态/响应/未知类型。
 * 调用方式：合法 App 外层帧回调中调用一次，仅用于诊断计数，不改变准入。
 * 线程约束：纯函数，不阻塞、不持有指针、不访问共享状态。
 */
static bool command_bridge_is_app_command_type(uint8_t type)
{
    switch (type) {
    case SC_APP_TYPE_WHEEL_SPEED_CMD:
    case SC_APP_TYPE_PID_PARAMS_CMD:
    case SC_APP_TYPE_RADAR_SET_SPEED:
    case SC_APP_TYPE_WHEEL_SPEED_SINGLE_CMD:
    case SC_APP_TYPE_MASTER_SPEED_CMD:
    case SC_APP_TYPE_CHASSIS_SPEED_CMD:
    case SC_APP_TYPE_CHASSIS_HEADING_CMD:
    case SC_APP_TYPE_SYS_CONFIG:
        return true;
    default:
        return false;
    }
}

/**
 * @brief 每秒输出一组 App BLE 跨层计数，区分 GATT 入队、完整帧、命令 ACK 和真实断开。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-09-03（BLE 指令链路诊断）。
 * @param now 当前 S3 单调时间，单位 us。
 * @return 无；周期未到时直接返回，所有计数仅做只读快照。
 * 调用方式：smartcar_service_task 在普通服务任务上下文每轮调用一次。
 * 线程约束：不阻塞控制队列；s3_ble 诊断快照只短暂持锁，日志输出有界且不参与准入。
 */
static void command_bridge_log_app_ble_diag(uint64_t now)
{
    s3_ble_disconnect_info_t disconnect_info = {0};

    if (now - s_last_app_ble_diag_us <
        ((uint64_t)SMARTCAR_SERVICE_APP_BLE_DIAG_PERIOD_MS * UINT64_C(1000))) {
        return;
    }
    s_last_app_ble_diag_us = now;
    (void)s3_ble_get_disconnect_info(&disconnect_info);
    ESP_LOGI(TAG, "APP_BLE_RX in=%lu drop=%lu valid=%lu cmd=%lu pe=%lu",
             (unsigned long)s_ble_rx_received,
             (unsigned long)s_ble_rx_dropped,
             (unsigned long)s_app_valid_frames,
             (unsigned long)s_app_command_frames,
             (unsigned long)s_ble_rx_protocol_errors);
    ESP_LOGI(TAG,
             "APP_BLE_TX rdy=%u ok=%lu bad=%lu adrop=%lu nf=%lu "
             "last=0x%02X/%u/%u seq=%lu disc=%lu dr=0x%02X",
             s3_ble_is_ready() ? 1U : 0U,
             (unsigned long)s_app_ack_ok,
             (unsigned long)s_app_ack_rejected,
             (unsigned long)s_app_ack_dropped,
             (unsigned long)s3_ble_get_notify_fail_count(),
             (unsigned)s_app_last_ack_type,
             (unsigned)s_app_last_ack_result,
             (unsigned)s_app_last_ack_stage,
             (unsigned long)s_app_last_ack_sequence,
             (unsigned long)disconnect_info.count,
             disconnect_info.valid ? (unsigned)disconnect_info.reason : 0U);
}

/**
 * @brief 读取 BLE 断连/会话停止请求，作为服务任务内的快速取消检查。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-09-02（断连旧命令重放修复）。
 * @return true 表示 GATT 回调或会话时效已请求停机；false 表示当前未请求。
 * 调用方式：服务循环、App parser 回调、运动事务入口和 SRP transport 发送前调用。
 * 线程约束：只读取 volatile 标志，不阻塞、不修改 service-owned link 状态。
 */
static bool command_bridge_ble_stop_requested(void)
{
    return s_ble_disconnect_stop_pending;
}

/**
 * @brief 清空 BLE RX 队列，丢弃停止/会话边界前已经排队的 App 字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-09-02（断连旧命令重放修复）。
 * @return 返回值：无；队列为空或尚未创建时不执行操作。
 * 调用方式：仅由 smartcar_service 任务在 BLE 断连或 App 会话过期边界调用。
 * 线程约束：不在 GATT 回调/ISR 中调用；不会解析或发送任何控制帧。
 */
static void command_bridge_flush_ble_rx_queue(void)
{
    if (s_ble_rx_queue != NULL) {
        (void)xQueueReset(s_ble_rx_queue);
    }
}

/**
 * @brief 重置 App 增量 parser 和 V2 会话状态，防止旧连接继续产生运动准入。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-09-02（断连旧命令重放修复）。
 * @return 返回值：无；回调绑定保持不变，仅清除半帧和会话状态。
 * 调用方式：仅由 smartcar_service 任务在断连/会话停止屏障调用。
 * 线程约束：必须与 sc_app_parser_feed() 串行，禁止 GATT 回调或并发重置。
 */
static void command_bridge_reset_app_session(void)
{
    sc_app_parser_init(&s_app_parser, app_command_on_frame,
                       app_command_on_error, NULL);
    memset(&s_app_v2_session, 0, sizeof(s_app_v2_session));
}

/**
 * @brief 在服务启动前注册或清除唯一的 STM 遥测下游 sink。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param sink 接收完整 SRP v4 wire 帧的同步回调；NULL 表示清除。
 * @param context 原样借用并传给 sink；必须与 sink 同时为 NULL 或同时非 NULL，生命周期覆盖服务运行期。
 * @return 注册成功返回 ESP_OK；参数不成对返回 ESP_ERR_INVALID_ARG；服务链路初始化后返回 ESP_ERR_INVALID_STATE。
 * 调用方式：必须在 smartcar_service_init() 前由启动路径调用；sink 返回值当前仅供下游自身诊断，服务不重试。
 * 线程约束：注册字段无锁写入，仅启动任务串行调用；sink 在服务任务中同步执行，必须复制输入、快速返回且不得递归进入服务/link。
 */
esp_err_t smartcar_service_set_telemetry_sink(
    smartcar_service_telemetry_sink_t sink,
    void *context)
{
    if ((sink == NULL) != (context == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_link_ready != 0U) {
        return ESP_ERR_INVALID_STATE;
    }
    s_telemetry_sink = sink;
    s_telemetry_sink_context = context;
    return ESP_OK;
}

/**
 * @brief 严格解析 SYS_CONFIG TLV，并提取唯一且在协议白名单内的 UART 波特率。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param frame 只读 SRP 逻辑帧；必须含非 NULL payload、TLV 标志和至少一个 TLV 头。
 * @param baud_rate 可写输出；成功时写入 SRP_BAUDRATE_DEFAULT 或 SRP_BAUDRATE_DEBUG，失败时内容可能已被部分改写、不可使用。
 * @return 找到恰好一个 4 字节波特率 TLV、全部 TLV 结构合法且值在白名单时为 true，否则为 false。
 * 调用方式：由 STM SYS_CONFIG 分发和 App SYS_CONFIG 准入路径调用；未知 TLV 可跳过，重复波特率 TLV 被拒绝。
 * 线程约束：不分配内存、不保留 payload；当前仅服务任务调用，frame/baud_rate 在调用期间须有效，禁止并发共享输出。
 */
static bool command_bridge_decode_baudrate(const srp_frame_t *frame,
                                           uint32_t *baud_rate)
{
    srp_tlv_iter_t iterator;
    bool found = false;
    uint8_t tag;
    uint8_t value_length;
    const uint8_t *value;

    if (frame == NULL || baud_rate == NULL || frame->payload == NULL ||
        (frame->flags & SRP_FLAG_TLV) == 0U || frame->length < 2U) {
        return false;
    }
    srp_tlv_iter_init(&iterator, frame->payload, frame->length);
    while (iterator.offset < iterator.length) {
        if (iterator.length - iterator.offset < 2U ||
            !srp_tlv_next(&iterator, &tag, &value_length, &value)) {
            return false;
        }
        if (tag == SRP_TLV_TAG_BAUDRATE) {
            if (found || value_length != 4U) {
                return false;
            }
            *baud_rate = srp_wire_read_u32_le(value);
            found = true;
        }
    }
    return found && (*baud_rate == SRP_BAUDRATE_DEFAULT ||
                     *baud_rate == SRP_BAUDRATE_DEBUG);
}

/**
 * @brief 读取 ESP 单调微秒计时，供同步、会话时效和诊断限频使用。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return esp_timer_get_time() 的非负微秒值；不是墙上时间，也不代表外设事件发生时刻。
 * 调用方式：仅供本文件服务状态机计算相对截止时间或生成 SRP 毫秒时间戳。
 * 线程约束：不访问本模块可变状态；当前任务/回调路径可调用，禁止把计时读取当成链路或硬件完成证据。
 */
static uint64_t now_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

/**
 * @brief 复位 S3<->STM32 SRP 会话、UART/parser/link 状态和所有运动事务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param reason 只在调用期间用于日志的零结尾原因；允许 NULL。
 * @return 返回值：无（void）；恢复后进入 UART_READY 并重新探测，旧运动事务不重放；本函数不直接发送零速或证明车辆已停。
 * 调用方式：服务任务在 UART 断流、BUS_OFF 或版本不匹配时调用；同时撤销运动目标，
 *            且不会把旧帧留在恢复后的发送队列中。
 * 线程约束：会 flush UART、修改 parser/link/会话全局状态，只允许 smartcar_service owner 调用；
 *           禁止 GATT 回调、ISR 或并发调用。
 */
static void command_bridge_restart_sync(const char *reason)
{
    const uint64_t restart_at = now_us();

    stm_uart_recover();
    srp_parser_reset(&s_parser);
    srp_link_recover(&s_link);
    stm_uart_set_sync_state(false);
    s_sync_state = SMARTCAR_SYNC_UART_READY;
    s_sync_attempts = 0U;
    s_sync_tx_sequence = 0U;
    s_next_sync_us = restart_at;
    s_last_sync_timeout_notify_us = 0U;
    s_last_stm_frame_us = 0U;
    s_bus_off_recovery_pending = false;
    s_bus_off_latched = false;
    s_motion_pending_target.valid = false;
    s_motion_pending_scale.valid = false;
    cancel_motion_transactions();
    ESP_LOGW(TAG, "SRP sync recovery reason=%s; probing STM32 again",
             reason == NULL ? "unknown" : reason);
}

/**
 * @brief 使用 App V1 包络编码 payload，并通过共享静态缓冲向 FFE2 提交通知。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param type App V1 消息类型。
 * @param payload 只读 payload；length 为 0 时允许 NULL，函数不保留指针。
 * @param length payload 字节数，不得超过 App 帧上限。
 * @return 编码成功且 BLE 接受全部通知分片时为 true；编码、连接、CCC 或底层提交失败时为 false。
 * 调用方式：服务任务把同步状态、遥测或 ACK 转成 App 帧时调用；true 不代表 App 已收到。
 * 线程约束：使用全局 s_app_tx_frame，不可重入或并发；可能进入 Bluedroid 发送，禁止 ISR/GATT 回调和持有不兼容锁调用。
 */
static bool notify_app_frame(uint8_t type, const uint8_t *payload, uint16_t length)
{
    uint16_t frame_length = 0U;

    if (sc_app_frame_encode(type, payload, length, s_app_tx_frame,
                            sizeof(s_app_tx_frame), &frame_length) != 0) {
        return false;
    }
    return s3_ble_notify_send(s_app_tx_frame, frame_length) == ESP_OK;
}

/**
 * @brief 向 App 提交同步状态事件并记录 ESTABLISHED/WAITING 日志，不改变底层安全门。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param synced 仅选择日志中的 ESTABLISHED 或 WAITING 文本；当前状态 payload 不单独编码该布尔值。
 * @param timeout true 时在 4 字节 STATUS payload 中写入 SC_APP_SYNC_TIMEOUT_ERROR，否则 payload 全零。
 * @return 返回值：无（void）；非 timeout 通知失败当前静默丢弃，timeout 失败会额外记录警告。
 * 调用方式：同步成功、同步超时及超时持续提示路径由服务任务调用；App 通知结果不反馈到底层 sync 状态。
 * 线程约束：会使用共享 App TX 缓冲并调用 BLE/日志，只允许服务任务 owner；禁止 ISR/GATT 回调或并发调用。
 */
static void command_bridge_notify_sync_status(bool synced, bool timeout)
{
    uint8_t payload[4] = {0U, 0U, 0U, 0U};

    if (timeout) {
        payload[3] = (uint8_t)(SC_APP_SYNC_TIMEOUT_ERROR & 0xFFU);
        payload[2] = (uint8_t)(SC_APP_SYNC_TIMEOUT_ERROR >> 8U);
    }
    if (!notify_app_frame(SC_APP_TYPE_STATUS, payload, sizeof(payload)) && timeout) {
        ESP_LOGW(TAG, "SYNC_TIMEOUT app status notify dropped");
    }
    ESP_LOGI(TAG, "SRP sync state=%s%s", synced ? "ESTABLISHED" : "WAITING",
             timeout ? " timeout" : "");
}

/**
 * @brief 在同步探测次数耗尽后清理链路并进入可周期重试的 TIMEOUT 状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；已处于 TIMEOUT 时幂等返回。
 * 调用方式：仅由 command_bridge_sync_step() 在服务任务中调用，并向 App 上报超时。
 * 线程约束：会 flush UART、重置 parser/link 和撤销运动事务；禁止 ISR/GATT/并发调用。
 */
static void command_bridge_enter_sync_timeout(void)
{
    const uint64_t timeout_at = now_us();

    if (s_sync_state == SMARTCAR_SYNC_TIMEOUT) {
        return;
    }
    /* Drop stale UART bytes and link transactions before reopening the retry
     * window. This is the recovery boundary after ten unanswered probes. */
    stm_uart_recover();
    srp_parser_reset(&s_parser);
    srp_link_recover(&s_link);
    s_sync_state = SMARTCAR_SYNC_TIMEOUT;
    stm_uart_set_sync_state(false);
    s_motion_pending_target.valid = false;
    s_motion_pending_scale.valid = false;
    cancel_motion_transactions();
    s_last_sync_timeout_notify_us = timeout_at;
    s_next_sync_us = timeout_at +
                     ((uint64_t)SMARTCAR_SERVICE_SYNC_TIMEOUT_RETRY_MS * UINT64_C(1000));
    command_bridge_notify_sync_status(false, true);
    ESP_LOGW(TAG, "SYNC_TIMEOUT attempts=%u", (unsigned)s_sync_attempts);
}

/**
 * @brief 推进 S3 主动发起的 SRP v4 同步、心跳和有限超时重试状态机。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param now 当前 S3 单调时间，单位 us。
 * @return 返回值：无（void）；发送失败仍消耗一次探测并按下一时刻继续，次数耗尽转 TIMEOUT。
 * 调用方式：smartcar_service 每轮先调用；同步请求固定为 `{4,0,0,0}`。
 * 线程约束：修改 link 和同步全局状态，可能同步进入 UART 发送；仅服务任务 owner 调用。
 */
static void command_bridge_sync_step(uint64_t now)
{
    static const uint8_t payload[SRP_PAYLOAD_CMD_SYNC_REQ_SIZE] = {
        SRP_PROTOCOL_VERSION_MAJOR, SRP_PROTOCOL_VERSION_MINOR, 0U, 0U
    };

    if (s_sync_state == SMARTCAR_SYNC_SYNCED) {
        if (now < s_next_sync_us) {
            return;
        }
        s_sync_tx_sequence = s_link.next_sequence;
        if (srp_link_send(&s_link, SRP_PRIORITY_COMMAND,
                          SRP_NODE_STM32H757, SRP_MSG_ID_CMD_SYNC_REQ,
                          SRP_FLAG_STREAM_DATA, payload, sizeof(payload),
                          (uint32_t)(now / UINT64_C(1000)), NULL, NULL) != 0) {
            ESP_LOGW(TAG, "SRP heartbeat transport send failed");
        }
        s_next_sync_us = now +
                         ((uint64_t)SMARTCAR_SERVICE_SYNC_HEARTBEAT_PERIOD_MS *
                          UINT64_C(1000));
        return;
    }
    if (s_sync_state == SMARTCAR_SYNC_TIMEOUT) {
        if (now < s_next_sync_us) {
            if (now - s_last_sync_timeout_notify_us >= UINT64_C(1000000)) {
                s_last_sync_timeout_notify_us = now;
                command_bridge_notify_sync_status(false, true);
            }
            return;
        }
        s_sync_state = SMARTCAR_SYNC_UART_READY;
        s_sync_attempts = 0U;
        s_next_sync_us = now;
    }
    if (now < s_next_sync_us) {
        return;
    }
    if (s_sync_attempts >= SMARTCAR_SERVICE_SYNC_MAX_ATTEMPTS) {
        command_bridge_enter_sync_timeout();
        return;
    }
    s_sync_state = SMARTCAR_SYNC_SYNCING;
    s_sync_tx_sequence = s_link.next_sequence;
    if (srp_link_send(&s_link, SRP_PRIORITY_COMMAND, SRP_NODE_STM32H757,
                      SRP_MSG_ID_CMD_SYNC_REQ, SRP_FLAG_STREAM_DATA,
                      payload, sizeof(payload),
                      (uint32_t)(now / UINT64_C(1000)), NULL, NULL) != 0) {
        ESP_LOGW(TAG, "SYNC_REQ transport send failed attempt=%u",
                 (unsigned)(s_sync_attempts + 1U));
    }
    ++s_sync_attempts;
    s_next_sync_us = now + ((uint64_t)SMARTCAR_SERVICE_SYNC_RETRY_PERIOD_MS *
                            UINT64_C(1000));
}

/**
 * @brief 将 srp_link 的完整线缆帧同步交给 STM UART2 发送适配层。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data 只读完整 SRP 帧，仅在回调期间借用。
 * @param length data 的实际字节数。
 * @param context link 配置上下文，当前忽略，允许 NULL。
 * @return 0 仅在 stm_uart_send() 返回完整长度时成立；否则返回 -1。
 * 调用方式：由 srp_link_send()/tick() 在服务任务调用栈中同步调用，不执行业务解析。
 * 线程约束：继承 UART 发送阻塞/串行化约束；不得保留 data 或递归操作同一 link。
 */
static int command_bridge_transport_send(const uint8_t *data, uint16_t length,
                                         void *context)
{
    (void)context;
    /* A disconnect can arrive while srp_link_tick() is walking its pending
     * slots. Reject ordinary traffic until the service task owns the stop
     * cleanup; that prevents a stale motion retry from reaching STM32. The
     * cleanup flag is reserved for the two explicit zero safety frames. */
    if (command_bridge_ble_stop_requested() && !s_ble_stop_cleanup_active) {
        return -1;
    }
    return stm_uart_send(data, length) == (int)length ? 0 : -1;
}

/**
 * @brief 按统一周期读取并输出 UART2 收发、错误、BREAK 和软件 ring 诊断摘要。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param now 当前 S3 单调时间，单位 us。
 * @return 返回值：无（void）；未到周期直接返回，统计 mutex 失败时 get_stats() 提供全零快照。
 * 调用方式：smartcar_service 每轮调用，实际输出受 SMARTCAR_SERVICE_UART_DIAG_PERIOD_MS 限频。
 * 线程约束：最多等待 UART storage mutex 10 ms 并调用 ESP_LOG；仅服务任务调用，禁止 ISR/GATT 回调，日志不能作为物理链路验收。
 */
static void command_bridge_log_uart_diag(uint64_t now)
{
    stm_uart_stats_t stats;

    if (now - s_last_uart_diag_us <
        ((uint64_t)SMARTCAR_SERVICE_UART_DIAG_PERIOD_MS * UINT64_C(1000))) {
        return;
    }
    s_last_uart_diag_us = now;
    stm_uart_get_stats(&stats);
    ESP_LOGI(TAG,
             "STM_UART_DIAG sync=%u rx=%lu rx_reads=%lu rx_buf=%u tx=%lu tx_q=%u "
             "tx_qdrop=%lu tx_err=%lu rx_err=%lu break=%lu overflow=%lu "
             "break_recoveries=%lu drop=%lu hal=%lu",
             (unsigned)s_sync_state, (unsigned long)stats.rx_bytes,
             (unsigned long)stats.rx_task_reads, (unsigned)stats.rx_buffered,
             (unsigned long)stats.tx_bytes, (unsigned)stats.tx_queue_pending,
             (unsigned long)stats.tx_queue_drop,
             (unsigned long)stats.tx_write_errors,
             (unsigned long)stats.rx_error_events,
             (unsigned long)stats.break_events,
             (unsigned long)stats.overflow,
             (unsigned long)stats.break_recoveries,
             (unsigned long)stats.drop,
             (unsigned long)stats.hal_error);
}

/**
 * @brief 检查已同步会话的 STM 帧 freshness，超时则回到完整同步恢复路径。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param now 当前 S3 单调时间，单位 us。
 * @return 返回值：无（void）；未同步、尚无首帧或未超时时不动作。
 * 调用方式：服务任务周期调用；超时阈值来自安全配置而非日志宏。
 * 线程约束：超时时会调用阻塞恢复并撤销运动事务，仅服务 owner 调用，禁止 ISR/GATT。
 */
static void command_bridge_check_stm_liveness(uint64_t now)
{
    const uint64_t timeout_us =
        (uint64_t)SMARTCAR_SERVICE_STM_FRAME_TIMEOUT_MS * UINT64_C(1000);

    if (s_sync_state != SMARTCAR_SYNC_SYNCED || s_last_stm_frame_us == 0U ||
        now - s_last_stm_frame_us < timeout_us) {
        return;
    }
    ESP_LOGW(TAG, "STM SRP receive timeout age_ms=%llu; restarting sync",
             (unsigned long long)((now - s_last_stm_frame_us) / UINT64_C(1000)));
    command_bridge_restart_sync("STM_RX_TIMEOUT");
}

/**
 * @brief 处理 srp_link 的 BUS_OFF 电平回调并安排一次延迟恢复。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param context link 配置上下文，当前忽略，允许 NULL。
 * @return 返回值：无（void）；本地 latch 已置位时幂等返回，抵御 tick 的 BUS_OFF 电平重复回调。
 * 调用方式：由 srp_link_tick() 在服务任务调用栈同步触发；立即关闭同步门并 flush UART。
 * 线程约束：只允许 link/service owner 上下文；不得从其他任务或 ISR 手工调用。
 */
static void command_bridge_bus_off(void *context)
{
    (void)context;
    if (s_bus_off_latched) {
        return;
    }
    s_bus_off_latched = true;
    stm_uart_recover();
    stm_uart_set_sync_state(false);
    s_sync_state = SMARTCAR_SYNC_UART_READY;
    s_sync_attempts = 0U;
    s_next_sync_us = now_us() + UINT64_C(100000);
    s_bus_off_recovery_pending = true;
    s_bus_off_recovery_at_us = now_us() +
        ((uint64_t)SMARTCAR_SERVICE_BUS_OFF_RECOVERY_MS * UINT64_C(1000));
    ESP_LOGE(TAG, "BUS_OFF: UART2 receive queue flushed");
}

/**
 * @brief 根据请求类型和序号构造 SRP 快速 ACK/ERROR 响应并尝试发回 STM。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param request 已通过 parser/link 分发的只读请求帧；NULL 时直接返回，不保留指针。
 * @param is_error 非零选择 SRP ERROR，0 选择 ACK；本函数不进一步规范化该值。
 * @param status_code 写入快速响应 payload 的状态码。
 * @return 返回值：无（void）；当前忽略 srp_link_send_fast_response() 返回值，发送失败可能静默丢弃。
 * 调用方式：command_bridge_on_frame() 对 ACK_REQUIRED 的 SYS_CONFIG、标定或未知命令同步调用。
 * 线程约束：会复用全局 link 并可能阻塞 UART，只允许服务/link owner；禁止 ISR/GATT 回调、递归 link 或并发调用。
 */
static void command_bridge_send_response(const srp_frame_t *request,
                                         uint8_t is_error, uint8_t status_code)
{
    if (request == NULL) {
        return;
    }
    (void)srp_link_send_fast_response(&s_link, SRP_PRIORITY_COMMAND,
                                       0U, is_error, request->type,
                                       request->sequence, status_code,
                                       (uint32_t)(now_us() / UINT64_C(1000)));
}

/**
 * @brief 校验 DualAHRS schema=2 payload，并分别尝试转发完整 SRP 帧到 sink 和原始 payload 到 App。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param frame 已通过 SRP parser/link 的只读逻辑帧；payload 仅在当前回调栈借用，允许 NULL。
 * @return 长度/schema 非法或 App FFE2 提交失败返回 false；App 通知提交成功返回 true，sink 拒绝/编码失败不改变该返回值。
 * 调用方式：relay_telemetry() 收到 SRP_MSG_ID_ATTITUDE 时同步调用；sink 得到栈上编码帧，必须在返回前复制。
 * 线程约束：服务任务单 owner；使用约 SRP_MAX_FRAME_SIZE 的栈缓冲并同步调用 sink/BLE，禁止 ISR、GATT 回调、保留输入/输出指针或递归服务。
 */
static bool relay_dual_attitude(const srp_frame_t *frame)
{
    if (frame == NULL || frame->payload == NULL ||
        frame->length != SRP_PAYLOAD_DUAL_AHRS_SIZE) {
        ++s_dual_len_reject;
        return false;
    }
    if (frame->payload[0] != SRP_DUAL_AHRS_SCHEMA || frame->payload[2] != 0U ||
        frame->payload[3] != 0U) {
        ++s_dual_schema_reject;
        return false;
    }
    if (s_telemetry_sink != NULL) {
        uint8_t encoded[SRP_MAX_FRAME_SIZE];
        uint16_t encoded_length = 0U;

        if (srp_encode_frame(frame, encoded, sizeof(encoded),
                             &encoded_length) == SRP_CODEC_OK) {
            (void)s_telemetry_sink(SRP_MSG_ID_ATTITUDE, encoded, encoded_length,
                                    (uint32_t)(now_us() / UINT64_C(1000)),
                                    s_telemetry_sink_context);
        }
    }
    if (!s3_ble_is_ready()) {
        ++s_dual_ble_not_ready;
    }
    if (!notify_app_frame(SC_APP_TYPE_ATTITUDE, frame->payload, frame->length)) {
        ++s_dual_notify_drop;
        return false;
    }
    return true;
}

/**
 * @brief 按消息 ID 校验 STM 遥测/BOOT_INFO，推进同步并转发到已注册 sink 或 App。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param frame 已通过 SRP parser/link 的只读逻辑帧；payload 只在调用期间借用，NULL 时丢弃。
 * @param message_id 用于选择严格 payload 契约和 App 类型的 SRP 消息 ID，通常等于 frame->type。
 * @return 返回值：无（void）；长度/schema/有限值/握手字段非法时静默丢弃；sink 返回值及多数 App 通知结果当前被忽略。
 * 调用方式：command_bridge_on_frame() 对白名单遥测和 RSP_BOOT_INFO 调用；成功同步只放行本地 UART 运动门，不代表硬件验收。
 * 线程约束：服务任务单 owner；可能使用 SRP_MAX_FRAME_SIZE 栈缓冲、同步调用借用 sink 及 BLE/UART，禁止 ISR/GATT 回调、并发或保留帧指针。
 */
static void relay_telemetry(const srp_frame_t *frame, uint16_t message_id)
{
    if (frame == NULL || frame->payload == NULL) {
        return;
    }
#if SMARTCAR_BMI323_DEBUG_ONLY
    (void)message_id;
#else
    uint8_t app_type;

    switch (message_id) {
    case SRP_MSG_ID_RSP_BOOT_INFO: {
        const uint8_t attempts = s_sync_attempts;
        const bool was_synced = s_sync_state == SMARTCAR_SYNC_SYNCED;

        if (frame->length != SRP_PAYLOAD_RSP_BOOT_INFO_SIZE ||
            frame->payload == NULL ||
            frame->priority != SRP_PRIORITY_COMMAND ||
            frame->flags != SRP_FLAG_STREAM_DATA ||
            frame->payload[0] != SRP_PROTOCOL_VERSION_MAJOR ||
            frame->payload[1] != SRP_PROTOCOL_VERSION_MINOR ||
            frame->payload[2] != SRP_STM_STATE_HOST_SYNCED ||
            frame->payload[3] != SRP_SYNC_FLAG_VERSION_OK ||
            frame->payload[5] != 0U || frame->payload[6] != 0U ||
            frame->payload[7] != 0U ||
            frame->payload[4] != s_sync_tx_sequence) {
            ESP_LOGW(TAG, "invalid RSP_BOOT_INFO ignored");
            return;
        }
        s_sync_state = SMARTCAR_SYNC_SYNCED;
        s_bus_off_latched = false;
        stm_uart_set_sync_state(true);
        s_last_stm_frame_us = now_us();
        s_next_sync_us = s_last_stm_frame_us +
                         ((uint64_t)SMARTCAR_SERVICE_SYNC_HEARTBEAT_PERIOD_MS *
                          UINT64_C(1000));
        if (!was_synced) {
            command_bridge_notify_sync_status(true, false);
            ESP_LOGI(TAG, "SRP SYNCED after %u attempt(s)",
                     (unsigned)(attempts == 0U ? 1U : attempts));
        }
        return;
    }
    case SRP_MSG_ID_ATTITUDE:
        (void)relay_dual_attitude(frame);
        return;
    case SRP_MSG_ID_IMU_CAL_STATUS:
        if (frame->length != SRP_PAYLOAD_IMU_CAL_STATUS_SIZE) {
            return;
        }
        app_type = UINT8_C(0x12);
        break;
    case SRP_MSG_ID_IMU_TELEMETRY:
        if (frame->length != SRP_PAYLOAD_IMU_TELEMETRY_SIZE ||
            (frame->payload[0] != SRP_IMU_SENSOR_LSM303 &&
             frame->payload[0] != SRP_IMU_SENSOR_BMI323)) {
            return;
        }
        app_type = UINT8_C(0x27);
        break;
    case SRP_MSG_ID_RADAR_STATUS:
        if (frame->length != SRP_PAYLOAD_RADAR_STATUS_SIZE) {
            return;
        }
        app_type = SC_APP_TYPE_RADAR_STATUS;
        break;
    case SRP_MSG_ID_WHEEL_SPEED_STATUS:
        if (frame->length != SRP_PAYLOAD_WHEEL_SPEED_STATUS_SIZE ||
            !srp_wire_read_f32_array_le(frame->payload, frame->length,
                                         (float[4]){0}, 4U)) {
            return;
        }
        app_type = SC_APP_TYPE_WHEEL_SPEED_STATUS;
        break;
    case SRP_MSG_ID_CHASSIS_STATE: {
        float values[4] = {0};

        if (frame->length != SRP_PAYLOAD_CHASSIS_STATE_SIZE ||
            frame->payload[0] != SRP_CHASSIS_STATE_SCHEMA ||
            (frame->payload[1] & UINT8_C(0xF0)) != 0U ||
            frame->payload[2] != 0U || frame->payload[3] != 0U ||
            !srp_wire_read_f32_array_le(&frame->payload[8], 16U,
                                         values, 4U) ||
            !isfinite(values[0]) || !isfinite(values[1]) ||
            !isfinite(values[2]) || !isfinite(values[3])) {
            return;
        }
        app_type = SC_APP_TYPE_CHASSIS_STATE;
        break;
    }
    case SRP_MSG_ID_WHEEL_CONTROL_STATUS: {
        float values[9] = {0};

        if (frame->length != SRP_PAYLOAD_WHEEL_CONTROL_STATUS_SIZE ||
            frame->payload[0] != SRP_WHEEL_CONTROL_STATUS_SCHEMA ||
            (frame->payload[1] != SRP_CHASSIS_MODE_DIFF &&
             frame->payload[1] != SRP_CHASSIS_MODE_WHEEL_INDEPENDENT) ||
            frame->payload[2] != 0U || frame->payload[3] != 0U ||
            !srp_wire_read_f32_array_le(&frame->payload[8], 4U, values, 1U) ||
            !srp_wire_read_f32_array_le(&frame->payload[12], 16U,
                                         &values[1], 4U) ||
            !srp_wire_read_f32_array_le(&frame->payload[28], 16U,
                                         &values[5], 4U) ||
            !isfinite(values[0]) || values[0] < 0.0f || values[0] > 4.0f) {
            return;
        }
        app_type = SC_APP_TYPE_WHEEL_CONTROL_STATUS;
        break;
    }
    case SRP_MSG_ID_POWER_STATUS:
        if (frame->length != SRP_PAYLOAD_POWER_STATUS_SIZE ||
            !isfinite(srp_wire_read_f32_le(frame->payload))) {
            return;
        }
        app_type = SC_APP_TYPE_POWER_STATUS;
        break;
    default:
        return;
    }
    if (s_telemetry_sink != NULL &&
        (message_id == SRP_MSG_ID_WHEEL_SPEED_STATUS ||
         message_id == SRP_MSG_ID_CHASSIS_STATE ||
         message_id == SRP_MSG_ID_IMU_TELEMETRY)) {
        uint8_t encoded[SRP_MAX_FRAME_SIZE];
        uint16_t encoded_length = 0U;

        if (srp_encode_frame(frame, encoded, sizeof(encoded),
                             &encoded_length) == SRP_CODEC_OK) {
            (void)s_telemetry_sink(message_id, encoded, encoded_length,
                                    (uint32_t)(now_us() / UINT64_C(1000)),
                                    s_telemetry_sink_context);
        }
    }
    (void)notify_app_frame(app_type, frame->payload, frame->length);
#endif
}

/**
 * @brief 将 RADAR_PWM_READY 的 SRP 完成结果转交雷达标定状态机。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param result 链路最终结果。
 * @param status_code STM 快速响应状态码。
 * @param context 事务上下文，当前忽略，允许 NULL。
 * @return 返回值：无（void）。
 * 调用方式：由 srp_link receive/tick 在服务任务内同步调用。
 * 线程约束：必须与 radar_calibration_manager 主状态机串行，禁止递归 link 或 ISR 调用。
 */
static void command_bridge_ready_response(srp_link_tx_result_t result,
                                          uint8_t status_code, void *context)
{
    (void)context;
    radar_calibration_manager_on_ready_response(result, status_code);
}

/**
 * @brief 作为雷达标定 manager transport，提交 ACK_REQUIRED 的 RADAR_PWM_READY。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param speed_percent 标定 PWM 百分比，当前 manager 仅提交 0。
 * @param context manager 登记上下文，当前忽略，允许 NULL。
 * @return 0 表示 srp_link 接受；负值表示未提交，manager 将回同步等待。
 * 调用方式：由 radar_calibration_manager_step() 在服务任务中同步调用。
 * 线程约束：复用全局 link，只允许服务 owner 调用；不得递归或从 ISR/GATT 调用。
 */
static int command_bridge_send_radar_pwm_ready(uint8_t speed_percent, void *context)
{
    const uint8_t payload[SRP_PAYLOAD_RADAR_PWM_READY_SIZE] = {speed_percent};

    (void)context;
    return srp_link_send(&s_link, SRP_PRIORITY_COMMAND,
                          SRP_NODE_STM32H757, SRP_MSG_ID_RADAR_PWM_READY,
                          SRP_FLAG_ACK_REQUIRED, payload, sizeof(payload),
                          (uint32_t)(now_us() / UINT64_C(1000)),
                          command_bridge_ready_response, NULL);
}

/**
 * @brief 分发 srp_link 交付的 STM 业务帧并生成必要 ACK/遥测/标定事件。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param frame 已通过 codec/link 处理的只读逻辑帧；允许 NULL，payload 仅在回调期间有效。
 * @param context link 配置上下文，当前忽略，允许 NULL。
 * @return 返回值：无（void）；未知消息在需要 ACK 时尝试返回 INVALID_PARAM，否则丢弃。
 * 调用方式：由 srp_link_receive() 在 smartcar_service 任务内同步调用。
 * 线程约束：会修改同步/标定状态并可能同步发送响应；不得保留 payload、递归 link 或并发调用。
 */
static void command_bridge_on_frame(const srp_frame_t *frame, void *context)
{
    const uint16_t message_id = frame == NULL ? 0U : frame->type;
    const bool ack_required = frame != NULL &&
        (frame->flags & SRP_FLAG_ACK_REQUIRED) != 0U;
    bool admitted = false;

    (void)context;
    if (frame == NULL) {
        return;
    }

    switch (message_id) {
    case SRP_MSG_ID_RSP_BOOT_INFO:
    case SRP_MSG_ID_ATTITUDE:
    case SRP_MSG_ID_IMU_CAL_STATUS:
    case SRP_MSG_ID_IMU_TELEMETRY:
    case SRP_MSG_ID_RADAR_STATUS:
    case SRP_MSG_ID_WHEEL_SPEED_STATUS:
    case SRP_MSG_ID_CHASSIS_STATE:
    case SRP_MSG_ID_WHEEL_CONTROL_STATUS:
    case SRP_MSG_ID_POWER_STATUS:
        relay_telemetry(frame, message_id);
        return;
    case SRP_MSG_ID_LOG:
        log_bridge_handle(frame);
        return;
    case SRP_MSG_ID_SYS_CONFIG: {
        uint32_t baud_rate = 0U;
        const bool valid = command_bridge_decode_baudrate(frame, &baud_rate);

        if (ack_required) {
            command_bridge_send_response(frame, valid ? 0U : 1U,
                                         valid ? SRP_FAST_RESP_OK :
                                                 SRP_FAST_RESP_INVALID_PARAM);
        }
        if (valid) {
            s_baud_change_pending = true;
            s_baud_change_value = baud_rate;
            s_baud_change_due_us = now_us() +
                                   ((uint64_t)SMARTCAR_SERVICE_BAUD_SWITCH_GUARD_MS *
                                    UINT64_C(1000));
        }
        return;
    }
    case SRP_MSG_ID_BOOT_READY:
#if SMARTCAR_BMI323_DEBUG_ONLY
        admitted = false;
#else
        admitted = radar_calibration_manager_on_boot_ready(frame->payload, frame->length);
#endif
        break;
    case SRP_MSG_ID_CAL_EVENT:
#if SMARTCAR_BMI323_DEBUG_ONLY
        admitted = false;
#else
        admitted = radar_calibration_manager_on_cal_event(frame->payload, frame->length);
#endif
        break;
    default:
        if (ack_required) {
            command_bridge_send_response(frame, 1U, SRP_FAST_RESP_INVALID_PARAM);
        }
        return;
    }

    if (ack_required) {
        command_bridge_send_response(frame, admitted ? 0U : 1U,
                                     admitted ? SRP_FAST_RESP_OK : SRP_FAST_RESP_BUSY);
    }
}

/**
 * @brief 记录 STM 帧到达时间并把 parser 完整帧同步交给 srp_link。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param frame parser 借用逻辑帧；允许 NULL，payload 只在当前 feed 回调期间有效。
 * @param context parser 上下文，当前忽略，允许 NULL。
 * @return 返回值：无（void）。
 * 调用方式：由 srp_parser_feed() 在服务任务中同步触发，不绕过 link ACK/安全状态。
 * 线程约束：同一 parser/link 单 owner；不得保留 frame 或从其他任务并发调用。
 */
static void command_bridge_parsed_frame(const srp_frame_t *frame, void *context)
{
    (void)context;
    if (frame != NULL) {
        s_last_stm_frame_us = now_us();
    }
    srp_link_receive(&s_link, frame);
}

/**
 * @brief 对结构完整的 CRC 错误候选帧重算 CRC，并输出有限长度的原始十六进制诊断。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data parser 借用的候选帧字节；允许 NULL，仅在调用期间读取。
 * @param length data 字节数；必须与头内 payload 长度严格一致才输出。
 * @return 返回值：无（void）；magic、最小长度、payload 上限或总长不符时不记录。
 * 调用方式：仅由 command_bridge_on_parser_error() 处理 CRC 错误时同步调用；最多打印 SRP_DEBUG_RAW_FRAME_MAX_BYTES。
 * 线程约束：服务/parser 单 owner；使用有界栈缓冲并调用 snprintf/ESP_LOG，禁止 ISR、GATT 回调、并发或保留 data。
 */
static void command_bridge_log_crc_debug(const uint8_t *data, size_t length)
{
    uint16_t payload_length;
    uint16_t expected_crc;
    uint16_t received_crc;
    size_t expected_length;
    size_t raw_length = 0U;
    const size_t raw_count = length < SRP_DEBUG_RAW_FRAME_MAX_BYTES
                                 ? length
                                 : SRP_DEBUG_RAW_FRAME_MAX_BYTES;
    char raw_frame[(SRP_DEBUG_RAW_FRAME_MAX_BYTES * 3U) + 1U];

    if (data == NULL || length < (size_t)SRP_HEADER_SIZE + SRP_TRAILER_SIZE ||
        data[0] != SRP_MAGIC_BYTE0 || data[1] != SRP_MAGIC_BYTE1) {
        return;
    }
    payload_length = (uint16_t)data[2] | ((uint16_t)data[3] << 8U);
    expected_length = (size_t)SRP_HEADER_SIZE + payload_length + SRP_TRAILER_SIZE;
    if (payload_length > SRP_MAX_PAYLOAD || length != expected_length) {
        return;
    }
    expected_crc = srp_crc16_ccitt_false(&data[2], 6U + payload_length);
    received_crc = (uint16_t)data[8U + payload_length] |
                   ((uint16_t)data[9U + payload_length] << 8U);
    raw_frame[0] = '\0';
    for (size_t index = 0U; index < raw_count; ++index) {
        const int written = snprintf(&raw_frame[raw_length],
                                     sizeof(raw_frame) - raw_length,
                                     index == 0U ? "%02X" : " %02X",
                                     (unsigned)data[index]);
        if (written < 0 || (size_t)written >= sizeof(raw_frame) - raw_length) {
            break;
        }
        raw_length += (size_t)written;
    }
    ESP_LOGW(TAG,
             "[SRP_DEBUG] Expected CRC=0x%04X, Received CRC=0x%04X, "
             "Raw Frame: %s%s",
             (unsigned)expected_crc, (unsigned)received_crc, raw_frame,
             length > raw_count ? " ..." : "");
}

/**
 * @brief 对 MAGIC/LENGTH/HEADER 三类 parser 错误输出最多每 2 秒一次的头诊断。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param error parser 错误枚举；其他错误类型直接忽略。
 * @return 返回值：无（void）；限频窗口内不输出，也不改变 parser/link 错误计数。
 * 调用方式：command_bridge_on_parser_error() 每次错误先调用，读取 s_parser 的最近丢弃字节和状态快照。
 * 线程约束：无锁读写服务静态诊断状态并调用 ESP_LOG，只允许 parser/service owner；禁止 ISR/GATT 回调或并发调用。
 */
static void command_bridge_log_header_fail(srp_parser_error_t error)
{
    const uint64_t now = now_us();

    if (error != SRP_PARSER_ERROR_MAGIC && error != SRP_PARSER_ERROR_LENGTH &&
        error != SRP_PARSER_ERROR_HEADER) {
        return;
    }
    if (s_last_header_fail_log_us != 0U &&
        now - s_last_header_fail_log_us < UINT64_C(2000000)) {
        return;
    }
    s_last_header_fail_log_us = now;
    ESP_LOGW(TAG,
             "[SRP_HEADER_FAIL] drop_byte=0x%02X expected_state=%d error=%u",
             (unsigned)s_parser.last_drop_byte,
             (int)s_parser.last_error_state, (unsigned)error);
}

/**
 * @brief 统计 parser 错误、输出有界诊断，并把非 MAGIC 噪声错误计入 link REC。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param error parser 错误类型。
 * @param data 错误候选帧的借用缓冲，可能为 NULL，只在回调期间有效。
 * @param length data 的当前字节数。
 * @param context parser 上下文，当前忽略，允许 NULL。
 * @return 返回值：无（void）。
 * 调用方式：由 srp_parser_feed() 在服务任务中同步触发；随机 MAGIC 搜索噪声不推高 REC。
 * 线程约束：可能格式化日志并修改 link 计数，只允许 parser/service owner 调用。
 */
static void command_bridge_on_parser_error(srp_parser_error_t error,
                                           const uint8_t *data, size_t length,
                                           void *context)
{
    (void)context;
    ++s_parser_errors;
    command_bridge_log_header_fail(error);
    if (error == SRP_PARSER_ERROR_CRC && data != NULL && length >= SRP_HEADER_SIZE) {
        if ((uint8_t)data[6] == SRP_MSG_ID_ATTITUDE) {
            ++s_dual_crc_reject;
        }
        command_bridge_log_crc_debug(data, length);
    }
    /* A random preamble byte is expected while seeking AA 55. It is useful
     * for diagnostics but must not accumulate REC toward SRP BUS_OFF. */
    if (error != SRP_PARSER_ERROR_MAGIC) {
        srp_link_report_parser_error(&s_link, error);
    }
    if (error != SRP_PARSER_ERROR_MAGIC && error != SRP_PARSER_ERROR_LENGTH) {
        ESP_LOGW(TAG, "RX parser error=%u bytes=%u rec=%u tec=%u",
                 (unsigned)error, (unsigned)length,
                 (unsigned)srp_link_get_rec(&s_link),
                 (unsigned)srp_link_get_tec(&s_link));
    }
}

/**
 * @brief 在 BLE FFE1 写事件中把有限字节复制到 smartcar_service RX 队列。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data GATT 拥有的只读写入缓冲，只在回调期间有效。
 * @param length 本次写入字节数，必须为 1..队列项容量。
 * @param context BLE 注册上下文，当前忽略，允许 NULL。
 * @return 返回值：无（void）；参数/队列无效或零等待入队失败时累计 dropped。
 * 调用方式：由 Bluedroid GATT 任务同步调用，只复制后交给服务任务解析。
 * 线程约束：GATT 任务上下文，不是硬件 ISR；不得阻塞、解析、保留 data 或直接驱动电机。
 */
static void service_ble_rx_enqueue(const uint8_t *data, size_t length, void *context)
{
    smartcar_ble_rx_item_t item;

    (void)context;
    if (s_ble_rx_queue == NULL || data == NULL || length == 0U ||
        length > sizeof(item.bytes)) {
        ++s_ble_rx_dropped;
        return;
    }
    command_bridge_saturating_increment(&s_ble_rx_received);
    item.length = (uint16_t)length;
    item.connection_epoch = s_ble_connection_epoch;
    (void)memcpy(item.bytes, data, length);
    if (xQueueSend(s_ble_rx_queue, &item, 0U) != pdPASS) {
        ++s_ble_rx_dropped;
    }
}

/**
 * @brief 构造两字节 App V1 ACK payload 并尝试通过 FFE2 发送。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param acknowledged_type 被确认的 App V1 业务类型。
 * @param result SC_APP_ACK_OK 或拒绝结果；本函数不校验取值范围。
 * @return 返回值：无（void）；忽略 notify_app_frame() 结果，BLE 未就绪/拥塞时可能静默丢弃。
 * 调用方式：V1 命令网关准入失败、SRP 事务完成或零速 stream 已排队时由服务任务调用。
 * 线程约束：复用全局 App TX 缓冲并调用 BLE，只允许服务任务 owner；禁止 ISR/GATT 回调、递归或并发调用。
 */
static void send_app_ack(uint8_t acknowledged_type, uint8_t result)
{
    const uint8_t payload[2] = {acknowledged_type, result};
    const bool sent = notify_app_frame(SC_APP_TYPE_ACK, payload, sizeof(payload));

    s_app_last_ack_type = acknowledged_type;
    s_app_last_ack_result = result;
    s_app_last_ack_stage = SC_APP_V2_STAGE_GATEWAY_ADMITTED;
    s_app_last_ack_sequence = 0U;

    if (result == SC_APP_ACK_OK) {
        command_bridge_saturating_increment(&s_app_ack_ok);
    } else {
        command_bridge_saturating_increment(&s_app_ack_rejected);
    }
    if (!sent) {
        command_bridge_saturating_increment(&s_app_ack_dropped);
    }
}

/**
 * @brief 将一个 16 位无符号值按小端序写入调用方缓冲。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param out 至少 2 字节的可写缓冲；不得为 NULL。
 * @param value 待编码数值。
 * @return 返回值：无（void）。
 * 调用方式：仅用于构造 App V2 ACK 固定字段；调用方负责容量和字段偏移。
 * 线程约束：纯写入、可重入、不阻塞；不做边界检查，调用期间调用方独占 out。
 */
static void append_u16_le(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)(value >> 8U);
}

/**
 * @brief 将一个 32 位无符号值按小端序写入调用方缓冲。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param out 至少 4 字节的可写缓冲；不得为 NULL。
 * @param value 待编码数值。
 * @return 返回值：无（void）。
 * 调用方式：仅用于构造 App V2 会话 ID、序号和能力字段；调用方负责容量和偏移。
 * 线程约束：纯写入、可重入、不阻塞；不做边界检查，调用期间调用方独占 out。
 */
static void append_u32_le(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)((value >> 8U) & 0xFFU);
    out[2] = (uint8_t)((value >> 16U) & 0xFFU);
    out[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

/**
 * @brief 从调用方缓冲读取一个小端 16 位无符号值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data 至少 2 字节的只读缓冲；不得为 NULL。
 * @return 解码后的 uint16_t 数值。
 * 调用方式：App V2 命令长度已校验后读取 valid_for_ms。
 * 线程约束：纯读取、可重入、不阻塞；不做 NULL/边界检查，data 仅需在调用期间有效。
 */
static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/**
 * @brief 从调用方缓冲读取一个小端 32 位无符号值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data 至少 4 字节的只读缓冲；不得为 NULL。
 * @return 解码后的 uint32_t 数值。
 * 调用方式：App V2 payload 长度已校验后读取 session ID 和 sequence。
 * 线程约束：纯读取、可重入、不阻塞；不做 NULL/边界检查，data 仅需在调用期间有效。
 */
static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

/**
 * @brief 使用 App V2 包络编码会话响应，并通过共享静态缓冲向 FFE2 提交通知。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param type App V2 响应类型。
 * @param payload 只读 payload；length 为 0 时允许 NULL，函数不保留指针。
 * @param length payload 字节数，不得超过 App 帧上限。
 * @return 编码成功且 BLE 接受全部通知分片时为 true，否则为 false；true 不代表 App 收讫。
 * 调用方式：HELLO/HEARTBEAT/COMMAND ACK 构造函数在服务任务中调用。
 * 线程约束：使用全局 s_app_tx_frame，不可重入或并发；可能调用 Bluedroid，禁止 ISR/GATT 回调。
 */
static bool notify_app_v2(uint8_t type, const uint8_t *payload, uint16_t length)
{
    uint16_t frame_length = 0U;

    if (sc_app_frame_encode_version(SC_APP_FRAME_VERSION_V2, type, payload,
                                    length, s_app_tx_frame,
                                    sizeof(s_app_tx_frame), &frame_length) != 0) {
        return false;
    }
    return s3_ble_notify_send(s_app_tx_frame, frame_length) == ESP_OK;
}

/**
 * @brief 使用当前 V2 session ID、心跳周期、TTL 和能力位构造 HELLO_ACK。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；忽略 FFE2 通知结果，BLE 未就绪/拥塞时响应可能丢失。
 * 调用方式：app_command_on_frame() 接受合法 V2 HELLO 并建立新 session 后同步调用。
 * 线程约束：读取无锁全局 session 并复用 App TX 缓冲，只允许服务任务 owner；禁止 ISR/GATT 回调或并发调用。
 */
static void app_v2_send_hello_ack(void)
{
    uint8_t payload[13] = {0};

    append_u32_le(&payload[1], s_app_v2_session.session_id);
    append_u16_le(&payload[5], (uint16_t)APP_V2_HEARTBEAT_PERIOD_MS);
    append_u16_le(&payload[7], (uint16_t)APP_V2_SESSION_TTL_MS);
    append_u32_le(&payload[9], 0x00000007U);
    payload[0] = SC_APP_FRAME_VERSION_V2;
    (void)notify_app_v2(SC_APP_V2_TYPE_HELLO_ACK, payload, sizeof(payload));
}

/**
 * @brief 使用当前 session ID 回送指定序号的 V2 HEARTBEAT_ACK。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param sequence App HEARTBEAT 携带的序号，原样按小端写回。
 * @return 返回值：无（void）；忽略 FFE2 通知结果，发送失败不回滚 session 活跃时间。
 * 调用方式：仅在 session ID 匹配的 V2 HEARTBEAT 路径同步调用。
 * 线程约束：读取全局 session 并复用 App TX 缓冲，只允许服务任务 owner；禁止 ISR/GATT 回调或并发调用。
 */
static void app_v2_send_heartbeat_ack(uint32_t sequence)
{
    uint8_t payload[9] = {0};

    append_u32_le(&payload[0], s_app_v2_session.session_id);
    append_u32_le(&payload[4], sequence);
    payload[8] = SC_APP_V2_RESULT_OK;
    (void)notify_app_v2(SC_APP_V2_TYPE_HEARTBEAT_ACK, payload, sizeof(payload));
}

/**
 * @brief 构造并尝试发送一条 V2 COMMAND_ACK，描述网关或 STM 接受阶段。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param sequence 被确认的 V2 命令序号。
 * @param app_type 被确认的内层 App 业务类型。
 * @param result V2 结果码。
 * @param stage 网关准入、STM 接受或停止已排队阶段码。
 * @return 返回值：无（void）；忽略 FFE2 通知结果，响应可能在断连/拥塞时丢失。
 * 调用方式：会话校验拒绝、重复序号重放或事务完成路径在服务任务中调用。
 * 线程约束：读取全局 session ID 并复用 App TX 缓冲，只允许服务任务 owner；禁止 ISR/GATT 回调或并发调用。
 */
static void app_v2_send_command_ack(uint32_t sequence, uint8_t app_type,
                                    uint8_t result, uint8_t stage)
{
    uint8_t payload[11] = {0};

    append_u32_le(&payload[0], s_app_v2_session.session_id);
    append_u32_le(&payload[4], sequence);
    payload[8] = app_type;
    payload[9] = result;
    payload[10] = stage;
    const bool sent = notify_app_v2(SC_APP_V2_TYPE_COMMAND_ACK, payload, sizeof(payload));

    s_app_last_ack_type = app_type;
    s_app_last_ack_result = result;
    s_app_last_ack_stage = stage;
    s_app_last_ack_sequence = sequence;

    if (result == SC_APP_V2_RESULT_OK) {
        command_bridge_saturating_increment(&s_app_ack_ok);
    } else {
        command_bridge_saturating_increment(&s_app_ack_rejected);
    }
    if (!sent) {
        command_bridge_saturating_increment(&s_app_ack_dropped);
    }
}

/**
 * @brief 在仍为当前 V2 序号时缓存最后一条 COMMAND_ACK，并立即尝试发送。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param sequence 被确认命令序号。
 * @param app_type 被确认内层业务类型。
 * @param result V2 结果码。
 * @param stage V2 接受阶段码。
 * @return 返回值：无（void）；session 不活跃或序号已变化时不缓存，但仍尝试发送 ACK。
 * 调用方式：send_command_result() 的有效 V2 session 分支调用，用于重复同序号命令的 ACK 重放。
 * 线程约束：无锁修改全局 session 并调用 BLE，只允许服务任务 owner；禁止 ISR/GATT 回调、递归或并发调用。
 */
static void app_v2_remember_command_ack(uint32_t sequence, uint8_t app_type,
                                        uint8_t result, uint8_t stage)
{
    if (s_app_v2_session.active && sequence == s_app_v2_session.last_sequence) {
        s_app_v2_session.have_last_ack = true;
        s_app_v2_session.last_ack_sequence = sequence;
        s_app_v2_session.last_ack_type = app_type;
        s_app_v2_session.last_ack_result = result;
        s_app_v2_session.last_ack_stage = stage;
    }
    app_v2_send_command_ack(sequence, app_type, result, stage);
}

static bool start_motion_command(const smartcar_motion_command_t *command);

/**
 * @brief 按命令来源版本把结果映射为 V1 ACK 或当前有效 session 的 V2 COMMAND_ACK。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param app_version SC_APP_FRAME_VERSION_V2 选择 V2，其他值按 V1 处理。
 * @param app_type 被确认的 App 业务类型。
 * @param session_id V2 session ID；V1 路径忽略。
 * @param sequence V2 命令序号；V1 路径忽略。
 * @param result App 结果码。
 * @param stage V2 阶段码；V1 路径忽略。
 * @return 返回值：无（void）；V2 session 已失效或 ID 不匹配时静默丢弃结果，BLE 发送失败也不重试。
 * 调用方式：运动/PID/波特率事务准入、完成、过期或停止排队路径由服务任务调用。
 * 线程约束：读写全局 V2 ACK 缓存并复用 BLE TX 缓冲，只允许服务任务 owner；禁止 ISR/GATT 回调或并发调用。
 */
static void send_command_result(uint8_t app_version, uint8_t app_type,
                                uint32_t session_id, uint32_t sequence,
                                uint8_t result, uint8_t stage)
{
    if (app_version == SC_APP_FRAME_VERSION_V2) {
        if (s_app_v2_session.active && s_app_v2_session.session_id == session_id) {
            app_v2_remember_command_ack(sequence, app_type, result, stage);
        }
    } else {
        send_app_ack(app_type, result);
    }
}

/**
 * @brief 为忙时运动命令选择“主速度缩放”或“最新目标”全局缓存槽。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param message_id SRP 运动消息 ID；MASTER_SPEED 选择独立缩放槽，其他值均选择目标槽。
 * @return 指向模块内部可写静态槽的借用指针，永不返回 NULL；调用方不得长期保存或跨线程使用。
 * 调用方式：仅由 queue_motion_command() 在已有 ACK 事务 in-flight 时调用。
 * 线程约束：无锁返回全局地址，只允许服务任务 owner；函数不验证 message_id，禁止并发读写返回槽。
 */
static smartcar_motion_command_t *pending_motion_slot(uint16_t message_id)
{
    return message_id == SRP_MSG_ID_MASTER_SPEED_CMD
               ? &s_motion_pending_scale
               : &s_motion_pending_target;
}

/**
 * @brief 识别需要走显式停止语义的四轮近零目标或航向全零命令。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param command 已复制的只读运动命令；允许 NULL，函数不保留指针。
 * @return 航向命令仅在 v/yaw 均精确为 0 且 flags=NONE 时为 true；四轮速度各绝对值不大于 0.001 时为 true；其他命令为 false。
 * 调用方式：queue_motion_command() 在入 pending/in-flight 前调用；底层 float-array 解码失败或非有限值返回 false。
 * 线程约束：只读取 command 和栈上数组、可重入、不阻塞；command payload 长度必须与结构中的 length 一致。
 */
static bool motion_command_is_zero_target(const smartcar_motion_command_t *command)
{
    float speeds[4] = {0.0f};

    if (command == NULL) {
        return false;
    }
    if (command->message_id == SRP_MSG_ID_CHASSIS_HEADING_CMD) {
        return command->length == SRP_PAYLOAD_CHASSIS_HEADING_CMD_SIZE &&
               srp_wire_read_f32_le(&command->payload[0]) == 0.0f &&
               srp_wire_read_f32_le(&command->payload[4]) == 0.0f &&
               srp_wire_read_u32_le(&command->payload[8]) ==
                   SRP_CHASSIS_HEADING_FLAGS_NONE;
    }
    return command->message_id == SRP_MSG_ID_WHEEL_SPEED_CMD &&
           command->length == SRP_PAYLOAD_WHEEL_SPEED_CMD_SIZE &&
           srp_wire_read_f32_array_le(command->payload, command->length,
                                       speeds, 4U) &&
           fabsf(speeds[0]) <= 0.001f && fabsf(speeds[1]) <= 0.001f &&
           fabsf(speeds[2]) <= 0.001f && fabsf(speeds[3]) <= 0.001f;
}

/**
 * @brief 取消所有运动类 pending ACK 槽并清除当前 in-flight 标志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；不发送零速帧，也不清除两个 queued pending 目标。
 * 调用方式：停止、断链、超时或重同步边界先调用，再由外层决定是否提交零速。
 * 线程约束：直接修改全局 link/事务状态，只允许服务任务 owner 调用。
 */
static void cancel_motion_transactions(void)
{
    static const uint16_t message_ids[] = {
        SRP_MSG_ID_WHEEL_SPEED_CMD,
        SRP_MSG_ID_WHEEL_SPEED_SINGLE_CMD,
        SRP_MSG_ID_MASTER_SPEED_CMD,
        SRP_MSG_ID_CHASSIS_SPEED_CMD,
        SRP_MSG_ID_CHASSIS_HEADING_CMD
    };

    for (size_t index = 0U; index < sizeof(message_ids) / sizeof(message_ids[0]);
         ++index) {
        srp_link_cancel_message(&s_link, message_ids[index]);
    }
    s_motion_inflight.valid = false;
    s_motion_tx_in_flight = false;
}

/**
 * @brief 取消旧运动事务并按命令类型提交航向 ACK 停止或四轮零速 stream。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param command 已复制且验证为零目标的只读命令；NULL 返回 false。
 * @return 航向清零分支在 start_motion_command() 成功提交 ACK_REQUIRED 事务时为 true；四轮零速分支在 srp_link 返回 0 时为 true；发送失败均为 false。
 * 调用方式：queue_motion_command() 识别零目标后调用；轮速 stream 成功即回 App STOP_QUEUED，航向分支仍等待其 ACK 回调。
 * 线程约束：会修改 link 和全局事务状态并同步发送，只允许服务任务调用。
 */
static bool send_motion_stop(const smartcar_motion_command_t *command)
{
    bool submitted;

    if (command == NULL) {
        return false;
    }
    cancel_motion_transactions();
    if (command->message_id == SRP_MSG_ID_CHASSIS_HEADING_CMD) {
        /* Target Yaw stop remains the same ACK-required SRP command so CM7
         * can clear the heading controller explicitly. */
        submitted = start_motion_command(command);
    } else {
        /* Preserve the legacy immediate wheel-stop stream semantics. */
        submitted = srp_link_send(
            &s_link, SRP_PRIORITY_COMMAND, SRP_NODE_STM32H757,
            SRP_MSG_ID_WHEEL_SPEED_CMD, SRP_FLAG_STREAM_DATA,
            command->payload, command->length,
            (uint32_t)(now_us() / UINT64_C(1000)), NULL, NULL) == 0;
    }

    if (submitted && command->message_id != SRP_MSG_ID_CHASSIS_HEADING_CMD) {
        send_command_result(command->app_version, command->app_type,
                            command->app_session_id, command->app_sequence,
                            SC_APP_ACK_OK, SC_APP_V2_STAGE_STOP_QUEUED);
    }
    return submitted;
}

/**
 * @brief 将普通 ACK_REQUIRED App 事务的 STM 结果映射回 V1/V2 App ACK。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param result SRP 最终结果。
 * @param status_code STM 快速响应状态码。
 * @param context 可选 smartcar_app_tx_context_t；NULL 使用历史轮速 V1 ACK 类型。
 * @return 返回值：无（void）；对应 V2 session 已失效时结果可能静默丢弃。
 * 调用方式：由 srp_link receive/tick 同步触发。
 * 线程约束：服务任务/link owner 上下文；不得保留 context 或递归发送同一事务。
 */
static void app_transaction_tx_complete(srp_link_tx_result_t result,
                                        uint8_t status_code, void *context)
{
    const smartcar_app_tx_context_t *tx = context;
    const uint8_t accepted = result == SRP_LINK_TX_OK &&
                             status_code == SRP_FAST_RESP_OK;
    if (tx == NULL) {
        send_app_ack(SC_APP_TYPE_WHEEL_SPEED_CMD,
                     accepted ? SC_APP_ACK_OK : SC_APP_ACK_REJECTED);
    } else {
        send_command_result(tx->app_version, tx->app_type, tx->session_id,
                            tx->sequence,
                            accepted ? SC_APP_ACK_OK : SC_APP_ACK_REJECTED,
                            accepted ? SC_APP_V2_STAGE_STM32_ACCEPTED :
                                       SC_APP_V2_STAGE_GATEWAY_ADMITTED);
    }
}

/**
 * @brief 回送波特率配置 ACK，并在 STM 接受后安排有保护间隔的本地切换。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param result SRP 最终结果。
 * @param status_code STM 快速响应状态码。
 * @param context 可选 smartcar_app_tx_context_t，只在回调期间读取。
 * @return 返回值：无（void）；接受时只安排本地延迟切换，实际 set_baud_rate 结果稍后记录。
 * 调用方式：SYS_CONFIG ACK_REQUIRED 事务由 srp_link 同步触发。
 * 线程约束：修改全局波特率切换状态，只允许服务/link owner 调用。
 */
static void baud_config_tx_complete(srp_link_tx_result_t result,
                                    uint8_t status_code, void *context)
{
    const smartcar_app_tx_context_t *tx = context;
    const bool accepted = result == SRP_LINK_TX_OK &&
                          status_code == SRP_FAST_RESP_OK;

    if (tx != NULL) {
        send_command_result(tx->app_version, tx->app_type, tx->session_id,
                            tx->sequence,
                            accepted ? SC_APP_ACK_OK : SC_APP_ACK_REJECTED,
                            accepted ? SC_APP_V2_STAGE_STM32_ACCEPTED :
                                       SC_APP_V2_STAGE_GATEWAY_ADMITTED);
    }
    if (accepted) {
        s_baud_change_pending = true;
        s_baud_change_due_us = now_us() +
                               ((uint64_t)SMARTCAR_SERVICE_BAUD_SWITCH_GUARD_MS *
                                UINT64_C(1000));
    }
}

/**
 * @brief 完成当前运动 ACK 事务并按“目标优先、缩放其次”提交下一条缓存命令。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param result 当前运动事务的 SRP 最终结果。
 * @param status_code STM 快速响应状态码。
 * @param context srp_link 登记上下文，当前不解引用。
 * @return 返回值：无（void）；pending 槽启动失败的返回值当前被忽略，对应命令由 start_motion_command() 自行回送拒绝。
 * 调用方式：由 srp_link receive/tick 在服务任务中同步触发并回送 App 阶段结果。
 * 线程约束：直接修改全局 in-flight/pending 状态，必须由单一服务 owner 串行执行。
 */
static void motion_command_tx_complete(srp_link_tx_result_t result,
                                       uint8_t status_code, void *context)
{
    smartcar_motion_command_t completed = s_motion_inflight;
    smartcar_motion_command_t next = {0};
    const uint8_t accepted = result == SRP_LINK_TX_OK &&
                             status_code == SRP_FAST_RESP_OK;

    (void)context;
    if (command_bridge_ble_stop_requested()) {
        /* The disconnect callback may set the stop flag while srp_link_receive
         * or srp_link_tick() is dispatching this completion. Do not emit a
         * stale ACK or release a cached target into start_motion_command(). */
        s_motion_inflight.valid = false;
        s_motion_tx_in_flight = false;
        s_motion_pending_target.valid = false;
        s_motion_pending_scale.valid = false;
        return;
    }
    s_motion_inflight.valid = false;
    s_motion_tx_in_flight = false;
    send_command_result(completed.app_version, completed.app_type,
                        completed.app_session_id, completed.app_sequence,
                        accepted ? SC_APP_ACK_OK : SC_APP_ACK_REJECTED,
                        accepted ? SC_APP_V2_STAGE_STM32_ACCEPTED :
                                   SC_APP_V2_STAGE_GATEWAY_ADMITTED);

    if (s_motion_pending_target.valid) {
        next = s_motion_pending_target;
        s_motion_pending_target.valid = false;
        (void)start_motion_command(&next);
    } else if (s_motion_pending_scale.valid) {
        next = s_motion_pending_scale;
        s_motion_pending_scale.valid = false;
        (void)start_motion_command(&next);
    }
}

/**
 * @brief 复制一条已通过 App 会话校验的运动命令并提交 ACK_REQUIRED SRP 事务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param command 只读命令；必须 valid、长度不超内部 payload，且未超过 valid_until_us。
 * @return true 表示 srp_link 已接受；false 表示参数、时效或链路发送失败并回送拒绝/过期。
 * 调用方式：仅 queue/完成回调从服务任务调用；最终执行仍由 STM 本地安全门决定。
 * 线程约束：修改单一全局 in-flight 槽并可能同步 UART 发送，不可重入或并发调用。
 */
static bool start_motion_command(const smartcar_motion_command_t *command)
{
    const uint32_t connection_epoch = s_ble_connection_epoch;
    uint32_t generation;
    int result;

    if (command == NULL || !command->valid ||
        command->length > sizeof(s_motion_inflight.payload)) {
        return false;
    }
    if (command_bridge_ble_stop_requested()) {
        return false;
    }
    if (command->valid_until_us != 0U && now_us() >= command->valid_until_us) {
        send_command_result(command->app_version, command->app_type,
                            command->app_session_id, command->app_sequence,
                            SC_APP_V2_RESULT_EXPIRED,
                            SC_APP_V2_STAGE_GATEWAY_ADMITTED);
        return false;
    }
    s_motion_inflight = *command;
    generation = ++s_motion_generation;
    s_motion_inflight.generation = generation;
    s_motion_tx_in_flight = true;
    result = srp_link_send(
        &s_link, SRP_PRIORITY_COMMAND, SRP_NODE_STM32H757,
        s_motion_inflight.message_id, SRP_FLAG_ACK_REQUIRED,
        s_motion_inflight.payload, s_motion_inflight.length,
        (uint32_t)(now_us() / UINT64_C(1000)), motion_command_tx_complete,
        &s_motion_inflight);
    if (command_bridge_ble_stop_requested() ||
        connection_epoch != s_ble_connection_epoch) {
        /* If the disconnect raced the synchronous transport call, cancel the
         * just-admitted ACK slot before the next service iteration handles the
         * stop boundary. The transport gate already rejects later retries. */
        if (result == 0 && s_motion_inflight.generation == generation) {
            srp_link_cancel_message(&s_link, s_motion_inflight.message_id);
        }
        if (s_motion_inflight.generation == generation) {
            s_motion_inflight.valid = false;
            s_motion_tx_in_flight = false;
        }
        return false;
    }
    if (result != 0 && s_motion_tx_in_flight &&
        s_motion_inflight.generation == generation) {
        s_motion_tx_in_flight = false;
        s_motion_inflight.valid = false;
        send_command_result(command->app_version, command->app_type,
                            command->app_session_id, command->app_sequence,
                            SC_APP_V2_RESULT_REJECTED,
                            SC_APP_V2_STAGE_GATEWAY_ADMITTED);
    }
    return result == 0;
}

/**
 * @brief 复制运动 payload；零目标立即走停止语义，忙时分别保留最新目标和缩放命令。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param app_type App V1 业务类型。
 * @param message_id 对应 SRP 运动消息 ID。
 * @param payload 只读 payload，返回前复制；不得为 NULL。
 * @param length payload 长度，不得超过内部固定数组。
 * @param meta 可选 V2 会话/序号/时效元数据；NULL 使用 V1 语义。
 * @return true 表示已提交或写入最新值缓存；false 表示参数、链路或时效失败。
 * 调用方式：App 帧完成校验后由服务任务调用，不绕过 STM 最终安全门。
 * 线程约束：使用无锁全局 pending/in-flight 状态，只允许服务任务 owner；忙时覆盖旧缓存不会为被替换命令单独生成完成 ACK。
 */
static bool queue_motion_command(uint8_t app_type, uint16_t message_id,
                                 const uint8_t *payload, uint8_t length,
                                 const smartcar_app_command_meta_t *meta)
{
    smartcar_motion_command_t command = {0};

    if (payload == NULL || length > sizeof(command.payload)) {
        return false;
    }
    if (command_bridge_ble_stop_requested()) {
        return false;
    }
    command.valid = true;
    command.app_type = app_type;
    command.app_version = meta == NULL ? SC_APP_FRAME_VERSION : meta->version;
    command.app_session_id = meta == NULL ? 0U : meta->session_id;
    command.app_sequence = meta == NULL ? 0U : meta->sequence;
    command.valid_until_us = meta == NULL ? 0U : meta->valid_until_us;
    command.message_id = message_id;
    command.length = length;
    (void)memcpy(command.payload, payload, length);
    if (motion_command_is_zero_target(&command)) {
        s_motion_pending_target.valid = false;
        s_motion_pending_scale.valid = false;
        return send_motion_stop(&command);
    }
    if (s_motion_tx_in_flight) {
        /* Keep the newest target and scale independently. A speed update
         * cannot erase an independent MasterScale update. */
        *pending_motion_slot(message_id) = command;
        return true;
    }
    return start_motion_command(&command);
}

/**
 * @brief 在 BLE 断开事件中撤销缓存目标并通知服务任务发送停止帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param context BLE 注册上下文，当前忽略，允许 NULL。
 * @return 返回值：无（void）；本回调不直接调用 srp_link 或 UART，物理停止尚未完成。
 * 调用方式：由 Bluedroid GATT 断开事件同步触发，服务循环随后消费 stop_pending。
 * 线程约束：GATT 任务上下文；只递增 BLE epoch 并置原子事件标志，不触碰 service-owned
 *           motion/session/parser/link 状态，真正清理和停止必须留在服务任务。
 */
static void command_bridge_on_ble_disconnect(void *context)
{
    (void)context;
    /* The GATT callback only signals the service task. Any queued RX item from
     * the previous connection carries the old epoch and is discarded there. */
    s_ble_connection_epoch += 1U;
    s_ble_disconnect_stop_pending = true;
}

/**
 * @brief 撤销旧运动事务，并依次发送航向清零和四轮零速两条 SRP 安全命令。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；日志中的 queued 只表示两次 link 调用返回 0，不证明 STM、MotorBoard 或车辆已经停止。
 * 调用方式：服务任务消费 BLE 断开、会话过期或恢复停机请求时调用。
 * 线程约束：修改 link/事务全局状态并可能阻塞 UART，只允许服务任务 owner 调用。
 */
static void command_bridge_send_zero_wheel_speed(void)
{
    uint8_t heading_payload[SRP_PAYLOAD_CHASSIS_HEADING_CMD_SIZE] = {0U};
    uint8_t payload[SRP_PAYLOAD_WHEEL_SPEED_CMD_SIZE];
    const float speeds[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    int heading_result;
    int wheel_result;

    s_motion_pending_target.valid = false;
    s_motion_pending_scale.valid = false;
    cancel_motion_transactions();
    /* Local disconnect/session safety must clear the CM7 heading controller
     * using the same SRP command as the App stop path. */
    heading_result = srp_link_send(
        &s_link, SRP_PRIORITY_COMMAND, SRP_NODE_STM32H757,
        SRP_MSG_ID_CHASSIS_HEADING_CMD, SRP_FLAG_ACK_REQUIRED,
        heading_payload, sizeof(heading_payload),
        (uint32_t)(now_us() / UINT64_C(1000)), NULL, NULL);
    srp_wire_write_f32_array_le(payload, speeds, 4U);
    wheel_result = srp_link_send(&s_link, SRP_PRIORITY_COMMAND,
                                 SRP_NODE_STM32H757, SRP_MSG_ID_WHEEL_SPEED_CMD,
                                 SRP_FLAG_STREAM_DATA, payload, sizeof(payload),
                                 (uint32_t)(now_us() / UINT64_C(1000)), NULL, NULL);
    if (heading_result != 0 || wheel_result != 0) {
        ESP_LOGE(TAG, "BLE disconnect stop frame send failed heading=%d wheel=%d",
                 heading_result, wheel_result);
    } else {
        ESP_LOGI(TAG, "BLE disconnect stop frames queued heading=0x17 wheel=0x02");
    }
}

/**
 * @brief 校验 App V1/V2 会话、序号、时效、类型和 payload，再转换为 SRP 命令。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param input_frame App parser 借用逻辑帧；允许 NULL，payload 只在回调期间有效。
 * @param context parser 注册上下文，当前忽略，允许 NULL。
 * @return 返回值：无（void）；成功路径异步/同步回 ACK，非法输入回拒绝或静默丢弃协议控制帧。
 * 调用方式：由 sc_app_parser_feed() 在 smartcar_service 任务中同步触发。
 * 线程约束：可修改会话和运动事务并同步进入 SRP/BLE；不得保留 payload、递归 parser 或并发调用。
 * 安全边界：网关准入不等于车辆执行，所有运动输出仍经 STM32 最终门控。
 */
static void app_command_on_frame(const sc_app_frame_view_t *input_frame, void *context)
{
    uint8_t result = SC_APP_ACK_REJECTED;
    sc_app_frame_view_t v2_command = {0};
    smartcar_app_command_meta_t meta = {0};
    const sc_app_frame_view_t *frame = input_frame;

    (void)context;
    if (frame == NULL) {
        return;
    }
    command_bridge_saturating_increment(&s_app_valid_frames);
    if ((frame->version == SC_APP_FRAME_VERSION_V2 &&
         frame->type == SC_APP_V2_TYPE_COMMAND) ||
        (frame->version != SC_APP_FRAME_VERSION_V2 &&
         command_bridge_is_app_command_type(frame->type))) {
        command_bridge_saturating_increment(&s_app_command_frames);
    }
    if (command_bridge_ble_stop_requested()) {
        /* A frame already dequeued before disconnect is still stale even if
         * its queue item carries the current parser epoch. */
        return;
    }
    meta.version = SC_APP_FRAME_VERSION;

    if (frame->version == SC_APP_FRAME_VERSION_V2) {
        if (frame->type == SC_APP_V2_TYPE_HELLO && frame->length == 6U &&
            frame->payload != NULL && frame->payload[0] == SC_APP_FRAME_VERSION_V2 &&
            frame->payload[1] == SC_APP_FRAME_VERSION_V2) {
            s_app_v2_session.active = true;
            s_app_v2_session.session_id = ++s_app_v2_next_session_id;
            if (s_app_v2_session.session_id == 0U) {
                s_app_v2_session.session_id = ++s_app_v2_next_session_id;
            }
            s_app_v2_session.have_sequence = false;
            s_app_v2_session.last_sequence = 0U;
            s_app_v2_session.have_last_ack = false;
            s_app_v2_session.last_activity_us = now_us();
            app_v2_send_hello_ack();
            return;
        }
        if (frame->type == SC_APP_V2_TYPE_HEARTBEAT && frame->length == 8U &&
            frame->payload != NULL) {
            const uint32_t session_id = read_u32_le(&frame->payload[0]);
            const uint32_t sequence = read_u32_le(&frame->payload[4]);
            if (s_app_v2_session.active && session_id == s_app_v2_session.session_id) {
                s_app_v2_session.last_activity_us = now_us();
                app_v2_send_heartbeat_ack(sequence);
            }
            return;
        }
        if (frame->type != SC_APP_V2_TYPE_COMMAND || frame->length < 11U ||
            frame->payload == NULL) {
            return;
        }
        meta.version = SC_APP_FRAME_VERSION_V2;
        meta.session_id = read_u32_le(&frame->payload[0]);
        meta.sequence = read_u32_le(&frame->payload[4]);
        const uint16_t valid_for_ms = read_u16_le(&frame->payload[8]);
        if (!s_app_v2_session.active ||
            meta.session_id != s_app_v2_session.session_id) {
            app_v2_send_command_ack(meta.sequence, frame->payload[10],
                                    SC_APP_V2_RESULT_SESSION_INVALID,
                                    SC_APP_V2_STAGE_GATEWAY_ADMITTED);
            return;
        }
        if (valid_for_ms < 20U || valid_for_ms > 1000U) {
            app_v2_send_command_ack(meta.sequence, frame->payload[10],
                                    SC_APP_V2_RESULT_EXPIRED,
                                    SC_APP_V2_STAGE_GATEWAY_ADMITTED);
            return;
        }
        if (s_app_v2_session.have_sequence &&
            meta.sequence <= s_app_v2_session.last_sequence) {
            if (s_app_v2_session.have_last_ack &&
                meta.sequence == s_app_v2_session.last_ack_sequence) {
                app_v2_send_command_ack(s_app_v2_session.last_ack_sequence,
                                        s_app_v2_session.last_ack_type,
                                        s_app_v2_session.last_ack_result,
                                        s_app_v2_session.last_ack_stage);
            } else {
                app_v2_send_command_ack(meta.sequence, frame->payload[10],
                                        SC_APP_V2_RESULT_STALE_SEQUENCE,
                                        SC_APP_V2_STAGE_GATEWAY_ADMITTED);
            }
            return;
        }
        s_app_v2_session.last_activity_us = now_us();
        s_app_v2_session.last_sequence = meta.sequence;
        s_app_v2_session.have_sequence = true;
        meta.valid_until_us = now_us() + ((uint64_t)valid_for_ms * UINT64_C(1000));
        v2_command.version = SC_APP_FRAME_VERSION;
        v2_command.type = frame->payload[10];
        v2_command.length = (uint16_t)(frame->length - 11U);
        v2_command.payload = &frame->payload[11];
        frame = &v2_command;
    }
#if !SMARTCAR_BMI323_DEBUG_ONLY
    if (frame->type == SC_APP_TYPE_WHEEL_SPEED_CMD &&
        frame->length == SRP_PAYLOAD_WHEEL_SPEED_CMD_SIZE &&
        frame->payload != NULL) {
        float speeds[4] = {0.0f};
        const bool valid = srp_wire_read_f32_array_le(
            frame->payload, frame->length, speeds, 4U);
        if (valid && queue_motion_command(
                         frame->type, SRP_MSG_ID_WHEEL_SPEED_CMD,
                         frame->payload, (uint8_t)frame->length, &meta)) {
            return;
        }
    } else if (frame->type == SC_APP_TYPE_CHASSIS_SPEED_CMD &&
               frame->length == SRP_PAYLOAD_CHASSIS_SPEED_CMD_SIZE &&
               frame->payload != NULL) {
        const float base_speed = srp_wire_read_f32_le(&frame->payload[0]);
        const float target_yaw_rate = srp_wire_read_f32_le(&frame->payload[4]);
        bool reserved_zero = true;

        for (size_t index = 8U;
             index < SRP_PAYLOAD_CHASSIS_SPEED_CMD_SIZE; ++index) {
            if (frame->payload[index] != 0U) {
                reserved_zero = false;
                break;
            }
        }
        if (isfinite(base_speed) && isfinite(target_yaw_rate) && reserved_zero &&
            queue_motion_command(frame->type, SRP_MSG_ID_CHASSIS_SPEED_CMD,
                                 frame->payload, (uint8_t)frame->length, &meta)) {
            return;
        }
    } else if (frame->type == SC_APP_TYPE_CHASSIS_HEADING_CMD &&
               frame->length == SRP_PAYLOAD_CHASSIS_HEADING_CMD_SIZE &&
               frame->payload != NULL) {
        const float target_v_mm_s = srp_wire_read_f32_le(&frame->payload[0]);
        const float target_yaw_deg = srp_wire_read_f32_le(&frame->payload[4]);
        const uint32_t flags = srp_wire_read_u32_le(&frame->payload[8]);

        if (isfinite(target_v_mm_s) && isfinite(target_yaw_deg) &&
            target_yaw_deg >= -180.0f && target_yaw_deg <= 180.0f &&
            flags == SRP_CHASSIS_HEADING_FLAGS_NONE &&
            queue_motion_command(frame->type, SRP_MSG_ID_CHASSIS_HEADING_CMD,
                                 frame->payload, (uint8_t)frame->length, &meta)) {
            return;
        }
    } else if (frame->type == SC_APP_TYPE_WHEEL_SPEED_SINGLE_CMD &&
               frame->length == SRP_PAYLOAD_WHEEL_SPEED_SINGLE_CMD_SIZE &&
               frame->payload != NULL && frame->payload[0] < 4U) {
        const float speed = srp_wire_read_f32_le(&frame->payload[1]);
        if (isfinite(speed) && fabsf(speed) <= 1000.0f &&
            queue_motion_command(frame->type,
                                 SRP_MSG_ID_WHEEL_SPEED_SINGLE_CMD,
                                 frame->payload, (uint8_t)frame->length, &meta)) {
            return;
        }
    } else if (frame->type == SC_APP_TYPE_MASTER_SPEED_CMD &&
               frame->length == SRP_PAYLOAD_MASTER_SPEED_CMD_SIZE &&
               frame->payload != NULL) {
        const float scale = srp_wire_read_f32_le(frame->payload);
        if (isfinite(scale) && scale >= 0.0f && scale <= 4.0f &&
            queue_motion_command(frame->type, SRP_MSG_ID_MASTER_SPEED_CMD,
                                 frame->payload, (uint8_t)frame->length, &meta)) {
            return;
        }
    } else if (frame->type == SC_APP_TYPE_PID_PARAMS_CMD &&
               frame->length == SRP_PAYLOAD_PID_PARAMS_SIZE &&
               frame->payload != NULL) {
        float params[4];
        const bool valid = srp_wire_read_f32_array_le(
            frame->payload, frame->length, params, 4U) &&
            params[0] >= SRP_PID_KP_MIN && params[0] <= SRP_PID_KP_MAX &&
            params[1] >= SRP_PID_KI_MIN && params[1] <= SRP_PID_KI_MAX &&
            params[2] >= SRP_PID_KD_MIN && params[2] <= SRP_PID_KD_MAX &&
            params[3] >= SRP_PID_ACCEL_MIN && params[3] <= SRP_PID_ACCEL_MAX;
        if (valid &&
            (s_pid_tx_context = (smartcar_app_tx_context_t){
                .app_type = frame->type,
                .app_version = meta.version,
                .session_id = meta.session_id,
                .sequence = meta.sequence
            },
            srp_link_send(&s_link, SRP_PRIORITY_COMMAND,
                           SRP_NODE_STM32H757, SRP_MSG_ID_PID_PARAMS_CMD,
                           SRP_FLAG_ACK_REQUIRED, frame->payload,
                           (uint8_t)frame->length,
                           (uint32_t)(now_us() / UINT64_C(1000)),
                           app_transaction_tx_complete,
                           &s_pid_tx_context) == 0)) {
            return;
        }
    } else if (frame->type == SC_APP_TYPE_SYS_CONFIG &&
               frame->length >= 2U && frame->payload != NULL) {
        srp_frame_t config_frame = {
            .priority = SRP_PRIORITY_COMMAND,
            .type = SRP_MSG_ID_SYS_CONFIG,
            .sequence = 0U,
            .flags = SRP_FLAG_TLV | SRP_FLAG_ACK_REQUIRED,
            .length = frame->length,
            .payload = frame->payload,
        };
        uint32_t baud_rate = 0U;
        if (command_bridge_decode_baudrate(&config_frame, &baud_rate) &&
            (s_baud_tx_context = (smartcar_app_tx_context_t){
                .app_type = frame->type,
                .app_version = meta.version,
                .session_id = meta.session_id,
                .sequence = meta.sequence
            },
            s_baud_change_value = baud_rate,
            srp_link_send(&s_link, SRP_PRIORITY_COMMAND, SRP_NODE_STM32H757,
                          SRP_MSG_ID_SYS_CONFIG, SRP_FLAG_TLV | SRP_FLAG_ACK_REQUIRED,
                          frame->payload, frame->length,
                          (uint32_t)(now_us() / UINT64_C(1000)),
                          baud_config_tx_complete, &s_baud_tx_context) == 0)) {
            return;
        }
    } else if (frame->type == SC_APP_TYPE_RADAR_SET_SPEED && frame->length == 1U &&
        frame->payload != NULL && frame->payload[0] <= RADAR_MAX_SPEED &&
        radar_control_set_speed(frame->payload[0])) {
        result = SC_APP_ACK_OK;
    }
#endif
    send_command_result(meta.version, frame->type, meta.session_id,
                        meta.sequence, result,
                        result == SC_APP_ACK_OK ? SC_APP_V2_STAGE_GATEWAY_ADMITTED :
                                                  SC_APP_V2_STAGE_GATEWAY_ADMITTED);
}

/**
 * @brief 统计并记录 App 外层帧错误，不生成运动命令。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param error App parser 负错误码。
 * @param data 错误候选帧借用缓冲，当前不读取。
 * @param length 候选帧当前长度，当前不读取。
 * @param context parser 上下文，当前忽略。
 * @return 返回值：无（void）。
 * 调用方式：由 sc_app_parser_feed() 在服务任务调用栈同步触发。
 * 线程约束：只允许 parser/service owner；不得保留 data 或从 GATT/ISR 直接调用。
 */
static void app_command_on_error(int error, const uint8_t *data,
                                 size_t length, void *context)
{
    (void)data;
    (void)length;
    (void)context;
    ++s_ble_rx_protocol_errors;
    ESP_LOGW(TAG, "App BLE envelope error=%d", error);
}

/**
 * @brief 读取本地雷达 RUNNING/速度快照，并分别尝试上报给 STM 和 App。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；SRP 和 BLE 两次提交结果当前均被忽略，状态可能静默丢失。
 * 调用方式：非 BMI323 debug-only 镜像中由服务任务每 APP_RADAR_STATUS_PERIOD_MS 调用一次；只上报本地门控，不参与 STM 运动控制。
 * 线程约束：可能两次等待 radar_control mutex 各 20 ms，并阻塞 UART/BLE 提交；仅服务任务调用，禁止 ISR/GATT 回调或并发调用。
 */
static void notify_radar_status(void)
{
    uint8_t payload[SRP_PAYLOAD_RADAR_STATUS_SIZE];

#if !SMARTCAR_BMI323_DEBUG_ONLY
    payload[0] = radar_control_is_running() ? 1U : 0U;
    payload[1] = payload[0] != 0U ? radar_control_get_speed() : 0U;
    (void)srp_link_send(&s_link, SRP_PRIORITY_TELEMETRY,
                          SRP_NODE_STM32H757, SRP_MSG_ID_RADAR_STATUS,
                          SRP_FLAG_STREAM_DATA, payload, sizeof(payload),
                          (uint32_t)(now_us() / UINT64_C(1000)), NULL, NULL);
    (void)notify_app_frame(SC_APP_TYPE_RADAR_STATUS, payload, sizeof(payload));
#else
    (void)payload;
#endif
}

/**
 * @brief 串行消费 BLE 队列、STM UART、SRP link、同步、遥测、停机和恢复事件。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param context FreeRTOS 任务参数，当前忽略，允许 NULL。
 * @return 不返回；任务永久按服务周期运行。
 * 调用方式：仅由 smartcar_service_init() 通过 xTaskCreate() 创建一个实例。
 * 线程约束：该任务是 parser/link/会话/运动事务唯一 owner；内部可能调用阻塞 UART/BLE，
 *           其他上下文只能通过队列或原子/标志交接，不得直接并发调用本函数。
 */
static void smartcar_service_task(void *context)
{
    TickType_t last_radar_status = xTaskGetTickCount();
    TickType_t last_stack_report = last_radar_status;

    (void)context;
    for (;;) {
        const uint32_t loop_connection_epoch = s_ble_connection_epoch;

        if (s_app_v2_session.active &&
            now_us() - s_app_v2_session.last_activity_us >=
            ((uint64_t)APP_V2_SESSION_TTL_MS * UINT64_C(1000))) {
            s_ble_disconnect_stop_pending = true;
            ESP_LOGW(TAG, "App BLE V2 session expired; stop requested");
        }
        if (s_ble_disconnect_stop_pending) {
            s_ble_disconnect_stop_pending = false;
            s_ble_stop_cleanup_active = true;
            /* The stop boundary owns all App parser/session/motion state. Drop
             * every queued byte before accepting a new HELLO/session. */
            command_bridge_flush_ble_rx_queue();
            command_bridge_reset_app_session();
            s_motion_pending_target.valid = false;
            s_motion_pending_scale.valid = false;
            cancel_motion_transactions();
            command_bridge_send_zero_wheel_speed();
            s_ble_stop_cleanup_active = false;
            /* Do not consume an RX item in the same iteration as the stop.
             * This keeps the safety frame ahead of any post-event traffic. */
            continue;
        }
        command_bridge_sync_step(now_us());
        if (command_bridge_ble_stop_requested()) {
            continue;
        }
        command_bridge_log_uart_diag(now_us());
        if (command_bridge_ble_stop_requested()) {
            continue;
        }
        command_bridge_log_app_ble_diag(now_us());
        if (command_bridge_ble_stop_requested()) {
            continue;
        }
        for (uint8_t budget = 0U; budget < SMARTCAR_SERVICE_BLE_RX_BUDGET;
             ++budget) {
            if (s_ble_rx_queue == NULL ||
                xQueueReceive(s_ble_rx_queue, &s_ble_rx_item, 0U) != pdPASS) {
                break;
            }
            if (s_ble_disconnect_stop_pending ||
                s_ble_rx_item.connection_epoch != loop_connection_epoch ||
                s_ble_rx_item.connection_epoch != s_ble_connection_epoch) {
                ++s_ble_rx_dropped;
                continue;
            }
            (void)sc_app_parser_feed(&s_app_parser, s_ble_rx_item.bytes,
                                     s_ble_rx_item.length);
        }

        if (command_bridge_ble_stop_requested()) {
            continue;
        }

        {
            const bool rx_discontinuity = stm_uart_take_rx_discontinuity();
            const bool break_recovery = stm_uart_take_break_recovery();
            if (break_recovery && s_sync_state != SMARTCAR_SYNC_SYNCED) {
                command_bridge_restart_sync("BREAK_THRESHOLD");
            }
            if (rx_discontinuity) {
                srp_parser_reset(&s_parser);
                const uint64_t reset_now = now_us();
                if (reset_now - s_last_rx_reset_log_us >=
                    ((uint64_t)SMARTCAR_SERVICE_RX_RESET_LOG_PERIOD_MS * UINT64_C(1000))) {
                    s_last_rx_reset_log_us = reset_now;
                    ESP_LOGW(TAG, "UART2 RX discontinuity; SRP parser reset to header seek");
                }
            }
            if (command_bridge_ble_stop_requested()) {
                continue;
            }
            const int received = stm_uart_receive_nonblock(s_uart_rx_buffer,
                                                            sizeof(s_uart_rx_buffer));
            if (received > 0 && !command_bridge_ble_stop_requested()) {
                (void)srp_parser_feed(&s_parser, s_uart_rx_buffer, (size_t)received);
            }
        }
        if (command_bridge_ble_stop_requested()) {
            continue;
        }
        srp_link_tick(&s_link, (uint32_t)(now_us() / UINT64_C(1000)));
        if (command_bridge_ble_stop_requested()) {
            continue;
        }
        command_bridge_check_stm_liveness(now_us());
        if (command_bridge_ble_stop_requested()) {
            continue;
        }
        if (s_baud_change_pending && now_us() >= s_baud_change_due_us) {
            const uint32_t baud_rate = s_baud_change_value;
            s_baud_change_pending = false;
            if (stm_uart_set_baud_rate(baud_rate) == ESP_OK) {
                ESP_LOGI(TAG, "SRP UART2 baud changed to %lu",
                         (unsigned long)baud_rate);
            } else {
                ESP_LOGE(TAG, "SRP UART2 baud change failed");
            }
        }
        if (command_bridge_ble_stop_requested()) {
            continue;
        }
        if (s_bus_off_recovery_pending && now_us() >= s_bus_off_recovery_at_us) {
            srp_link_recover(&s_link);
            s_bus_off_recovery_pending = false;
            s_bus_off_latched = false;
            ESP_LOGI(TAG, "SRP link recovered");
        }
        if (command_bridge_ble_stop_requested()) {
            continue;
        }

#if !SMARTCAR_BMI323_DEBUG_ONLY
        radar_calibration_manager_step();
        if ((xTaskGetTickCount() - last_radar_status) >=
            pdMS_TO_TICKS(APP_RADAR_STATUS_PERIOD_MS)) {
            last_radar_status = xTaskGetTickCount();
            notify_radar_status();
        }
#endif
        if (command_bridge_ble_stop_requested()) {
            continue;
        }
        if ((xTaskGetTickCount() - last_stack_report) >=
            pdMS_TO_TICKS(SMARTCAR_SERVICE_STACK_REPORT_PERIOD_MS)) {
            last_stack_report = xTaskGetTickCount();
            s_stack_min_free_bytes = uxTaskGetStackHighWaterMark(NULL);
            s_stack_hwm_valid = true;
            ESP_LOGI(TAG, "STACK_HWM task=%s min_free_words=%u errors=%lu rec=%u tec=%u",
                     SMARTCAR_SERVICE_TASK_NAME, (unsigned)s_stack_min_free_bytes,
                     (unsigned long)s_parser_errors,
                     (unsigned)srp_link_get_rec(&s_link),
                     (unsigned)srp_link_get_tec(&s_link));
        }
        vTaskDelay(SMARTCAR_SERVICE_TASK_DELAY_TICKS);
    }
}

/**
 * @brief 创建静态 BLE RX 队列，初始化 App/SRP parser/link、标定 manager 和唯一服务任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 全部当前步骤成功返回 ESP_OK；队列/任务失败返回 ESP_ERR_NO_MEM，回调注册失败返回对应错误。
 * 调用方式：app_main 在 UART2、BLE 和雷达基础层初始化后调用一次；调用前可注册 telemetry sink。
 * 线程约束：非幂等、仅启动任务调用；会创建 16 KiB 服务任务并改写全部无锁全局状态，失败路径不完整回滚已注册回调/已创建资源，禁止 ISR/GATT 回调或并发重入。
 */
esp_err_t smartcar_service_init(void)
{
    const srp_link_config_t link_config = {
        .local_node = SRP_NODE_ESP32_S3,
        .ack_timeout_ms = SRP_LINK_ACK_TIMEOUT_MS,
        .max_retries = SRP_LINK_MAX_RETRIES,
        .transport_send = command_bridge_transport_send,
        .on_frame = command_bridge_on_frame,
        .on_bus_off = command_bridge_bus_off,
        .context = NULL,
    };

    s_ble_rx_dropped = 0U;
    s_ble_rx_received = 0U;
    s_ble_rx_protocol_errors = 0U;
    s_parser_errors = 0U;
    s_dual_len_reject = 0U;
    s_dual_schema_reject = 0U;
    s_dual_crc_reject = 0U;
    s_dual_notify_drop = 0U;
    s_dual_ble_not_ready = 0U;
    s_ble_disconnect_stop_pending = false;
    s_ble_connection_epoch = 0U;
    s_ble_stop_cleanup_active = false;
    s_baud_change_pending = false;
    s_baud_change_value = STM_UART_BAUD_RATE;
    s_baud_change_due_us = 0U;
    s_sync_state = SMARTCAR_SYNC_UART_READY;
    s_sync_attempts = 0U;
    s_sync_tx_sequence = 0U;
    s_next_sync_us = 0U;
    s_last_sync_timeout_notify_us = 0U;
    s_last_stm_frame_us = 0U;
    s_last_uart_diag_us = 0U;
    s_last_rx_reset_log_us = 0U;
    s_last_header_fail_log_us = 0U;
    s_last_app_ble_diag_us = 0U;
    s_app_valid_frames = 0U;
    s_app_command_frames = 0U;
    s_app_ack_ok = 0U;
    s_app_ack_rejected = 0U;
    s_app_ack_dropped = 0U;
    s_app_last_ack_type = 0U;
    s_app_last_ack_result = 0U;
    s_app_last_ack_stage = 0U;
    s_app_last_ack_sequence = 0U;
    stm_uart_set_sync_state(false);
    command_bridge_reset_app_session();
    s_motion_inflight.valid = false;
    s_motion_pending_target.valid = false;
    s_motion_pending_scale.valid = false;
    s_motion_tx_in_flight = false;
    s_motion_generation = 0U;
    s_stack_hwm_valid = false;
    s_bus_off_recovery_pending = false;
    s_bus_off_latched = false;
    s_ble_rx_queue = xQueueCreateStatic(SMARTCAR_SERVICE_BLE_RX_QUEUE_DEPTH,
                                        sizeof(smartcar_ble_rx_item_t),
                                        s_ble_rx_queue_buffer,
                                        &s_ble_rx_queue_storage);
    if (s_ble_rx_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    srp_link_init(&s_link, &link_config);
    srp_parser_init(&s_parser, command_bridge_parsed_frame,
                     command_bridge_on_parser_error, NULL);
    s_link_ready = 1U;
#if !SMARTCAR_BMI323_DEBUG_ONLY
    radar_calibration_manager_init();
    radar_calibration_manager_set_transport(command_bridge_send_radar_pwm_ready, NULL);
#endif
    if (s3_ble_set_disconnect_callback(command_bridge_on_ble_disconnect, NULL) != ESP_OK) {
        s_ble_rx_queue = NULL;
        s_link_ready = 0U;
        return ESP_FAIL;
    }
    if (xTaskCreate(smartcar_service_task, SMARTCAR_SERVICE_TASK_NAME,
                    SMARTCAR_SERVICE_TASK_STACK, NULL,
                    SMARTCAR_SERVICE_TASK_PRIORITY, &s_task) != pdPASS) {
        s_ble_rx_queue = NULL;
        s_link_ready = 0U;
        return ESP_ERR_NO_MEM;
    }
    return s3_ble_register_rx_callback(service_ble_rx_enqueue, NULL);
}

/**
 * @brief FreeRTOS 栈溢出全局 hook：输出早期诊断后立即 abort，阻止受损任务继续运行。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param task 检测到溢出的任务句柄；允许 NULL，NULL 时高水位记为 0。
 * @param task_name FreeRTOS 借用的任务名，仅在本次日志格式化期间读取；允许 NULL。
 * @return 不返回；ESP_EARLY_LOGE 后调用 abort()，由平台 panic/reset 策略接管。
 * 调用方式：只由启用栈溢出检查的 FreeRTOS 内核调用，业务代码不得手工调用；本函数不尝试通过已可能损坏的服务栈发送停机帧。
 * 线程约束：运行上下文由 FreeRTOS 检测点决定，不能阻塞、分配资源或取得业务 mutex；仅使用早期日志和只读诊断快照，最终终止当前执行流。
 */
void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    const UBaseType_t task_hwm = task == NULL ? 0U :
                                 uxTaskGetStackHighWaterMark(task);

    ESP_EARLY_LOGE(TAG,
                   "STACK_OVERFLOW task=%s handle=%p task_hwm=%u "
                   "service_hwm=%u service_hwm_valid=%u",
                   task_name != NULL ? task_name : "<null>",
                   (void *)task,
                   (unsigned)task_hwm,
                   (unsigned)s_stack_min_free_bytes,
                   s_stack_hwm_valid ? 1U : 0U);
    abort();
}
