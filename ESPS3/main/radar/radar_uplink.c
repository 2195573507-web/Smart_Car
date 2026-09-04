#include "radar_uplink.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <netdb.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "radar_uplink_protocol.h"
#include "radar_uplink_tx.h"
#include "radar_telemetry_observability.h"
#include "radar_telemetry_age.h"
#include "radar_telemetry_queue.h"
#include "radar_uart.h"
#include "s3_ble.h"
#include "smartcar_service.h"
#include "smartcar_debug_config.h"
#include "smartcar_wifi_sta.h"

/* 雷达 Wi-Fi/TCP 上行实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include "sdkconfig.h"
static const char *TAG = "RADAR_UPLINK";

#define RADAR_UPLINK_TASK_STACK_SIZE 6144U
#define RADAR_UPLINK_TASK_PRIORITY 4U
/* Notifications wake the sender; this timeout still services Wi-Fi state. */
#define RADAR_UPLINK_WAIT_MS 100U
/* Drain a bounded burst after a TCP stall without holding the UART FIFO lock. */
#define RADAR_UPLINK_BURST_MAX_FRAMES 4U
#define RADAR_UPLINK_CONNECT_TIMEOUT_MS 500U
#define RADAR_UPLINK_TCP_PORT "8765"
#define RADAR_UPLINK_RETRY_INITIAL_MS 500U
#define RADAR_UPLINK_RETRY_MAX_MS 10000U
#define RADAR_UPLINK_RETRY_YIELD_TICKS 1U
#define RADAR_UPLINK_TELEMETRY_WHEEL_FIFO_DEPTH 32U
/* Candidate operational limits; tune only after a real capture. */
#define RADAR_UPLINK_MAX_RADAR_DEQUEUE_AGE_MS 500U
#define RADAR_UPLINK_MAX_TELEMETRY_AGE_MS 1000U
#define RADAR_UPLINK_TELEMETRY_MUTEX_WAIT_TICKS 0U
#define RADAR_UPLINK_TELEMETRY_FEATURE_ID "S3_TELEM_OBS_V1"

#ifndef SMARTCAR_TELEMETRY_SOURCE_SHA8
#error "SMARTCAR_TELEMETRY_SOURCE_SHA8 must be provided by main/CMakeLists.txt"
#endif

_Static_assert(SRP_MSG_ID_CHASSIS_STATE == UINT8_C(0x15),
               "telemetry READY marker must match chassis message id");
_Static_assert(RADAR_UPLINK_MESSAGE_SRP_TELEMETRY_EXPERIMENTAL == 2U,
               "telemetry READY marker must match outer message type");

#if CONFIG_SMARTCAR_RADAR_UPLINK_ENABLED
/**
 * @brief  格式化一条有界文本并转交 S3 BLE 日志通道。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  level 仅处理 INFO、WARN、ERROR；其他级别格式化后不发送。
 * @param  format printf 风格格式字符串；NULL 时不动作。
 * @param  ... 与 format 匹配的可变参数。
 * @return 无；格式错误、空文本、超出 SMARTCAR_LOG_MAX_PAYLOAD 或 BLE 日志发送失败时不向调用方报错。
 * 调用方式：上行初始化/任务路径输出可选诊断；日志不参与 TCP 状态机和安全控制判定。
 * 线程约束：使用栈缓冲并调用日志/BLE API，可在普通任务或 ESP 事件任务中使用；
 *           禁止 ISR 和硬实时路径。
 */
static void radar_uplink_ble_log(smartcar_log_level_t level,
                                 const char *format,
                                 ...)
{
    char message[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
    va_list args;
    int written;

    if (format == NULL) {
        return;
    }

    va_start(args, format);
    written = vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    if (written <= 0 || (size_t)written >= sizeof(message)) {
        return;
    }

    switch (level) {
    case SMARTCAR_LOG_LEVEL_INFO:
        (void)s3_log_info(message);
        break;
    case SMARTCAR_LOG_LEVEL_WARN:
        (void)s3_log_warn(message);
        break;
    case SMARTCAR_LOG_LEVEL_ERROR:
        (void)s3_log_error(message);
        break;
    default:
        break;
    }
}
#endif

#if CONFIG_SMARTCAR_RADAR_UPLINK_ENABLED
static TaskHandle_t s_uplink_task;
static bool s_initialized;
static int s_socket = -1;
static radar_telemetry_queue_t s_telemetry_queue;
static radar_telemetry_entry_t *s_telemetry_wheel_entries;
static radar_telemetry_entry_t s_telemetry_chassis_entry;
static radar_telemetry_entry_t s_telemetry_attitude_entry;
static radar_telemetry_entry_t s_telemetry_imu_entries[RADAR_TELEMETRY_QUEUE_IMU_SLOT_COUNT];
static SemaphoreHandle_t s_telemetry_mutex;
static StaticSemaphore_t s_telemetry_mutex_storage;
static bool s_telemetry_queue_ready;
static radar_telemetry_observability_stats_t s_telemetry_observability;
static portMUX_TYPE s_telemetry_observability_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_telemetry_observability_last_log_ms;
static bool s_telemetry_observability_log_timestamp_valid;
typedef struct {
    uint32_t successful_frames;
    uint64_t successful_bytes;
    uint32_t send_failures;
    uint32_t send_timeouts;
    uint32_t partial_writes;
    uint32_t pending_retries;
    uint32_t connect_failures;
    uint32_t reconnects;
    uint32_t resync_discarded_frames;
    uint32_t encode_failures;
    uint32_t radar_stale_drops;
    uint32_t radar_sequence_gaps;
    uint32_t telemetry_stale_drops;
    uint32_t telemetry_encode_failures;
    uint32_t telemetry_sent_frames;
    uint64_t telemetry_sent_bytes;
    uint32_t last_telemetry_sequence;
    uint32_t telemetry_lock_drops;
    uint32_t last_sent_sequence;
    uint32_t max_dequeue_age_ms;
    uint32_t last_report_ms;
    bool report_timestamp_valid;
} radar_uplink_stats_t;

/**
 * @brief  在可观测统计临界区内记录一次 telemetry sink 调用。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  message_id 收到的 SRPv4 消息 ID；未知类型仍累计 sink 总次数。
 * @return 无；计数达到 UINT32_MAX 后保持饱和。
 * 调用方式：radar_uplink_telemetry_sink() 在检查队列状态和尝试 mutex 前调用。
 * 线程约束：使用 s_telemetry_observability_lock 的短临界区，可由普通任务并发调用；禁止 ISR 调用该非 ISR 包装。
 */
static void telemetry_observability_note_sink(uint16_t message_id)
{
    portENTER_CRITICAL(&s_telemetry_observability_lock);
    radar_telemetry_observability_note_sink_call(&s_telemetry_observability,
                                                  message_id);
    portEXIT_CRITICAL(&s_telemetry_observability_lock);
}

/**
 * @brief  在可观测统计临界区内记录一次 telemetry 队列外层锁竞争丢弃。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 无；计数达到 UINT32_MAX 后保持饱和。
 * 调用方式：telemetry sink、pop 或队列统计的零等待 mutex 未取得时调用。
 * 线程约束：仅持有短 portMUX 临界区、不执行日志或网络操作；供普通任务使用，禁止 ISR 调用该包装。
 */
static void telemetry_observability_note_lock_drop(void)
{
    portENTER_CRITICAL(&s_telemetry_observability_lock);
    radar_telemetry_observability_note_lock_drop(&s_telemetry_observability);
    portEXIT_CRITICAL(&s_telemetry_observability_lock);
}

/**
 * @brief  在可观测统计临界区内记录一次 telemetry 队列 push 精确结果。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  result ACCEPTED、OVERWRITTEN 或 REJECTED；未知值按 rejected 统计。
 * @return 无；相关计数达到 UINT32_MAX 后保持饱和。
 * 调用方式：radar_uplink_telemetry_sink() 每次入队尝试或前置状态拒绝后调用一次。
 * 线程约束：使用短 portMUX 临界区串行化共享统计，不阻塞等待 RTOS mutex；禁止 ISR 调用该包装。
 */
static void telemetry_observability_note_queue_result(
    radar_telemetry_queue_push_result_t result)
{
    portENTER_CRITICAL(&s_telemetry_observability_lock);
    radar_telemetry_observability_note_queue_result(&s_telemetry_observability,
                                                     result);
    portEXIT_CRITICAL(&s_telemetry_observability_lock);
}

/**
 * @brief  在可观测统计临界区内记录一次类型 2 遥测包封装完成。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 无；计数达到 UINT32_MAX 后保持饱和。
 * 调用方式：上行任务成功编码类型 2 包并建立待发送状态后调用；不代表 TCP 已发送。
 * 线程约束：仅持有短 portMUX 临界区；供普通任务使用，禁止 ISR 调用该包装。
 */
static void telemetry_observability_note_packet_prepared(void)
{
    portENTER_CRITICAL(&s_telemetry_observability_lock);
    radar_telemetry_observability_note_packet_prepared(
        &s_telemetry_observability);
    portEXIT_CRITICAL(&s_telemetry_observability_lock);
}

/**
 * @brief  在可观测统计临界区内记录类型 2 包的发送完成、等待或失败结果。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  result radar_uplink_tx_send() 结果；COMPLETE/FAILED 分别计数，WAIT 不改变计数。
 * @return 无；相关计数达到 UINT32_MAX 后保持饱和。
 * 调用方式：上行任务每次推进类型 2 待发包后调用；成功仅表示本地整包 send 完成。
 * 线程约束：使用短 portMUX 临界区，不执行 socket 操作；供普通任务使用，禁止 ISR 调用该包装。
 */
static void telemetry_observability_note_type2_tx_result(
    radar_uplink_tx_result_t result)
{
    portENTER_CRITICAL(&s_telemetry_observability_lock);
    radar_telemetry_observability_note_type2_tx_result(
        &s_telemetry_observability, result);
    portEXIT_CRITICAL(&s_telemetry_observability_lock);
}

/**
 * @brief  在可观测统计临界区内记录一条超时效遥测队列项被丢弃。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 无；计数达到 UINT32_MAX 后保持饱和。
 * 调用方式：上行任务弹出 telemetry entry 并判定帧龄超过限制时调用。
 * 线程约束：仅持有短 portMUX 临界区；供普通任务使用，禁止 ISR 调用该包装。
 */
static void telemetry_observability_note_stale_drop(void)
{
    portENTER_CRITICAL(&s_telemetry_observability_lock);
    radar_telemetry_observability_note_stale_drop(&s_telemetry_observability);
    portEXIT_CRITICAL(&s_telemetry_observability_lock);
}

/**
 * @brief  在可观测统计临界区内复制一份一致的累计计数快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[out] snapshot 调用方拥有的输出对象；可为 NULL，NULL 时底层复制不动作。
 * @return 无；不清零或修改源统计。
 * 调用方式：仅由上行任务的低频完整日志、短日志和周期统计路径调用。
 * 线程约束：在 s_telemetry_observability_lock 短临界区内执行结构体复制；禁止 ISR 调用该包装。
 * 所有权约束：返回后 snapshot 与全局统计独立，函数不保留输出指针。
 */
static void telemetry_observability_get_snapshot(
    radar_telemetry_observability_stats_t *snapshot)
{
    portENTER_CRITICAL(&s_telemetry_observability_lock);
    radar_telemetry_observability_snapshot(&s_telemetry_observability,
                                            snapshot);
    portEXIT_CRITICAL(&s_telemetry_observability_lock);
}

/**
 * @brief  若上行任务已创建，则发送一次 FreeRTOS task notification 唤醒它。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 无；任务句柄为空时通知被静默忽略。
 * 调用方式：telemetry sink 和 Wi-Fi/IP 事件回调在状态变化后调用；通知只用于唤醒，不携带帧所有权。
 * 线程约束：调用 xTaskNotifyGive() 而非 FromISR 版本，仅允许任务/ESP 事件循环上下文，禁止 ISR。
 */
static void notify_uplink_task(void)
{
    if (s_uplink_task != NULL) {
        xTaskNotifyGive(s_uplink_task);
    }
}

/**
 * @brief  回滚启动期 telemetry sink/队列状态并释放 wheel FIFO 的 PSRAM 存储。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 无；未分配 wheel 存储时只清状态和 sink。
 * 调用方式：仅在 smartcar_service_init() 之前的上行初始化失败路径调用；不是通用运行期反初始化接口。
 * 线程约束：启动任务串行执行、无并发生产者/消费者；任务启动或服务运行后调用会产生竞态，
 *           禁止 ISR 调用。
 * 所有权约束：释放 heap_caps_calloc() 得到的 wheel_entries；静态 attitude/IMU 槽和静态 mutex 存储不释放。
 */
static void telemetry_queue_release(void)
{
    /* The service setter is only legal before service init, which is the
     * only lifecycle in which this rollback helper is called. */
    (void)smartcar_service_set_telemetry_sink(NULL, NULL);
    s_telemetry_queue_ready = false;
    s_telemetry_mutex = NULL;
    if (s_telemetry_wheel_entries != NULL) {
        heap_caps_free(s_telemetry_wheel_entries);
        s_telemetry_wheel_entries = NULL;
    }
}

/**
 * @brief  分配 telemetry wheel FIFO 的 PSRAM，并初始化队列与静态 mutex。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return ESP_OK 表示外部存储、普通 C 队列和 mutex 均就绪；内存不足返回 ESP_ERR_NO_MEM，
 *         队列存储绑定异常返回 ESP_ERR_INVALID_STATE。
 * 调用方式：radar_uplink_init() 在注册 smartcar_service telemetry sink 和创建上行任务前调用一次。
 * 线程约束：启动期执行 PSRAM 分配和 FreeRTOS 对象创建，禁止 ISR、并发调用或运行期重复准备。
 * 失败语义：队列/mutex 初始化失败会调用 telemetry_queue_release() 回滚已分配 wheel 存储。
 */
static esp_err_t telemetry_queue_prepare(void)
{
    const size_t storage_bytes = RADAR_UPLINK_TELEMETRY_WHEEL_FIFO_DEPTH *
                                 sizeof(*s_telemetry_wheel_entries);
    const radar_telemetry_queue_storage_t storage = {
        .wheel_entries = NULL,
        .wheel_capacity = RADAR_UPLINK_TELEMETRY_WHEEL_FIFO_DEPTH,
        .chassis_entry = &s_telemetry_chassis_entry,
        .attitude_entry = &s_telemetry_attitude_entry,
        .imu_entries = s_telemetry_imu_entries,
    };

    s_telemetry_wheel_entries = heap_caps_calloc(
        RADAR_UPLINK_TELEMETRY_WHEEL_FIFO_DEPTH,
        sizeof(*s_telemetry_wheel_entries),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_telemetry_wheel_entries == NULL) {
        ESP_LOGE(TAG, "TELEMETRY FIFO PSRAM ALLOC FAILED entries=%u bytes=%u",
                 (unsigned)RADAR_UPLINK_TELEMETRY_WHEEL_FIFO_DEPTH,
                 (unsigned)storage_bytes);
        return ESP_ERR_NO_MEM;
    }

    radar_telemetry_queue_storage_t mutable_storage = storage;
    mutable_storage.wheel_entries = s_telemetry_wheel_entries;
    if (!radar_telemetry_queue_init(&s_telemetry_queue, &mutable_storage)) {
        telemetry_queue_release();
        return ESP_ERR_INVALID_STATE;
    }
    s_telemetry_mutex = xSemaphoreCreateMutexStatic(&s_telemetry_mutex_storage);
    if (s_telemetry_mutex == NULL) {
        telemetry_queue_release();
        return ESP_ERR_NO_MEM;
    }
    s_telemetry_queue_ready = true;
    ESP_LOGI(TAG, "TELEMETRY FIFO READY entries=%u bytes=%u",
             (unsigned)RADAR_UPLINK_TELEMETRY_WHEEL_FIFO_DEPTH,
             (unsigned)storage_bytes);
    return ESP_OK;
}

/**
 * @brief  将服务层给出的完整 SRPv4 遥测帧复制到独立有界队列并唤醒上行任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  message_id 预期 SRPv4 type，必须属于队列支持的遥测类型。
 * @param  encoded_frame 服务层借出的完整线缆帧；函数返回前完成校验/复制，不保留指针。
 * @param  encoded_length 线缆帧字节数。
 * @param  ingress_timestamp_ms 服务层接收时间，单位 ms，原样存入队列项。
 * @param  context 注册 sink 时传入的借用上下文，必须精确等于 &s_telemetry_queue。
 * @return 本地队列接受帧时为 true；上下文/状态无效、mutex 零等待失败、帧无效或队列满时为 false。
 * 调用方式：由 smartcar_service 的单一服务任务同步调用；返回后服务层可立即复用 encoded_frame。
 * 线程约束：mutex 等待为 0 tick，绝不因慢网络阻塞服务任务；失败会丢当前遥测并增加锁/队列统计，
 *           禁止 ISR 调用。
 */
static bool radar_uplink_telemetry_sink(uint16_t message_id,
                                        const uint8_t *encoded_frame,
                                        uint16_t encoded_length,
                                        uint32_t ingress_timestamp_ms,
                                        void *context)
{
    radar_telemetry_queue_push_result_t result;

    /* CHASSIS_STATE has a frozen ROS STATUS route on TCP 8766. It must never
     * enter the experimental S3RD telemetry envelope on TCP 8765. */
    if (message_id == SRP_MSG_ID_CHASSIS_STATE) {
        return false;
    }
    telemetry_observability_note_sink(message_id);
    if (context != &s_telemetry_queue || !s_telemetry_queue_ready ||
        s_telemetry_mutex == NULL) {
        telemetry_observability_note_queue_result(
            RADAR_TELEMETRY_QUEUE_PUSH_REJECTED);
        return false;
    }
    if (xSemaphoreTake(s_telemetry_mutex,
                       RADAR_UPLINK_TELEMETRY_MUTEX_WAIT_TICKS) != pdTRUE) {
        telemetry_observability_note_lock_drop();
        telemetry_observability_note_queue_result(
            RADAR_TELEMETRY_QUEUE_PUSH_REJECTED);
        return false;
    }
    result = radar_telemetry_queue_push_ex(&s_telemetry_queue,
                                           message_id,
                                           encoded_frame,
                                           encoded_length,
                                           ingress_timestamp_ms);
    (void)xSemaphoreGive(s_telemetry_mutex);
    telemetry_observability_note_queue_result(result);
    if (result != RADAR_TELEMETRY_QUEUE_PUSH_REJECTED) {
        notify_uplink_task();
        return true;
    }
    return false;
}

/**
 * @brief  在零等待 mutex 保护下从 telemetry 队列复制并消费下一项。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[out] entry 非 NULL；成功时获得完整 SRPv4 帧副本及接收元数据。
 * @return 成功弹出为 true；参数/队列/mutex 无效、锁忙或队列空时为 false。
 * 调用方式：仅由上行任务准备 telemetry 包时调用；false 不区分空队列与锁竞争，调用方不得忙等。
 * 线程约束：mutex 等待 0 tick并复制完整 entry；禁止 ISR，entry 由上行任务独占。
 */
static bool radar_uplink_pop_telemetry(radar_telemetry_entry_t *entry)
{
    bool popped = false;

    if (entry == NULL || !s_telemetry_queue_ready || s_telemetry_mutex == NULL) {
        return false;
    }
    if (xSemaphoreTake(s_telemetry_mutex,
                       RADAR_UPLINK_TELEMETRY_MUTEX_WAIT_TICKS) != pdTRUE) {
        telemetry_observability_note_lock_drop();
        return false;
    }
    popped = radar_telemetry_queue_pop(&s_telemetry_queue, entry);
    (void)xSemaphoreGive(s_telemetry_mutex);
    return popped;
}

/**
 * @brief  在零等待 mutex 保护下获取 telemetry 队列统计快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[out] stats 可为 NULL；非 NULL 时先清零，取锁成功后再复制真实统计。
 * @return 无；队列未就绪或锁忙时保留全零输出并增加 telemetry 锁丢弃计数。
 * 调用方式：上行任务的低频统计日志调用；全零可能表示真实空状态，也可能表示未就绪/锁竞争。
 * 线程约束：mutex 等待 0 tick、不阻塞；禁止 ISR，快照仅代表调用时刻的本地队列。
 */
static void radar_uplink_get_telemetry_stats(
    radar_telemetry_queue_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    (void)memset(stats, 0, sizeof(*stats));
    if (!s_telemetry_queue_ready || s_telemetry_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_telemetry_mutex,
                       RADAR_UPLINK_TELEMETRY_MUTEX_WAIT_TICKS) != pdTRUE) {
        telemetry_observability_note_lock_drop();
        return;
    }
    radar_telemetry_queue_get_stats(&s_telemetry_queue, stats);
    (void)xSemaphoreGive(s_telemetry_mutex);
}

/**
 * @brief  关闭当前 TCP socket 并把模块句柄复位为无效值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 无；句柄小于 0 时不动作，shutdown/close 的错误被忽略。
 * 调用方式：上行任务发现 Wi-Fi 断开、send 失败或准备重连时调用；关闭后旧 fd 不得再使用。
 * 线程约束：s_socket 由单一上行任务拥有，无内部锁；禁止事件回调/ISR/其他任务并发关闭。
 */
static void close_socket(void)
{
    if (s_socket >= 0) {
        shutdown(s_socket, SHUT_RDWR);
        close(s_socket);
        s_socket = -1;
    }
}

/**
 * @brief  解析配置的主机/端口并建立一个非阻塞 TCP 连接。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 成功时返回保持 O_NONBLOCK 的 socket fd；DNS、socket、fcntl、connect、select 或 SO_ERROR 失败返回 -1。
 * 调用方式：仅由上行任务在已获得 IP 且当前无 socket 时调用；失败后由外层指数退避再试。
 * 线程约束：getaddrinfo() 解析时间没有本函数级超时，connect 的 EINPROGRESS 等待最多 500 ms；
 *           会阻塞低优先级上行任务，禁止 ISR、事件回调或控制任务调用。
 * 验证边界：成功只证明 TCP 三次握手在本地完成，不证明 Windows 应用解析、ROS2 发布或数据消费。
 */
static int connect_endpoint(void)
{
    struct addrinfo hints = {0};
    struct addrinfo *result = NULL;
    struct timeval timeout = {
        .tv_sec = RADAR_UPLINK_CONNECT_TIMEOUT_MS / 1000U,
        .tv_usec = (RADAR_UPLINK_CONNECT_TIMEOUT_MS % 1000U) * 1000U,
    };
    int socket_fd;

    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    if (getaddrinfo(CONFIG_SMARTCAR_RADAR_UPLINK_HOST,
                    RADAR_UPLINK_TCP_PORT,
                    &hints,
                    &result) != 0 || result == NULL) {
        return -1;
    }

    socket_fd = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (socket_fd < 0) {
        freeaddrinfo(result);
        return -1;
    }

    const int original_flags = fcntl(socket_fd, F_GETFL, 0);
    if (original_flags < 0 || fcntl(socket_fd, F_SETFL, original_flags | O_NONBLOCK) < 0) {
        close(socket_fd);
        freeaddrinfo(result);
        return -1;
    }

    int connect_result = connect(socket_fd, result->ai_addr, result->ai_addrlen);
    if (connect_result != 0 && errno != EINPROGRESS) {
        close(socket_fd);
        freeaddrinfo(result);
        return -1;
    }
    if (connect_result != 0) {
        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(socket_fd, &write_fds);
        int select_result;
        do {
            select_result = select(socket_fd + 1, NULL, &write_fds, NULL, &timeout);
        } while (select_result < 0 && errno == EINTR);
        if (select_result <= 0 || !FD_ISSET(socket_fd, &write_fds)) {
            close(socket_fd);
            freeaddrinfo(result);
            return -1;
        }

        int socket_error = 0;
        socklen_t socket_error_length = sizeof(socket_error);
        if (getsockopt(socket_fd,
                       SOL_SOCKET,
                       SO_ERROR,
                       &socket_error,
                       &socket_error_length) != 0 || socket_error != 0) {
            close(socket_fd);
            freeaddrinfo(result);
            return -1;
        }
    }

    freeaddrinfo(result);
    return socket_fd;
}

/**
 * @brief  使用当前模块 socket 执行一次非阻塞 send()。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  context 发送状态机传入的上下文，当前忽略，可为 NULL。
 * @param  data 当前未发送片段的借用指针，仅本次调用期间有效。
 * @param  length 希望写入的剩余字节数。
 * @return send() 的写入字节数或负错误值；errno 原样供 radar_uplink_tx_send() 判定。
 * 调用方式：仅作为 radar_uplink_tx_send() 回调；调用前 s_socket 必须是已连接的非阻塞 fd。
 * 线程约束：单一上行任务调用，不保存 data；禁止 ISR、并发 send 或在回调内递归发送状态机。
 */
static int radar_socket_send(void *context, const uint8_t *data, size_t length)
{
    (void)context;
    return (int)send(s_socket, data, length, 0);
}

/**
 * @brief  读取 YDLIDAR 帧 CT 字节的最低位，判断是否为扫描零包边界。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  frame 候选原始帧；可为 NULL。
 * @param  length frame 可读字节数，必须大于 2 才读取 CT。
 * @return frame 有效到 CT 字节且 bit0 为 1 时返回 true，否则 false。
 * 调用方式：上行重同步路径对已由 UART parser 校验的帧调用；本函数本身不验证帧头、长度或校验和。
 * 线程约束：纯只读计算、可重入、不阻塞。
 */
static bool radar_uplink_frame_is_zero_packet(const uint8_t *frame, size_t length)
{
    return frame != NULL && length > 2U && (frame[2] & 0x01U) != 0U;
}

/**
 * @brief  在断线后放弃未完成发送状态，并要求雷达流重新等待零包边界。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] pending_packet 非 NULL；若原为 true，统计一次被丢弃的待发包，随后清 false。
 * @param[in,out] tx_state 非 NULL；重置 offset、retry 和 partial 状态。
 * @param[out] waiting_for_zero_packet 非 NULL；写 true，阻止普通雷达帧跨连接续接。
 * @param[in,out] stats 非 NULL；按需增加 resync_discarded_frames。
 * @return 无；任一指针为空时整体不动作。
 * 调用方式：上行任务在 Wi-Fi 断开或 send 失败并关闭 socket 后调用；调用方另行清 pending 类型等状态。
 * 线程约束：只修改上行任务私有状态、无锁；禁止其他任务或 ISR 调用。
 */
static void radar_uplink_reset_after_disconnect(
    bool *pending_packet,
    radar_uplink_tx_state_t *tx_state,
    bool *waiting_for_zero_packet,
    radar_uplink_stats_t *stats)
{
    if (pending_packet == NULL || tx_state == NULL ||
        waiting_for_zero_packet == NULL || stats == NULL) {
        return;
    }
    if (*pending_packet) {
        ++stats->resync_discarded_frames;
    }
    *pending_packet = false;
    radar_uplink_tx_reset(tx_state);
    *waiting_for_zero_packet = true;
}

/**
 * @brief  按固定周期输出完整 telemetry 可观测统计，并可发送一条有界 BLE 短日志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 无；距离上次输出不足 RADAR_UPLINK_TELEMETRY_LOG_INTERVAL_MS 时静默返回，
 *         短日志格式化或 BLE 投递失败不影响上行任务。
 * 调用方式：上行任务主循环和分段退避等待路径周期调用；首次调用立即输出。
 * 线程约束：仅允许单一上行任务调用；快照使用短 portMUX 临界区，ESP/BLE 日志可能耗时，禁止 ISR。
 * 语义边界：输出为 S3 本地累计结果，不证明 Windows、ROS2 或 /scan 已接收数据。
 */
static void radar_uplink_log_telemetry_observability(void)
{
    const uint32_t now_ms = (uint32_t)esp_log_timestamp();
    radar_telemetry_observability_stats_t telemetry;
    char short_log[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    if (s_telemetry_observability_log_timestamp_valid &&
        (uint32_t)(now_ms - s_telemetry_observability_last_log_ms) <
            RADAR_UPLINK_TELEMETRY_LOG_INTERVAL_MS) {
        return;
    }
    s_telemetry_observability_last_log_ms = now_ms;
    s_telemetry_observability_log_timestamp_valid = true;
    telemetry_observability_get_snapshot(&telemetry);

    ESP_LOGI(TAG,
             "S3_TELEM_OBS telemetry_sink_calls=%" PRIu32
             " sink_wheel=%" PRIu32 " sink_attitude=%" PRIu32
             " sink_imu=%" PRIu32 " sink_chassis=%" PRIu32
             " telemetry_lock_drops=%" PRIu32
             " queue_accepted=%" PRIu32 " queue_rejected=%" PRIu32
             " queue_overwritten=%" PRIu32
             " telemetry_packets_prepared=%" PRIu32
             " telemetry_type2_sent=%" PRIu32
             " telemetry_stale_drops=%" PRIu32
             " telemetry_send_failures=%" PRIu32,
             telemetry.telemetry_sink_calls,
             telemetry.sink_wheel,
             telemetry.sink_attitude,
             telemetry.sink_imu,
             telemetry.sink_chassis,
             telemetry.telemetry_lock_drops,
             telemetry.queue_accepted,
             telemetry.queue_rejected,
             telemetry.queue_overwritten,
             telemetry.telemetry_packets_prepared,
             telemetry.telemetry_type2_sent,
             telemetry.telemetry_stale_drops,
             telemetry.telemetry_send_failures);

    if (radar_telemetry_observability_format_short_log(
            &telemetry, short_log, sizeof(short_log))) {
        (void)s3_log_info(short_log);
    }
}

/**
 * @brief  将上行重试等待拆成有限时间片，并在每个时间片后维护 telemetry 日志节流。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  delay_ms 计划阻塞的总时长，单位 ms；为 0 时立即返回。
 * @return 无；实际等待受 FreeRTOS tick 换算精度和任务调度影响。
 * 调用方式：Wi-Fi 配置/连接或 TCP 连接失败后的指数退避路径调用，保证长退避期间仍周期输出统计。
 * 线程约束：调用 vTaskDelay() 主动阻塞当前上行任务，并可能执行日志/BLE 投递；仅上行任务调用，禁止 ISR。
 */
static void radar_uplink_delay_with_telemetry_log(uint32_t delay_ms)
{
    while (delay_ms > 0U) {
        const uint32_t slice_ms =
            delay_ms > RADAR_UPLINK_TELEMETRY_LOG_INTERVAL_MS
                ? RADAR_UPLINK_TELEMETRY_LOG_INTERVAL_MS
                : delay_ms;
        vTaskDelay(pdMS_TO_TICKS(slice_ms));
        radar_uplink_log_telemetry_observability();
        delay_ms -= slice_ms;
    }
}

/**
 * @brief  按统一周期输出 TCP、重同步、遥测队列和任务栈统计，并重置区间最大帧龄。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] stats 上行任务私有累计统计；可为 NULL。输出后更新节流时间并清 max_dequeue_age_ms。
 * @return 无；参数为空或节流周期未到时不动作。
 * 调用方式：上行任务在连接失败或每轮 burst 后调用；BLE 日志失败被忽略，不影响 TCP 状态机。
 * 线程约束：仅上行任务调用；telemetry 统计 mutex 为零等待，日志/格式化可能耗时，禁止 ISR。
 * 数据边界：sent 只表示本地 send() 完成，不证明对端应用、ROS2 或 /scan 已处理。
 */
static void radar_uplink_log_stats(radar_uplink_stats_t *stats)
{
    const uint32_t now_ms = (uint32_t)esp_log_timestamp();
    radar_telemetry_queue_stats_t telemetry_stats;
    radar_telemetry_observability_stats_t telemetry_observability;
    const UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);

    if (stats == NULL ||
        (stats->report_timestamp_valid &&
         (uint32_t)(now_ms - stats->last_report_ms) < RADAR_UPLINK_STATS_INTERVAL_MS)) {
        return;
    }

    stats->last_report_ms = now_ms;
    stats->report_timestamp_valid = true;
    telemetry_observability_get_snapshot(&telemetry_observability);
    stats->telemetry_lock_drops =
        telemetry_observability.telemetry_lock_drops;
    radar_uplink_get_telemetry_stats(&telemetry_stats);
    ESP_LOGI(TAG,
             "RADAR_UPLINK_STATS sent=%" PRIu32 " bytes=%" PRIu64
             " send_fail=%" PRIu32 " send_timeout=%" PRIu32
             " partial=%" PRIu32 " pending_retry=%" PRIu32
             " connect_fail=%" PRIu32 " reconnect=%" PRIu32
             " sync_drop=%" PRIu32 " encode_fail=%" PRIu32
             " stale_drop=%" PRIu32 " seq_gap=%" PRIu32
             " telem_sent=%" PRIu32 " telem_bytes=%" PRIu64
             " telem_stale=%" PRIu32 " telem_encode=%" PRIu32
             " telem_q=%u telem_q_drop=%" PRIu32 " telem_q_reject=%" PRIu32
             " last_seq=%" PRIu32 " telem_seq=%" PRIu32
             " max_age_ms=%" PRIu32 " stack_hwm=%u",
             stats->successful_frames,
             stats->successful_bytes,
             stats->send_failures,
             stats->send_timeouts,
             stats->partial_writes,
             stats->pending_retries,
             stats->connect_failures,
             stats->reconnects,
             stats->resync_discarded_frames,
             stats->encode_failures,
             stats->radar_stale_drops,
             stats->radar_sequence_gaps,
             stats->telemetry_sent_frames,
             stats->telemetry_sent_bytes,
             stats->telemetry_stale_drops,
             stats->telemetry_encode_failures,
             (unsigned)telemetry_stats.depth,
             telemetry_stats.wheel.dropped,
             telemetry_stats.rejected,
             stats->last_sent_sequence,
             stats->last_telemetry_sequence,
             stats->max_dequeue_age_ms,
             (unsigned)stack_hwm);

    radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_INFO,
                         "UPLINK_STATS sent=%" PRIu32 " fail=%" PRIu32
                         " timeout=%" PRIu32 " conn_fail=%" PRIu32
                         " reconn=%" PRIu32 " seq=%" PRIu32
                         " age=%" PRIu32 " sync=%" PRIu32
                         " t_sent=%" PRIu32 " t_stale=%" PRIu32
                         " t_q=%u",
                         stats->successful_frames,
                         stats->send_failures,
                         stats->send_timeouts,
                         stats->connect_failures,
                         stats->reconnects,
                         stats->last_sent_sequence,
                         stats->max_dequeue_age_ms,
                         stats->resync_discarded_frames,
                         stats->telemetry_sent_frames,
                         stats->telemetry_stale_drops,
                         (unsigned)telemetry_stats.depth);
    stats->max_dequeue_age_ms = 0U;
}

typedef enum {
    RADAR_UPLINK_PENDING_NONE = 0,
    RADAR_UPLINK_PENDING_RADAR,
    RADAR_UPLINK_PENDING_TELEMETRY,
} radar_uplink_pending_kind_t;

/**
 * @brief  丢弃过期或编码失败项，直到取得一条可发送 telemetry S3RD 包或队列暂空。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[out] entry 非 NULL；每次 pop 都会写入并消费一个本地 telemetry 队列项。
 * @param[out] packet 非 NULL、至少可写 packet_capacity 字节的 S3RD 输出缓冲。
 * @param  packet_capacity packet 的字节容量。
 * @param[out] packet_length 非 NULL；成功时写完整 S3RD 包长，失败时不得使用其值。
 * @param[in,out] uplink_sequence 非 NULL；成功编码后递增且跳过 0，失败时不推进。
 * @param[in,out] stats 非 NULL；累计过期丢弃和编码失败计数。
 * @return 成功准备一包为 true；参数无效、队列空/锁忙或所有已取项均被丢弃时为 false。
 * 调用方式：仅上行任务在没有 pending 包时调用；false 不等于链路故障，已弹出的过期/坏项不会回队。
 * 线程约束：可能循环复制/编码多帧并短暂取 telemetry mutex，禁止 ISR；packet/entry 由上行任务独占。
 */
static bool radar_uplink_prepare_telemetry_packet(
    radar_telemetry_entry_t *entry,
    uint8_t *packet,
    size_t packet_capacity,
    size_t *packet_length,
    uint32_t *uplink_sequence,
    radar_uplink_stats_t *stats)
{
    if (entry == NULL || packet == NULL || packet_length == NULL ||
        uplink_sequence == NULL || stats == NULL) {
        return false;
    }

    for (;;) {
        if (!radar_uplink_pop_telemetry(entry)) {
            return false;
        }

        const uint32_t now_ms =
            (uint32_t)((uint64_t)esp_timer_get_time() / UINT64_C(1000));
        if (radar_telemetry_age_is_stale(
                now_ms,
                entry->ingress_timestamp_ms,
                RADAR_UPLINK_MAX_TELEMETRY_AGE_MS)) {
            ++stats->telemetry_stale_drops;
            telemetry_observability_note_stale_drop();
            continue;
        }

        uint32_t sequence = *uplink_sequence + 1U;
        if (sequence == 0U) {
            sequence = 1U;
        }
        const radar_uplink_status_t status = radar_uplink_encode_envelope(
            entry->data,
            entry->length,
            RADAR_UPLINK_MESSAGE_SRP_TELEMETRY_EXPERIMENTAL,
            0U,
            (uint32_t)CONFIG_SMARTCAR_RADAR_UPLINK_DEVICE_ID,
            (uint32_t)CONFIG_SMARTCAR_RADAR_UPLINK_STREAM_ID,
            sequence,
            entry->ingress_timestamp_ms,
            packet,
            packet_capacity,
            packet_length);
        if (status != RADAR_UPLINK_OK) {
            ++stats->telemetry_encode_failures;
            continue;
        }

        *uplink_sequence = sequence;
        telemetry_observability_note_packet_prepared();
        return true;
    }
}

/**
 * @brief  消费雷达 FIFO，丢弃过期或不同步帧，并准备下一条可发送 RAW_FRAME S3RD 包。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[out] frame 非 NULL 的原始帧缓冲。
 * @param  frame_capacity frame 容量，传给 radar_uart_pop_frame()。
 * @param[out] frame_length 非 NULL；每次弹出写帧长，只有返回 true 时对应待发帧。
 * @param[out] frame_sequence 非 NULL；每次弹出写 UART 帧序号。
 * @param[out] frame_timestamp_ms 非 NULL；写 UART 采集时间，单位 ms。
 * @param[out] dequeue_age_ms 非 NULL；写弹出时帧龄，单位 ms。
 * @param[out] packet 非 NULL 的 S3RD 输出缓冲。
 * @param  packet_capacity packet 容量。
 * @param[out] packet_length 非 NULL；成功时写完整包长，失败时不得使用。
 * @param[in,out] waiting_for_zero_packet 非 NULL；过期、序号断层或编码失败时置 true，零包可通过门控。
 * @param[in,out] radar_sequence_valid 非 NULL；异常时清 false，成功准备不会在此置 true。
 * @param  last_radar_sequence 非 NULL；读取最近成功发送的 UART 帧序号，函数不在此更新。
 * @param[out] zero_packet 非 NULL；成功时标记本次包是否为零包。
 * @param[in,out] uplink_sequence 非 NULL；成功编码后递增且跳过 0。
 * @param[in,out] stats 非 NULL；累计过期、序号断层、重同步丢弃和编码失败计数。
 * @return 成功准备一包为 true；参数无效、FIFO 暂无可取帧或所有候选被丢弃时为 false。
 * 调用方式：上行任务在无 pending 包时调用；返回前可能消费多条帧，非零帧在等待零包期间直接丢弃。
 * 线程约束：会反复取 FIFO mutex、校验并编码完整帧，禁止 ISR；所有状态指针由单一上行任务拥有。
 */
static bool radar_uplink_prepare_radar_packet(
    uint8_t *frame,
    size_t frame_capacity,
    size_t *frame_length,
    uint32_t *frame_sequence,
    uint32_t *frame_timestamp_ms,
    uint32_t *dequeue_age_ms,
    uint8_t *packet,
    size_t packet_capacity,
    size_t *packet_length,
    bool *waiting_for_zero_packet,
    bool *radar_sequence_valid,
    uint32_t *last_radar_sequence,
    bool *zero_packet,
    uint32_t *uplink_sequence,
    radar_uplink_stats_t *stats)
{
    if (frame == NULL || frame_length == NULL || frame_sequence == NULL ||
        frame_timestamp_ms == NULL || dequeue_age_ms == NULL || packet == NULL ||
        packet_length == NULL || waiting_for_zero_packet == NULL ||
        radar_sequence_valid == NULL || last_radar_sequence == NULL ||
        zero_packet == NULL || uplink_sequence == NULL || stats == NULL) {
        return false;
    }

    while (radar_uart_pop_frame(frame,
                                frame_capacity,
                                frame_length,
                                frame_sequence,
                                frame_timestamp_ms,
                                dequeue_age_ms)) {
        if (*frame_sequence == 0U) {
            ++stats->encode_failures;
            *waiting_for_zero_packet = true;
            *radar_sequence_valid = false;
            continue;
        }
        if (*dequeue_age_ms > RADAR_UPLINK_MAX_RADAR_DEQUEUE_AGE_MS) {
            ++stats->radar_stale_drops;
            *waiting_for_zero_packet = true;
            *radar_sequence_valid = false;
            continue;
        }

        const bool is_zero = radar_uplink_frame_is_zero_packet(frame,
                                                                *frame_length);
        if (!*waiting_for_zero_packet && *radar_sequence_valid &&
            *frame_sequence != *last_radar_sequence + 1U) {
            ++stats->radar_sequence_gaps;
            *waiting_for_zero_packet = true;
            *radar_sequence_valid = false;
            if (!is_zero) {
                ++stats->resync_discarded_frames;
                continue;
            }
        }
        if (*waiting_for_zero_packet && !is_zero) {
            ++stats->resync_discarded_frames;
            continue;
        }

        uint32_t packet_sequence = *uplink_sequence + 1U;
        if (packet_sequence == 0U) {
            packet_sequence = 1U;
        }
        if (radar_uplink_encode_frame(
                frame,
                *frame_length,
                (uint32_t)CONFIG_SMARTCAR_RADAR_UPLINK_DEVICE_ID,
                (uint32_t)CONFIG_SMARTCAR_RADAR_UPLINK_STREAM_ID,
                packet_sequence,
                *frame_timestamp_ms,
                packet,
                packet_capacity,
                packet_length) != RADAR_UPLINK_OK) {
            ++stats->encode_failures;
            *waiting_for_zero_packet = true;
            *radar_sequence_valid = false;
            continue;
        }

        *uplink_sequence = packet_sequence;
        *zero_packet = is_zero;
        return true;
    }
    return false;
}

/**
 * @brief  管理 Wi-Fi/TCP 重连、雷达零包同步、双类队列调度和非阻塞分片发送。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  context xTaskCreate() 传入的任务上下文，当前固定为 NULL 并忽略。
 * @return 无；FreeRTOS 任务进入永久循环，不应返回。
 * 调用方式：仅由 radar_uplink_init() 创建一次；通知或最多 100 ms 超时唤醒，
 *           Wi-Fi/TCP 失败使用有界指数退避。
 * 线程约束：低优先级任务独占 socket、发送状态和大块栈缓冲；会因 DNS/connect/backoff 延迟，
 *           禁止手工或 ISR 调用。
 * 重同步语义：连接断开会放弃部分发送包；雷达 RAW_FRAME 恢复前必须先遇到零包，
 *             telemetry 队列独立有界调度。
 * 发送边界：COMPLETE 仅表示本地 socket 接受全部字节，不代表 Windows/ROS2 已处理。
 */
static void radar_uplink_task(void *context)
{
    (void)context;
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    uint8_t packet[RADAR_UPLINK_MAX_PACKET_SIZE];
    size_t frame_length = 0U;
    size_t packet_length = 0U;
    uint32_t radar_frame_sequence = 0U;
    uint32_t uplink_sequence = 0U;
    uint32_t timestamp_ms = 0U;
    uint32_t pending_sequence = 0U;
    uint32_t pending_radar_frame_sequence = 0U;
    uint32_t retry_ms = RADAR_UPLINK_RETRY_INITIAL_MS;
    bool wifi_connected_logged = false;
    bool pending_packet = false;
    bool pending_zero_packet = false;
    bool waiting_for_zero_packet = true;
    bool radar_sequence_valid = false;
    uint32_t last_radar_sequence = 0U;
    bool telemetry_turn = false;
    radar_uplink_pending_kind_t pending_kind = RADAR_UPLINK_PENDING_NONE;
    radar_telemetry_entry_t telemetry_entry;
    radar_uplink_tx_state_t tx_state = {0};
    radar_uplink_stats_t stats = {0};

    for (;;) {
        (void)ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(RADAR_UPLINK_WAIT_MS));
        radar_uplink_log_telemetry_observability();
        if (!smartcar_wifi_sta_is_connected()) {
            wifi_connected_logged = false;
            close_socket();
            if (pending_packet &&
                pending_kind == RADAR_UPLINK_PENDING_TELEMETRY) {
                telemetry_observability_note_type2_tx_result(
                    RADAR_UPLINK_TX_FAILED);
            }
            radar_uplink_reset_after_disconnect(&pending_packet,
                                                &tx_state,
                                                &waiting_for_zero_packet,
                                                &stats);
            pending_zero_packet = false;
            pending_kind = RADAR_UPLINK_PENDING_NONE;
            radar_sequence_valid = false;
            telemetry_turn = false;
            continue;
        }
        if (!wifi_connected_logged) {
            radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_INFO, "WIFI CONNECTED");
            wifi_connected_logged = true;
        }
        if (s_socket < 0) {
            s_socket = connect_endpoint();
            if (s_socket < 0) {
                ++stats.connect_failures;
                radar_uplink_log_stats(&stats);
                ESP_LOGW(TAG, "TCP CONNECT FAILED");
                radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_WARN,
                                     "TCP CONNECT FAILED");
                radar_uplink_delay_with_telemetry_log(retry_ms);
                retry_ms = retry_ms < RADAR_UPLINK_RETRY_MAX_MS / 2U
                               ? retry_ms * 2U
                               : RADAR_UPLINK_RETRY_MAX_MS;
                continue;
            }
            retry_ms = RADAR_UPLINK_RETRY_INITIAL_MS;
            ++stats.reconnects;
            waiting_for_zero_packet = true;
            radar_sequence_valid = false;
            telemetry_turn = false;
            ESP_LOGI(TAG, "TCP CONNECTED");
            radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_INFO, "TCP CONNECTED");
        }

        bool send_waited = false;
        for (uint32_t sent_in_burst = 0U;
             sent_in_burst < RADAR_UPLINK_BURST_MAX_FRAMES;
             ++sent_in_burst) {
            uint32_t dequeue_age_ms = 0U;
            if (!pending_packet) {
                bool packet_ready = false;
                /* A reconnect must establish the radar ring boundary first;
                 * once synchronized, alternate classes to avoid starvation. */
                const bool prefer_telemetry = telemetry_turn &&
                                               !waiting_for_zero_packet;
                if (prefer_telemetry) {
                    packet_ready = radar_uplink_prepare_telemetry_packet(
                        &telemetry_entry,
                        packet,
                        sizeof(packet),
                        &packet_length,
                        &uplink_sequence,
                        &stats);
                    if (packet_ready) {
                        pending_kind = RADAR_UPLINK_PENDING_TELEMETRY;
                        pending_sequence = uplink_sequence;
                    }
                }
                if (!packet_ready) {
                    packet_ready = radar_uplink_prepare_radar_packet(
                        frame,
                        sizeof(frame),
                        &frame_length,
                        &radar_frame_sequence,
                        &timestamp_ms,
                        &dequeue_age_ms,
                        packet,
                        sizeof(packet),
                        &packet_length,
                        &waiting_for_zero_packet,
                        &radar_sequence_valid,
                        &last_radar_sequence,
                        &pending_zero_packet,
                        &uplink_sequence,
                        &stats);
                    if (packet_ready) {
                        pending_kind = RADAR_UPLINK_PENDING_RADAR;
                        pending_sequence = uplink_sequence;
                        pending_radar_frame_sequence = radar_frame_sequence;
                        if (dequeue_age_ms > stats.max_dequeue_age_ms) {
                            stats.max_dequeue_age_ms = dequeue_age_ms;
                        }
                    }
                }
                if (!packet_ready && !prefer_telemetry) {
                    packet_ready = radar_uplink_prepare_telemetry_packet(
                        &telemetry_entry,
                        packet,
                        sizeof(packet),
                        &packet_length,
                        &uplink_sequence,
                        &stats);
                    if (packet_ready) {
                        pending_kind = RADAR_UPLINK_PENDING_TELEMETRY;
                        pending_sequence = uplink_sequence;
                    }
                }
                if (!packet_ready) {
                    break;
                }
                radar_uplink_tx_reset(&tx_state);
                pending_packet = true;
            }

            const radar_uplink_tx_result_t send_result =
                radar_uplink_tx_send(&tx_state,
                                     packet,
                                     packet_length,
                                     radar_socket_send,
                                     NULL);
            if (pending_kind == RADAR_UPLINK_PENDING_TELEMETRY) {
                telemetry_observability_note_type2_tx_result(send_result);
            }
            if (tx_state.wrote_partial) {
                ++stats.partial_writes;
            }
            if (send_result == RADAR_UPLINK_TX_WAIT) {
                ++stats.send_timeouts;
                ++stats.pending_retries;
                send_waited = true;
                break;
            }
            if (send_result == RADAR_UPLINK_TX_FAILED) {
                const int send_errno = errno;
                ++stats.send_failures;
                ESP_LOGW(TAG, "TCP SEND FAILED errno=%d", send_errno);
                radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_WARN,
                                     "TCP SEND FAILED errno=%d", send_errno);
                close_socket();
                radar_uplink_reset_after_disconnect(&pending_packet,
                                                    &tx_state,
                                                    &waiting_for_zero_packet,
                                                    &stats);
                pending_zero_packet = false;
                pending_kind = RADAR_UPLINK_PENDING_NONE;
                radar_sequence_valid = false;
                telemetry_turn = false;
                break;
            }

            pending_packet = false;
            radar_uplink_tx_reset(&tx_state);
            ++stats.successful_frames;
            stats.successful_bytes += packet_length;
            if (pending_kind == RADAR_UPLINK_PENDING_RADAR) {
                stats.last_sent_sequence = pending_sequence;
                last_radar_sequence = pending_radar_frame_sequence;
                radar_sequence_valid = true;
                if (pending_zero_packet) {
                    waiting_for_zero_packet = false;
                }
                telemetry_turn = true;
            } else if (pending_kind == RADAR_UPLINK_PENDING_TELEMETRY) {
                ++stats.telemetry_sent_frames;
                stats.telemetry_sent_bytes += packet_length;
                stats.last_telemetry_sequence = pending_sequence;
                telemetry_turn = false;
            }
            pending_zero_packet = false;
            pending_kind = RADAR_UPLINK_PENDING_NONE;
        }
        radar_uplink_log_stats(&stats);
        if (send_waited) {
            vTaskDelay(RADAR_UPLINK_RETRY_YIELD_TICKS);
        }
    }
}
#endif

/**
 * @brief  启动可选 Wi-Fi STA、telemetry 队列/sink 和低优先级 TCP 上行任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return Kconfig 关闭时返回 ESP_OK 但不创建资源；启用时 ESP_OK 表示本地 Wi-Fi/队列/sink/任务启动完成，
 *         重复调用、配置、内存或 ESP-IDF 初始化失败返回对应错误。
 * 调用方式：app_main() 在 radar_uart_init() 后、smartcar_service_init() 前调用一次；
 *           启用时需有效凭据、主机和端口。
 * 线程约束：仅系统启动任务调用；会分配 PSRAM/RTOS 对象、注册事件回调并启动 Wi-Fi/任务，
 *           禁止 ISR 或并发调用。
 * 失败语义：部分中后段错误不会完整撤销 netif、事件处理器或 Wi-Fi 状态，失败后不保证可直接重试。
 * 验证边界：成功不表示已获 IP、TCP 已连接、对端收包、ROS2 已发布或雷达数据有效。
 */
esp_err_t radar_uplink_init(void)
{
#if !CONFIG_SMARTCAR_RADAR_UPLINK_ENABLED
    ESP_LOGI(TAG, "DISABLED");
    return ESP_OK;
#else
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (CONFIG_SMARTCAR_RADAR_UPLINK_HOST[0] == '\0') {
        ESP_LOGE(TAG, "CONFIG INCOMPLETE; DISABLED");
        radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_ERROR, "CONFIG INCOMPLETE; DISABLED");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = smartcar_wifi_sta_start();
    if (ret != ESP_OK) {
        radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_ERROR,
                             "WIFI STA OWNER FAILED err=%s", esp_err_to_name(ret));
        return ret;
    }

    radar_telemetry_observability_init(&s_telemetry_observability);
    s_telemetry_observability_last_log_ms = 0U;
    s_telemetry_observability_log_timestamp_valid = false;
    ret = telemetry_queue_prepare();
    if (ret != ESP_OK) {
        radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_ERROR,
                             "TELEMETRY FIFO INIT FAILED err=%s",
                             esp_err_to_name(ret));
        return ret;
    }
    ret = smartcar_service_set_telemetry_sink(radar_uplink_telemetry_sink,
                                               &s_telemetry_queue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TELEMETRY SINK REGISTER FAILED: %s", esp_err_to_name(ret));
        telemetry_queue_release();
        return ret;
    }

    if (xTaskCreate(radar_uplink_task,
                   "radar_uplink",
                   RADAR_UPLINK_TASK_STACK_SIZE,
                   NULL,
                   RADAR_UPLINK_TASK_PRIORITY,
                   &s_uplink_task) != pdPASS) {
        radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_ERROR, "UPLINK TASK CREATE FAILED");
        telemetry_queue_release();
        return ESP_ERR_NO_MEM;
    }
    radar_uart_set_frame_notification_task(s_uplink_task);
    s_initialized = true;
    ESP_LOGI(TAG, "READY TCP uplink");
    ESP_LOGI(TAG, "%s src=%s", RADAR_UPLINK_TELEMETRY_FEATURE_ID,
             SMARTCAR_TELEMETRY_SOURCE_SHA8);
    radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_INFO, "READY TCP uplink");
    radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_INFO, "%s src=%s",
                         RADAR_UPLINK_TELEMETRY_FEATURE_ID,
                         SMARTCAR_TELEMETRY_SOURCE_SHA8);
    return ESP_OK;
#endif
}

/**
 * @brief  查询实验性上行功能是否完成本地初始化并保有 worker 句柄。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return Kconfig 关闭时固定 false；启用时 initialized、Wi-Fi started 和任务句柄均有效才为 true。
 * 调用方式：仅作本地状态/诊断读取；不代表当前有 IP、socket 已连接或对端/ROS2 已消费。
 * 线程约束：无锁快照、不阻塞；初始化完成后读取，禁止把该值作为跨任务同步屏障。
 */
bool radar_uplink_is_running(void)
{
#if !CONFIG_SMARTCAR_RADAR_UPLINK_ENABLED
    return false;
#else
    return s_initialized && s_uplink_task != NULL;
#endif
}
