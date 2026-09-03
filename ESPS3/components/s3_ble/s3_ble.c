#include "s3_ble.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "s3_ble_log_tx.h"
#include "smartcar_log.h"

/* S3 BLE GATT 实现；创建人：待确认（当前维护人：Zhiqin）。 */

#define S3_BLE_DEVICE_NAME "SmartCar_S3"
#define S3_BLE_MAX_RX_LEN 1032U
#define S3_BLE_ADV_CONFIG_FLAG BIT0
#define S3_BLE_SCAN_RSP_CONFIG_FLAG BIT1
#define S3_BLE_LOG_TX_TASK_STACK_SIZE 3072U
#define S3_BLE_LOG_TX_TASK_PRIORITY (tskIDLE_PRIORITY + 1U)
#define S3_BLE_LOG_TX_CHUNK_DELAY_TICKS 1U

/* UUID bytes use the little-endian representation required by Bluedroid. */
static const uint8_t s_service_uuid[ESP_UUID_LEN_128] = {
    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xE0, 0xFF, 0x00, 0x00,
};
static const uint8_t s_rx_uuid[ESP_UUID_LEN_128] = {
    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xE1, 0xFF, 0x00, 0x00,
};
static const uint8_t s_tx_uuid[ESP_UUID_LEN_128] = {
    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xE2, 0xFF, 0x00, 0x00,
};
static const uint8_t s_log_uuid[ESP_UUID_LEN_128] = {
    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xE3, 0xFF, 0x00, 0x00,
};

static const char *TAG = "S3_BLE";
static const uint16_t s_primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t s_character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t s_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t s_rx_property =
    ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t s_tx_property = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t s_log_property = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t s_ccc_initial_value[2] = {0x00, 0x00};

enum {
    IDX_SERVICE,
    IDX_RX_CHAR,
    IDX_RX_VALUE,
    IDX_TX_CHAR,
    IDX_TX_VALUE,
    IDX_TX_CCC,
    IDX_LOG_CHAR,
    IDX_LOG_VALUE,
    IDX_LOG_CCC,
    IDX_COUNT,
};

static uint16_t s_handles[IDX_COUNT];
static esp_gatt_if_t s_gatts_if = ESP_GATT_IF_NONE;
static volatile uint16_t s_conn_id;

typedef struct {
    bool connected;
    bool notify_enabled;
    bool log_notify_enabled;
    bool ready;
} ble_state_t;

static volatile ble_state_t s_ble_state;
static bool s_initialized;
static uint8_t s_adv_config_done;
static bool s_service_ready;
static bool s_adv_start_requested;
static volatile uint16_t s_att_mtu = 23U;
static uint32_t s_ble_notify_fail_count;
static s3_ble_rx_callback_t s_rx_callback;
static void *s_rx_callback_context;
static portMUX_TYPE s_rx_callback_lock = portMUX_INITIALIZER_UNLOCKED;
static s3_ble_ready_callback_t s_ready_callback;
static void *s_ready_callback_context;
static s3_ble_disconnect_callback_t s_disconnect_callback;
static void *s_disconnect_callback_context;
static s3_ble_log_tx_t s_log_tx;
static portMUX_TYPE s_log_tx_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_log_tx_task;
static bool s_log_tx_context_initialized;
static bool s_disconnect_info_valid;
static bool s_prev_disconnect_report_pending;
static uint8_t s_last_disconnect_reason;
static uint32_t s_disconnect_count;

/**
 * @brief 对一个 32 位诊断计数执行饱和加一，保持 UINT32_MAX 不回绕。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param value 非 NULL 可写计数指针；函数不保存该指针。
 * @return 无。
 * 调用方式：仅供本文件在记录通知失败或断开事件时调用。
 * 线程约束：函数自身不加锁；当前调用点均位于 s_log_tx_lock 临界区，禁止脱离同步直接并发调用或用于 ISR。
 */
static void s3_ble_saturating_increment(uint32_t *value)
{
    if (*value != UINT32_MAX) {
        ++(*value);
    }
}

/**
 * @brief 向已创建的 FFE3 TX worker 发送一次无负载任务通知。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 无；worker 尚未创建时静默返回。
 * 调用方式：日志入队以及连接、CCC、拥塞状态变化后调用，使 worker 重新评估发送条件。
 * 线程约束：使用 xTaskNotifyGive()，可从普通任务或当前 GATT 回调上下文调用，不阻塞；禁止硬件 ISR。
 */
static void s3_ble_log_tx_wake(void)
{
    TaskHandle_t task = s_log_tx_task;

    if (task != NULL) {
        (void)xTaskNotifyGive(task);
    }
}

/**
 * @brief FFE3 的唯一 GATT TX owner，按状态机串行发送完整日志帧的全部分片。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param context 固定为 NULL，当前不使用。
 * @return 不返回。
 * 调用方式：s3_ble_init() 只创建一个实例；由入队和 GATT 状态变化通知唤醒，READY 时提交 FFE3 notification。
 * 线程约束：最低业务优先级任务；无通知时无限阻塞，只在本函数调用 FFE3 send_indicate，状态访问使用短临界区，
 *           每个成功分片后让出一个 tick；禁止其他线程直接发送 FFE3 以免帧分片交织。
 */
static void s3_ble_log_tx_worker(void *context)
{
    (void)context;

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        for (;;) {
            s3_ble_log_tx_chunk_t chunk;
            s3_ble_log_tx_prepare_result_t prepare_result;
            uint16_t conn_id;
            uint16_t max_payload;

            portENTER_CRITICAL(&s_log_tx_lock);
            max_payload = s_att_mtu > 3U ? (uint16_t)(s_att_mtu - 3U) : 20U;
            prepare_result = s3_ble_log_tx_prepare_chunk(&s_log_tx,
                                                         max_payload,
                                                         &chunk);
            conn_id = s_conn_id;
            portEXIT_CRITICAL(&s_log_tx_lock);

            if (prepare_result != S3_BLE_LOG_TX_PREPARE_READY) {
                break;
            }

            const esp_err_t ret = esp_ble_gatts_send_indicate(
                s_gatts_if, conn_id, s_handles[IDX_LOG_VALUE],
                chunk.length, chunk.data, false);

            portENTER_CRITICAL(&s_log_tx_lock);
            (void)s3_ble_log_tx_complete_chunk(&s_log_tx, chunk.token,
                                               ret == ESP_OK);
            if (ret != ESP_OK) {
                s3_ble_saturating_increment(&s_ble_notify_fail_count);
            }
            portEXIT_CRITICAL(&s_log_tx_lock);

            if (ret != ESP_OK) {
                break;
            }
            vTaskDelay((TickType_t)S3_BLE_LOG_TX_CHUNK_DELAY_TICKS);
        }
    }
}

/**
 * @brief 将一段 S3 文本编码为调用方拥有的完整独立日志帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param level 日志级别，调用方应保证不大于 SMARTCAR_LOG_LEVEL_ERROR。
 * @param timestamp_ms 写入日志包络的毫秒时间戳。
 * @param text 只读文本字节，至少在本函数返回前有效；不要求由本函数读取 NUL 终止符。
 * @param text_length 要编码的文本字节数，不得超过 SMARTCAR_LOG_MAX_PAYLOAD。
 * @param frame 至少 SMARTCAR_LOG_MAX_FRAME_SIZE 字节的可写缓冲。
 * @param frame_length 成功时写入完整帧长度。
 * @return 编码成功返回 ESP_OK，否则返回 ESP_FAIL。
 * 调用方式：由 s3_ble_log_emit() 调用，随后复制进固定 FFE3 队列。
 * 线程约束：纯编码，不调用 Bluedroid；普通任务或有界 GATT 标记生成路径可调用。
 */
static esp_err_t s3_ble_log_encode_text(smartcar_log_level_t level,
                                        uint32_t timestamp_ms,
                                        const char *text,
                                        uint8_t text_length,
                                        uint8_t *frame,
                                        uint16_t *frame_length)
{
    size_t encoded_length = 0U;

    if (smartcar_log_encode(SMARTCAR_LOG_SOURCE_S3, level,
                            timestamp_ms,
                            (const uint8_t *)text, text_length,
                            frame, SMARTCAR_LOG_MAX_FRAME_SIZE,
                            &encoded_length) != SMARTCAR_LOG_OK) {
        return ESP_FAIL;
    }
    *frame_length = (uint16_t)encoded_length;
    return ESP_OK;
}

static const esp_ble_adv_params_t s_adv_params = {
    /* Bluedroid uses 0.625 ms units: 160 == 100 ms. */
    .adv_int_min = 160,
    .adv_int_max = 160,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static const esp_ble_adv_data_t s_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    /* Name + flags fit in the primary 31-byte advertising payload. */
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
};

static const esp_ble_adv_data_t s_scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = false,
    .include_txpower = false,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    /* Keep the complete 128-bit service UUID in scan response data. */
    .service_uuid_len = sizeof(s_service_uuid),
    .p_service_uuid = (uint8_t *)s_service_uuid,
    .flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
};

/**
 * @brief 根据连接和 FFE2 CCC 状态更新 ready，并同步发出由 false 到 true 的边沿回调。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）。
 * 调用方式：由 GATTS 连接、断开和 FFE2 CCC 写事件调用；FFE3 日志 CCC 不参与 ready 计算。
 * 线程约束：运行于 Bluedroid GATT 回调上下文；回调指针无锁读取并在当前栈同步执行，注册必须与事件安全协调，回调不得阻塞或直接控制硬件。
 */
static void update_ready_state(void)
{
    const bool ready = s_ble_state.connected && s_ble_state.notify_enabled;
    const bool became_ready = ready && !s_ble_state.ready;

    s_ble_state.ready = ready;
    if (became_ready && s_ready_callback != NULL) {
        s_ready_callback(s_ready_callback_context);
    }
}

/**
 * @brief 校验一次 FFE1 写入，并把 GATT 借用缓冲同步交给当前注册消费者。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data Bluedroid GATT 拥有的只读写入缓冲，仅在上层 GATT 回调期间有效。
 * @param length 本次写入字节数；0 或大于 S3_BLE_MAX_RX_LEN 时丢弃。
 * @return 返回值：无（void）；无消费者或参数非法时静默丢弃。
 * 调用方式：只由 gatts_event_handler() 的非 prepare FFE1 写事件调用；消费者必须在返回前复制或入队。
 * 线程约束：运行于 GATT 回调上下文；仅在短临界区复制回调指针/上下文，回调在解锁后同步执行，不得阻塞、保留 data 或直接驱动硬件。
 */
static void s3_ble_dispatch_rx_write(const uint8_t *data, size_t length)
{
    s3_ble_rx_callback_t callback;
    void *context;

    if (data == NULL || length == 0U || length > S3_BLE_MAX_RX_LEN) {
        return;
    }

    portENTER_CRITICAL(&s_rx_callback_lock);
    callback = s_rx_callback;
    context = s_rx_callback_context;
    portEXIT_CRITICAL(&s_rx_callback_lock);

    if (callback != NULL) {
        callback(data, length, context);
    }
}

static const esp_gatts_attr_db_t s_gatt_db[IDX_COUNT] = {
    [IDX_SERVICE] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&s_primary_service_uuid, ESP_GATT_PERM_READ,
         sizeof(s_service_uuid), sizeof(s_service_uuid), (uint8_t *)s_service_uuid},
    },
    [IDX_RX_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&s_character_declaration_uuid, ESP_GATT_PERM_READ,
         sizeof(s_rx_property), sizeof(s_rx_property), (uint8_t *)&s_rx_property},
    },
    [IDX_RX_VALUE] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t *)s_rx_uuid, ESP_GATT_PERM_WRITE,
         S3_BLE_MAX_RX_LEN, 0, NULL},
    },
    [IDX_TX_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&s_character_declaration_uuid, ESP_GATT_PERM_READ,
         sizeof(s_tx_property), sizeof(s_tx_property), (uint8_t *)&s_tx_property},
    },
    [IDX_TX_VALUE] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t *)s_tx_uuid, ESP_GATT_PERM_READ,
         S3_BLE_MAX_RX_LEN, 0, NULL},
    },
    [IDX_TX_CCC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&s_client_config_uuid,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(s_ccc_initial_value),
         sizeof(s_ccc_initial_value), (uint8_t *)s_ccc_initial_value},
    },
    [IDX_LOG_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&s_character_declaration_uuid, ESP_GATT_PERM_READ,
         sizeof(s_log_property), sizeof(s_log_property), (uint8_t *)&s_log_property},
    },
    [IDX_LOG_VALUE] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t *)s_log_uuid, ESP_GATT_PERM_READ,
         S3_BLE_MAX_RX_LEN, 0, NULL},
    },
    [IDX_LOG_CCC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&s_client_config_uuid,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(s_ccc_initial_value),
         sizeof(s_ccc_initial_value), (uint8_t *)s_ccc_initial_value},
    },
};

/**
 * @brief 在服务表和两份广播数据均配置完成后，至多提交一次可连接广播启动请求。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；同步提交失败时清除请求标志并记录错误，等待后续事件重试。
 * 调用方式：由 GAP 数据配置完成、GATT 服务就绪和断开事件路径调用；实际启动结果由 GAP 完成事件确认。
 * 线程约束：运行于 Bluedroid GAP/GATT 回调上下文，读写无锁 BLE 状态并调用 GAP API；禁止 ISR 或其他任务并发调用。
 */
static void start_advertising_if_ready(void)
{
    if (s_service_ready &&
        s_adv_config_done == (S3_BLE_ADV_CONFIG_FLAG | S3_BLE_SCAN_RSP_CONFIG_FLAG) &&
        !s_adv_start_requested) {
        s_adv_start_requested = true;
        ESP_LOGI(TAG, "[S3_BLE] ADV PAYLOAD READY");
        esp_err_t ret = esp_ble_gap_start_advertising((esp_ble_adv_params_t *)&s_adv_params);
        if (ret != ESP_OK) {
            s_adv_start_requested = false;
            ESP_LOGE(TAG, "ADV START FAILED rc=%d", (int)ret);
        }
    }
}

/**
 * @brief 处理广播数据配置和广播启动完成事件，维护广播提交状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param event Bluedroid GAP 事件类型；未识别事件被忽略。
 * @param param Bluedroid 借用的事件参数，仅在回调期间有效；受支持事件下应为非 NULL。
 * @return 返回值：无（void）。
 * 调用方式：由 esp_ble_gap_register_callback() 注册后由 Bluedroid 同步回调；本函数不保留 param。
 * 线程约束：运行于协议栈回调上下文，可能继续提交广播和写日志；必须快速返回，禁止阻塞、ISR 调用或业务任务直接调用。
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        s_adv_config_done |= S3_BLE_ADV_CONFIG_FLAG;
        start_advertising_if_ready();
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        s_adv_config_done |= S3_BLE_SCAN_RSP_CONFIG_FLAG;
        start_advertising_if_ready();
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            s_adv_start_requested = false;
            ESP_LOGE(TAG, "ADV START FAILED rc=%d", (int)param->adv_start_cmpl.status);
        } else {
            ESP_LOGI(TAG, "[S3_BLE] ADV STARTED");
        }
        break;
    default:
        break;
    }
}

/**
 * @brief 处理 GATT 注册、服务表、连接、断开、写入和 MTU 事件并维护 FFE1/FFE2/FFE3 状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param event Bluedroid GATTS 事件类型；未处理事件被忽略。
 * @param gatts_if 当前 GATT server interface，注册事件时保存到模块状态。
 * @param param Bluedroid 借用的事件联合体，仅在回调期间有效；受支持事件下应为非 NULL。
 * @return 返回值：无（void）；属性表失败仅记录并返回，部分注册配置使用 ESP_ERROR_CHECK，失败会触发系统 abort/reset 语义。
 * 调用方式：由 esp_ble_gatts_register_callback() 注册后由 Bluedroid 调用；写入路径会同步调用已注册业务回调。
 * 线程约束：协议栈回调上下文；不得保留 param/write.value，断开/ready/RX 回调必须快速返回；FFE3 事件只更新状态、生成有限标记并唤醒 worker，不执行通知循环。
 */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        s_gatts_if = gatts_if;
        ESP_ERROR_CHECK(esp_ble_gap_set_device_name(S3_BLE_DEVICE_NAME));
        ESP_LOGI(TAG, "[S3_BLE] ADV CONFIG name=%s ad_type=0x%02X flags=0x%02X",
                 S3_BLE_DEVICE_NAME, ESP_BLE_AD_TYPE_NAME_CMPL, s_adv_data.flag);
        ESP_LOGI(TAG, "[S3_BLE] ADV FIELDS flags=0x%02X name=%s",
                 s_adv_data.flag, S3_BLE_DEVICE_NAME);
        ESP_LOGI(TAG, "[S3_BLE] RSP FIELDS service_uuid_len=%u",
                 (unsigned)s_scan_rsp_data.service_uuid_len);
        ESP_LOGI(TAG, "[S3_BLE] SERVICE UUID=0000FFE0-0000-1000-8000-00805F9B34FB");
        ESP_LOGI(TAG, "[S3_BLE] ADV PARAMS type=ADV_TYPE_IND conn=UND disc=GEN interval=100ms");
        ESP_ERROR_CHECK(esp_ble_gap_config_adv_data((esp_ble_adv_data_t *)&s_adv_data));
        ESP_ERROR_CHECK(esp_ble_gap_config_adv_data((esp_ble_adv_data_t *)&s_scan_rsp_data));
        esp_ble_gatts_create_attr_tab(s_gatt_db, gatts_if, IDX_COUNT, 0);
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK ||
            param->add_attr_tab.num_handle != IDX_COUNT) {
            ESP_LOGE(TAG, "BLE GATT table creation failed: status=%d handles=%d",
                     param->add_attr_tab.status, param->add_attr_tab.num_handle);
            return;
        }
        memcpy(s_handles, param->add_attr_tab.handles, sizeof(s_handles));
        esp_ble_gatts_start_service(s_handles[IDX_SERVICE]);
        s_service_ready = true;
        ESP_LOGI(TAG, "BLE SERVICE READY");
        start_advertising_if_ready();
        break;
    case ESP_GATTS_CONNECT_EVT:
        s_ble_state.connected = true;
        s_ble_state.notify_enabled = false;
        s_ble_state.log_notify_enabled = false;
        update_ready_state();
        portENTER_CRITICAL(&s_log_tx_lock);
        s_att_mtu = 23U;
        s_conn_id = param->connect.conn_id;
        s3_ble_log_tx_set_connected(&s_log_tx, true);
        portEXIT_CRITICAL(&s_log_tx_lock);
        s3_ble_log_tx_wake();
        ESP_LOGI(TAG, "[S3_BLE] CLIENT CONNECTED");
        ESP_LOGI(TAG, "conn_id=%u", (unsigned)param->connect.conn_id);
        ESP_LOGI(TAG, "addr=" ESP_BD_ADDR_STR,
                 ESP_BD_ADDR_HEX(param->connect.remote_bda));
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        s_ble_state.connected = false;
        s_ble_state.notify_enabled = false;
        s_ble_state.log_notify_enabled = false;
        update_ready_state();
        portENTER_CRITICAL(&s_log_tx_lock);
        s_disconnect_info_valid = true;
        s_prev_disconnect_report_pending = true;
        s_last_disconnect_reason = param->disconnect.reason;
        s3_ble_saturating_increment(&s_disconnect_count);
        s3_ble_log_tx_set_connected(&s_log_tx, false);
        portEXIT_CRITICAL(&s_log_tx_lock);
        s3_ble_log_tx_wake();
        if (s_disconnect_callback != NULL) {
            s_disconnect_callback(s_disconnect_callback_context);
        }
        s_adv_start_requested = false;
        ESP_LOGI(TAG, "[S3_BLE] CLIENT DISCONNECTED");
        ESP_LOGI(TAG, "reason=0x%02x", (unsigned)param->disconnect.reason);
        start_advertising_if_ready();
        break;
    case ESP_GATTS_WRITE_EVT:
        if (!param->write.is_prep && param->write.handle == s_handles[IDX_RX_VALUE]) {
            s3_ble_dispatch_rx_write(param->write.value, param->write.len);
        } else if (!param->write.is_prep && param->write.handle == s_handles[IDX_TX_CCC] &&
                   param->write.len >= sizeof(uint16_t)) {
            uint16_t ccc = (uint16_t)param->write.value[0] |
                           ((uint16_t)param->write.value[1] << 8);
            s_ble_state.notify_enabled = (ccc & 0x0001U) != 0U;
            update_ready_state();
        } else if (!param->write.is_prep && param->write.handle == s_handles[IDX_LOG_CCC] &&
                   param->write.len >= sizeof(uint16_t)) {
            const uint16_t ccc = (uint16_t)param->write.value[0] |
                                 ((uint16_t)param->write.value[1] << 8);
            const bool enabled = (ccc & 0x0001U) != 0U;

            if (enabled) {
                bool report_previous_disconnect;
                uint8_t previous_reason;
                uint32_t previous_count;
                char previous_disconnect_log[64];

                /* Queue markers while CCC is still false so the worker cannot race them. */
                (void)s3_ble_log_emit(SMARTCAR_LOG_LEVEL_INFO, "BOOT");
                if (s_ble_state.connected) {
                    (void)s3_ble_log_emit(SMARTCAR_LOG_LEVEL_INFO,
                                          "BLE_CONNECTED");
                }

                portENTER_CRITICAL(&s_log_tx_lock);
                report_previous_disconnect = s_prev_disconnect_report_pending;
                previous_reason = s_last_disconnect_reason;
                previous_count = s_disconnect_count;
                portEXIT_CRITICAL(&s_log_tx_lock);

                if (report_previous_disconnect) {
                    const int written = snprintf(
                        previous_disconnect_log, sizeof(previous_disconnect_log),
                        "BLE_PREV_DISC reason=0x%02X count=%lu",
                        (unsigned)previous_reason, (unsigned long)previous_count);
                    if (written > 0 && (size_t)written < sizeof(previous_disconnect_log) &&
                        s3_ble_log_emit(SMARTCAR_LOG_LEVEL_WARN,
                                        previous_disconnect_log) == ESP_OK) {
                        portENTER_CRITICAL(&s_log_tx_lock);
                        if (s_disconnect_count == previous_count &&
                            s_last_disconnect_reason == previous_reason) {
                            s_prev_disconnect_report_pending = false;
                        }
                        portEXIT_CRITICAL(&s_log_tx_lock);
                    }
                }
            }
            s_ble_state.log_notify_enabled = enabled;
            portENTER_CRITICAL(&s_log_tx_lock);
            s3_ble_log_tx_set_ccc_enabled(&s_log_tx, enabled);
            portEXIT_CRITICAL(&s_log_tx_lock);
            s3_ble_log_tx_wake();
        }
        break;
    case ESP_GATTS_MTU_EVT:
        if (param->mtu.mtu >= 23U) {
            portENTER_CRITICAL(&s_log_tx_lock);
            s_att_mtu = param->mtu.mtu;
            portEXIT_CRITICAL(&s_log_tx_lock);
        }
        break;
    case ESP_GATTS_CONGEST_EVT:
        if (param->congest.conn_id == s_conn_id) {
            portENTER_CRITICAL(&s_log_tx_lock);
            s3_ble_log_tx_set_congested(&s_log_tx, param->congest.congested);
            portEXIT_CRITICAL(&s_log_tx_lock);
            s3_ble_log_tx_wake();
        }
        break;
    default:
        break;
    }
}

/**
 * @brief 初始化 BLE controller/Bluedroid，注册 GATT/GAP 回调并提交应用注册。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return ESP_OK 表示初始化步骤和异步 app 注册已提交；否则返回首个 ESP-IDF 错误，函数不执行完整回滚。
 * 调用方式：由 app_main 启动路径调用；成功后的重复调用直接返回 ESP_OK，但服务表/广播 ready 仍需等待异步回调。
 * 线程约束：仅启动任务串行调用；可能分配 controller/Bluedroid 资源，禁止 ISR/GATT 回调或并发初始化，返回成功不证明客户端已连接。
 */
esp_err_t s3_ble_init(void)
{
    esp_err_t ret;

    if (s_initialized) {
        return ESP_OK;
    }
    if (!s_log_tx_context_initialized) {
        s3_ble_log_tx_init(&s_log_tx);
        s_log_tx_context_initialized = true;
    }
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    ret = esp_bluedroid_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret != ESP_OK) {
        return ret;
    }
    if (s_log_tx_task == NULL &&
        xTaskCreate(s3_ble_log_tx_worker, "ble_log_tx",
                    S3_BLE_LOG_TX_TASK_STACK_SIZE, NULL,
                    S3_BLE_LOG_TX_TASK_PRIORITY,
                    &s_log_tx_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ret = esp_ble_gatts_app_register(0);
    if (ret != ESP_OK) {
        return ret;
    }
    s_initialized = true;
    ESP_LOGI(TAG, "BLE INIT OK");
    ESP_LOGI(TAG, "BLE DEVICE NAME: %s", S3_BLE_DEVICE_NAME);
    return ESP_OK;
}

/**
 * @brief 按当前 ATT MTU 分片并向 FFE2 提交一条状态/控制响应通知流。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data 只读完整 App 帧；每次 Bluedroid 提交调用返回前必须保持有效。
 * @param len 完整 App 帧长度，范围为 1..S3_BLE_MAX_RX_LEN（当前为 1032 字节）。
 * @return 全部分片提交成功返回 ESP_OK；参数或未就绪返回对应错误，底层分片失败返回其错误并增加失败计数。
 * 调用方式：服务任务编码 App 帧后调用；使用无确认 notification，ESP_OK 不代表客户端收到或重组完成；
 *           当前只由底层返回值反映拥塞，本函数没有独立的 FFE2 拥塞状态门。
 * 线程约束：读取无锁连接/MTU 状态并调用 GATT API；禁止硬件 ISR，避免多个任务并发发送导致分片交织，断开可与发送竞争。
 */
esp_err_t s3_ble_notify_send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U || len > S3_BLE_MAX_RX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized || !s_ble_state.ready) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint16_t max_payload = s_att_mtu > 3U ? (uint16_t)(s_att_mtu - 3U) : 20U;
    uint16_t offset = 0U;
    while (offset < len) {
        uint16_t chunk_len = (uint16_t)(len - offset);
        if (chunk_len > max_payload) {
            chunk_len = max_payload;
        }
        esp_err_t ret = esp_ble_gatts_send_indicate(s_gatts_if,
                                                    s_conn_id,
                                                    s_handles[IDX_TX_VALUE],
                                                    chunk_len,
                                                    (uint8_t *)&data[offset],
                                                    false);
        if (ret != ESP_OK) {
            portENTER_CRITICAL(&s_log_tx_lock);
            s3_ble_saturating_increment(&s_ble_notify_fail_count);
            portEXIT_CRITICAL(&s_log_tx_lock);
            return ret;
        }
        offset = (uint16_t)(offset + chunk_len);
    }
    return ESP_OK;
}

/**
 * @brief 校验并复制一条完整 SmartCarLog 帧到固定 FFE3 优先级队列。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data 只读 smartcar_log 完整帧；每次提交调用返回前必须有效。
 * @param len 待发完整日志帧字节数，范围为 1..SMARTCAR_LOG_MAX_FRAME_SIZE。
 * @return 完整帧成功复制入队返回 ESP_OK；参数、长度、CRC 或字段非法返回 ESP_ERR_INVALID_ARG。
 * 调用方式：所有 FFE3 生产者只调用本接口；ESP_OK 不表示 worker 已发送或 App 已收到。
 * 线程约束：短 portMUX 临界区内最多复制 108 字节，无 RTOS 等待；支持多任务生产者，禁止 ISR。
 */
esp_err_t s3_ble_log_notify_send(const uint8_t *data, uint16_t len)
{
    smartcar_log_record_t record;
    s3_ble_log_tx_priority_t priority;
    bool enqueued;

    if (data == NULL || len == 0U || len > SMARTCAR_LOG_MAX_FRAME_SIZE ||
        smartcar_log_decode(data, len, &record) != SMARTCAR_LOG_OK) {
        return ESP_ERR_INVALID_ARG;
    }

    priority = record.level >= SMARTCAR_LOG_LEVEL_WARN
                   ? S3_BLE_LOG_TX_PRIORITY_CRITICAL
                   : S3_BLE_LOG_TX_PRIORITY_NORMAL;
    portENTER_CRITICAL(&s_log_tx_lock);
    enqueued = s3_ble_log_tx_enqueue(&s_log_tx, priority, data, len);
    portEXIT_CRITICAL(&s_log_tx_lock);
    if (!enqueued) {
        return ESP_ERR_INVALID_ARG;
    }
    s3_ble_log_tx_wake();
    return ESP_OK;
}

/**
 * @brief 截断并编码一条 S3 来源日志，再复制进固定 FFE3 优先级队列。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param level 日志级别，必须不大于 SMARTCAR_LOG_LEVEL_ERROR。
 * @param text NUL 结尾只读文本；NULL 或空字符串无效，超过协议上限时只取前 SMARTCAR_LOG_MAX_PAYLOAD 字节。
 * @return 参数非法返回 ESP_ERR_INVALID_ARG；编码/入队成功返回 ESP_OK，队满时丢各自队列最旧帧并仍接收新帧。
 * 调用方式：普通任务调用；本模块在 FFE3 CCC GATT 事件中只用它生成有限标记，不直接发送。
 * 线程约束：编码在临界区外，入队只复制完整帧；无 BLE 等待，禁止硬件 ISR。
 */
esp_err_t s3_ble_log_emit(smartcar_log_level_t level, const char *text)
{
    uint8_t frame[SMARTCAR_LOG_MAX_FRAME_SIZE];
    uint16_t frame_length = 0U;
    size_t text_length = 0U;

    if (text == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (level > SMARTCAR_LOG_LEVEL_ERROR) {
        return ESP_ERR_INVALID_ARG;
    }
    while (text[text_length] != '\0' && text_length < SMARTCAR_LOG_MAX_PAYLOAD) {
        ++text_length;
    }
    if (text_length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s3_ble_log_encode_text(level, (uint32_t)esp_log_timestamp(),
                               text, (uint8_t)text_length,
                               frame, &frame_length) != ESP_OK) {
        return ESP_FAIL;
    }
    return s3_ble_log_notify_send(frame, frame_length);
}

/**
 * @brief 以 INFO 级别编码一条 S3 诊断文本并复制进 FFE3 普通队列。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param text NUL 结尾只读文本；NULL/空文本无效，超长文本按公共日志接口截断。
 * @return 完整继承 s3_ble_log_emit() 的参数、编码和入队结果；ESP_OK 仅表示已入队。
 * 调用方式：启动、雷达和服务任务记录 INFO 事件时调用；实际发送由唯一 FFE3 worker 完成。
 * 线程约束：同 s3_ble_log_emit()；禁止硬件 ISR，不得用日志路径阻塞控制心跳或安全动作。
 */
esp_err_t s3_log_info(const char *text)
{
    return s3_ble_log_emit(SMARTCAR_LOG_LEVEL_INFO, text);
}

/**
 * @brief 以 WARN 级别编码一条 S3 诊断文本并复制进 FFE3 关键队列。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param text NUL 结尾只读文本；NULL/空文本无效，超长文本按公共日志接口截断。
 * @return 完整继承 s3_ble_log_emit() 的参数、编码和入队结果；队满覆盖关键队列最旧帧仍返回 ESP_OK。
 * 调用方式：任务上下文记录可恢复异常；不能代替状态机故障状态或重试策略。
 * 线程约束：同 s3_ble_log_emit()；禁止硬件 ISR，避免在持有控制锁时调用。
 */
esp_err_t s3_log_warn(const char *text)
{
    return s3_ble_log_emit(SMARTCAR_LOG_LEVEL_WARN, text);
}

/**
 * @brief 以 ERROR 级别编码一条 S3 诊断文本并复制进 FFE3 关键队列。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param text NUL 结尾只读文本；NULL/空文本无效，超长文本按公共日志接口截断。
 * @return 完整继承 s3_ble_log_emit() 的参数、编码和入队结果；ESP_OK 不表示手机已收到。
 * 调用方式：任务上下文记录错误；日志成功不能代替急停、故障恢复或持久故障状态。
 * 线程约束：同 s3_ble_log_emit()；禁止硬件 ISR，不得让日志失败阻断安全路径。
 */
esp_err_t s3_log_error(const char *text)
{
    return s3_ble_log_emit(SMARTCAR_LOG_LEVEL_ERROR, text);
}

/**
 * @brief 查询连接存在且客户端已打开 FFE2 CCC 的无锁 ready 快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return true 仅表示当前本地连接和 FFE2 CCC 条件满足；不证明会话有效或对端已收到通知。
 * 调用方式：服务任务发送前或低频诊断读取；不能作为控制会话唯一准入条件。
 * 线程约束：volatile 无锁读取、不阻塞，可与 GATT 状态更新竞争；禁止据此绕过服务层安全门。
 */
bool s3_ble_is_ready(void)
{
    return s_ble_state.ready;
}

/**
 * @brief 查询 BLE 已初始化、已连接且客户端已打开 FFE3 CCC 的无锁快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 三项本地条件同时满足时为 true；不保证下一次 GATT 提交一定成功。
 * 调用方式：低频诊断读取；日志生产者始终只复制完整帧入固定队列。
 * 线程约束：无锁、不阻塞，可与 GATT 连接/CCC 更新竞争；禁止硬件 ISR 依赖该快照执行发送。
 */
bool s3_ble_is_log_ready(void)
{
    return s_initialized && s_ble_state.connected &&
           s_ble_state.log_notify_enabled;
}

/**
 * @brief 在统一临界区内复制 FFE3 队列和 TX 累计统计，读取不清零。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param stats 非 NULL 调用方输出；成功时写入完整快照，不保存指针。
 * @return 复制成功返回 ESP_OK；stats 为 NULL 返回 ESP_ERR_INVALID_ARG。
 * 调用方式：任务低频读取本地日志发送健康度；成功不表示对端收到任何帧。
 * 线程约束：短暂持有 s_log_tx_lock，不阻塞等待 BLE；禁止硬件 ISR。
 */
esp_err_t s3_ble_get_log_notify_stats(s3_ble_log_notify_stats_t *stats)
{
    s3_ble_log_tx_stats_t snapshot;

    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_log_tx_lock);
    s3_ble_log_tx_get_stats(&s_log_tx, &snapshot);
    portEXIT_CRITICAL(&s_log_tx_lock);

    stats->queued = snapshot.queued;
    stats->sent_frames = snapshot.sent_frames;
    stats->sent_chunks = snapshot.sent_chunks;
    stats->drop_normal = snapshot.drop_normal;
    stats->drop_critical = snapshot.drop_critical;
    stats->send_fail = snapshot.send_fail;
    stats->congest_events = snapshot.congest_events;
    stats->partial_drop = snapshot.partial_drop;
    stats->current_depth = snapshot.current_depth;
    stats->high_watermark = snapshot.high_watermark;
    return ESP_OK;
}

/**
 * @brief 在统一临界区内复制最近一次 GATT 断开原因和饱和累计次数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param info 非 NULL 调用方输出；成功时写入 valid、原始 reason 和累计 count，不保存指针。
 * @return 复制成功返回 ESP_OK；info 为 NULL 返回 ESP_ERR_INVALID_ARG。
 * 调用方式：任务低频诊断读取；valid=false 时不得解释 reason 字段。
 * 线程约束：短暂持有 s_log_tx_lock，不清零状态；禁止硬件 ISR。
 */
esp_err_t s3_ble_get_disconnect_info(s3_ble_disconnect_info_t *info)
{
    if (info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_log_tx_lock);
    info->valid = s_disconnect_info_valid;
    info->reason = s_last_disconnect_reason;
    info->count = s_disconnect_count;
    portEXIT_CRITICAL(&s_log_tx_lock);
    return ESP_OK;
}

/**
 * @brief 读取 FFE2/FFE3 底层通知提交失败的累计诊断计数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 自启动以来 FFE2/FFE3 GATT 提交失败的饱和快照，读取不清零；前置错误和队列 drop 不计入。
 * 调用方式：仅作低频健康诊断，不能用该值确认某一通知成功或失败。
 * 线程约束：短临界区读取；禁止从 ISR 用作控制判据。
 */
uint32_t s3_ble_get_notify_fail_count(void)
{
    uint32_t count;

    portENTER_CRITICAL(&s_log_tx_lock);
    count = s_ble_notify_fail_count;
    portEXIT_CRITICAL(&s_log_tx_lock);
    return count;
}

/**
 * @brief 注册或清除 FFE2 ready 边沿回调，并在当前已 ready 时同步补发一次。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param callback ready 回调；NULL 表示清除。
 * @param context 原样借用并传给 callback，不转移所有权；允许 NULL，生命周期须覆盖注册期及在途回调。
 * @return 当前实现固定返回 ESP_OK。
 * 调用方式：优先在 BLE 初始化/连接前注册；若调用时已经 ready，回调会在本函数返回前同步执行。
 * 线程约束：注册字段无锁更新；边沿回调通常在 GATT 上下文，同步补发则在注册调用者上下文，回调不得阻塞且不得假定固定线程。
 */
esp_err_t s3_ble_set_ready_callback(s3_ble_ready_callback_t callback,
                                    void *context)
{
    s_ready_callback = callback;
    s_ready_callback_context = context;
    if (s_ble_state.ready && s_ready_callback != NULL) {
        s_ready_callback(s_ready_callback_context);
    }
    return ESP_OK;
}

/**
 * @brief 注册或清除 GATT 断开同步回调。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param callback 断开回调；NULL 表示清除。
 * @param context 原样借用并传给 callback，不转移所有权；允许 NULL，生命周期须覆盖注册期。
 * @return 当前实现固定返回 ESP_OK。
 * 调用方式：由 smartcar_service 初始化阶段注册；回调只应撤销会话/置位或入队，真正停机由服务任务完成。
 * 线程约束：注册字段无锁更新，应与 GATT 事件串行；回调运行于 Bluedroid GATT 上下文，禁止阻塞或直接驱动硬件。
 */
esp_err_t s3_ble_set_disconnect_callback(s3_ble_disconnect_callback_t callback,
                                         void *context)
{
    s_disconnect_callback = callback;
    s_disconnect_callback_context = context;
    return ESP_OK;
}

/**
 * @brief 兼容旧调用方的 FFE2 发送包装，行为等同 s3_ble_notify_send()。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data 只读完整 App 帧，发送提交期间有效。
 * @param len 待发字节数，范围为 1..S3_BLE_MAX_RX_LEN。
 * @return 原样返回 s3_ble_notify_send() 的参数、状态或底层提交结果。
 * 调用方式：仅供历史调用点迁移；新代码使用主接口并处理失败。
 * 线程约束：继承主接口约束；禁止硬件 ISR，并避免并发通知分片交织。
 */
esp_err_t s3_ble_send(const uint8_t *data, uint16_t len)
{
    return s3_ble_notify_send(data, len);
}

/**
 * @brief 原子替换或清除唯一 FFE1 写入消费者及其上下文。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param callback FFE1 同步消费者；NULL 表示清除当前消费者。
 * @param context 原样借用并传给 callback，不转移所有权；允许 NULL。
 * @return 当前实现固定返回 ESP_OK。
 * 调用方式：smartcar_service 创建 RX 队列后注册；消费者必须在回调返回前复制 GATT 数据。
 * 线程约束：指针对在 portMUX 临界区内更新；已被 GATT 路径快照的旧回调仍可能在本函数返回后执行，释放旧 context 前必须自行协调，禁止 ISR 注册。
 */
esp_err_t s3_ble_register_rx_callback(s3_ble_rx_callback_t callback,
                                      void *context)
{
    portENTER_CRITICAL(&s_rx_callback_lock);
    s_rx_callback = callback;
    s_rx_callback_context = context;
    portEXIT_CRITICAL(&s_rx_callback_lock);
    return ESP_OK;
}

/**
 * @brief 兼容旧名称的 FFE1 消费者注册包装。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param callback FFE1 同步消费者；NULL 表示清除。
 * @param context 回调借用上下文，允许 NULL，不转移所有权。
 * @return 原样返回 s3_ble_register_rx_callback()，当前固定为 ESP_OK。
 * 调用方式：仅供旧调用点兼容，新代码使用 s3_ble_register_rx_callback()。
 * 线程约束：继承主注册接口的临界区和在途回调生命周期约束；禁止 ISR 注册。
 */
esp_err_t s3_ble_set_rx_callback(s3_ble_rx_callback_t callback, void *context)
{
    return s3_ble_register_rx_callback(callback, context);
}
