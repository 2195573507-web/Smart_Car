#include "s3_ble.h"

#include <stdbool.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "smartcar_log.h"

#define S3_BLE_DEVICE_NAME "SmartCar_S3"
#define S3_BLE_MAX_RX_LEN 1032U
#define S3_BLE_ADV_CONFIG_FLAG BIT0
#define S3_BLE_SCAN_RSP_CONFIG_FLAG BIT1

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
static const uint8_t s_rx_property = ESP_GATT_CHAR_PROP_BIT_WRITE;
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
static uint16_t s_conn_id;

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
static uint16_t s_att_mtu = 23U;
static volatile uint32_t s_ble_notify_fail_count;
static s3_ble_rx_callback_t s_rx_callback;
static void *s_rx_callback_context;
static portMUX_TYPE s_rx_callback_lock = portMUX_INITIALIZER_UNLOCKED;
static s3_ble_ready_callback_t s_ready_callback;
static void *s_ready_callback_context;
static s3_ble_disconnect_callback_t s_disconnect_callback;
static void *s_disconnect_callback_context;

/* Keep boot and early bring-up events until the FFE3 subscriber is ready. */
#define S3_LOG_PENDING_CAPACITY 48U
typedef struct {
    smartcar_log_level_t level;
    uint32_t timestamp_ms;
    uint8_t length;
    char text[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
} s3_pending_log_t;

static s3_pending_log_t s_pending_logs[S3_LOG_PENDING_CAPACITY];
static uint8_t s_pending_log_head;
static uint8_t s_pending_log_tail;
static uint8_t s_pending_log_count;
static portMUX_TYPE s_pending_log_lock = portMUX_INITIALIZER_UNLOCKED;

static esp_err_t s3_ble_log_send_text(smartcar_log_level_t level,
                                      uint32_t timestamp_ms,
                                      const char *text, uint8_t text_length)
{
    uint8_t frame[SMARTCAR_LOG_MAX_FRAME_SIZE];
    size_t frame_length = 0U;

    if (smartcar_log_encode(SMARTCAR_LOG_SOURCE_S3, level,
                            timestamp_ms,
                            (const uint8_t *)text, text_length,
                            frame, sizeof(frame), &frame_length) != SMARTCAR_LOG_OK) {
        return ESP_FAIL;
    }
    return s3_ble_log_notify_send(frame, (uint16_t)frame_length);
}

static void s3_ble_log_flush_pending(void)
{
    for (;;) {
        s3_pending_log_t pending;

        if (!s_ble_state.connected || !s_ble_state.log_notify_enabled) {
            return;
        }
        portENTER_CRITICAL(&s_pending_log_lock);
        if (s_pending_log_count == 0U) {
            portEXIT_CRITICAL(&s_pending_log_lock);
            return;
        }
        pending = s_pending_logs[s_pending_log_tail];
        portEXIT_CRITICAL(&s_pending_log_lock);

        if (s3_ble_log_send_text(pending.level, pending.timestamp_ms,
                                 pending.text,
                                 pending.length) != ESP_OK) {
            return;
        }
        portENTER_CRITICAL(&s_pending_log_lock);
        s_pending_log_tail = (uint8_t)((s_pending_log_tail + 1U) %
                                       S3_LOG_PENDING_CAPACITY);
        --s_pending_log_count;
        portEXIT_CRITICAL(&s_pending_log_lock);
    }
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

static void update_ready_state(void)
{
    const bool ready = s_ble_state.connected && s_ble_state.notify_enabled;
    const bool became_ready = ready && !s_ble_state.ready;

    s_ble_state.ready = ready;
    if (became_ready && s_ready_callback != NULL) {
        s_ready_callback(s_ready_callback_context);
    }
}

/* GATT owns the write buffer. The receiver must synchronously copy it into
 * its queue, but callback registration itself can race BLE initialization. */
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
        s_att_mtu = 23U;
        s_conn_id = param->connect.conn_id;
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
            s_ble_state.log_notify_enabled = (ccc & 0x0001U) != 0U;
            if (s_ble_state.log_notify_enabled) {
                (void)s3_ble_log_emit(SMARTCAR_LOG_LEVEL_INFO, "BOOT");
                if (s_ble_state.connected) {
                    (void)s3_ble_log_emit(SMARTCAR_LOG_LEVEL_INFO,
                                          "BLE_CONNECTED");
                }
                s3_ble_log_flush_pending();
            }
        }
        break;
    case ESP_GATTS_MTU_EVT:
        if (param->mtu.mtu >= 23U) {
            s_att_mtu = param->mtu.mtu;
        }
        break;
    default:
        break;
    }
}

esp_err_t s3_ble_init(void)
{
    esp_err_t ret;

    if (s_initialized) {
        return ESP_OK;
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
    ret = esp_ble_gatts_app_register(0);
    if (ret != ESP_OK) {
        return ret;
    }
    s_initialized = true;
    ESP_LOGI(TAG, "BLE INIT OK");
    ESP_LOGI(TAG, "BLE DEVICE NAME: %s", S3_BLE_DEVICE_NAME);
    return ESP_OK;
}

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
            ++s_ble_notify_fail_count;
            return ret;
        }
        offset = (uint16_t)(offset + chunk_len);
    }
    return ESP_OK;
}

esp_err_t s3_ble_log_notify_send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U || len > S3_BLE_MAX_RX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized || !s_ble_state.connected ||
        !s_ble_state.log_notify_enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint16_t max_payload = s_att_mtu > 3U ? (uint16_t)(s_att_mtu - 3U) : 20U;
    uint16_t offset = 0U;
    while (offset < len) {
        uint16_t chunk_len = (uint16_t)(len - offset);
        if (chunk_len > max_payload) {
            chunk_len = max_payload;
        }
        const esp_err_t ret = esp_ble_gatts_send_indicate(s_gatts_if,
                                                            s_conn_id,
                                                            s_handles[IDX_LOG_VALUE],
                                                            chunk_len,
                                                            (uint8_t *)&data[offset],
                                                            false);
        if (ret != ESP_OK) {
            ++s_ble_notify_fail_count;
            return ret;
        }
        offset = (uint16_t)(offset + chunk_len);
    }
    return ESP_OK;
}

esp_err_t s3_ble_log_emit(smartcar_log_level_t level, const char *text)
{
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

    if (!s_initialized || !s_ble_state.connected ||
        !s_ble_state.log_notify_enabled) {
        esp_err_t result = ESP_OK;

        portENTER_CRITICAL(&s_pending_log_lock);
        if (s_pending_log_count >= S3_LOG_PENDING_CAPACITY) {
            result = ESP_ERR_NO_MEM;
        } else {
            s3_pending_log_t *pending = &s_pending_logs[s_pending_log_head];
            pending->level = level;
            pending->timestamp_ms = (uint32_t)esp_log_timestamp();
            pending->length = (uint8_t)text_length;
            memcpy(pending->text, text, text_length);
            pending->text[text_length] = '\0';
            s_pending_log_head = (uint8_t)((s_pending_log_head + 1U) %
                                           S3_LOG_PENDING_CAPACITY);
            ++s_pending_log_count;
        }
        portEXIT_CRITICAL(&s_pending_log_lock);
        return result;
    }
    return s3_ble_log_send_text(level, (uint32_t)esp_log_timestamp(),
                                text, (uint8_t)text_length);
}

esp_err_t s3_log_info(const char *text)
{
    return s3_ble_log_emit(SMARTCAR_LOG_LEVEL_INFO, text);
}

esp_err_t s3_log_warn(const char *text)
{
    return s3_ble_log_emit(SMARTCAR_LOG_LEVEL_WARN, text);
}

esp_err_t s3_log_error(const char *text)
{
    return s3_ble_log_emit(SMARTCAR_LOG_LEVEL_ERROR, text);
}

bool s3_ble_is_ready(void)
{
    return s_ble_state.ready;
}

bool s3_ble_is_log_ready(void)
{
    return s_initialized && s_ble_state.connected &&
           s_ble_state.log_notify_enabled;
}

uint32_t s3_ble_get_notify_fail_count(void)
{
    return s_ble_notify_fail_count;
}

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

esp_err_t s3_ble_set_disconnect_callback(s3_ble_disconnect_callback_t callback,
                                         void *context)
{
    s_disconnect_callback = callback;
    s_disconnect_callback_context = context;
    return ESP_OK;
}

esp_err_t s3_ble_send(const uint8_t *data, uint16_t len)
{
    return s3_ble_notify_send(data, len);
}

esp_err_t s3_ble_register_rx_callback(s3_ble_rx_callback_t callback,
                                      void *context)
{
    portENTER_CRITICAL(&s_rx_callback_lock);
    s_rx_callback = callback;
    s_rx_callback_context = context;
    portEXIT_CRITICAL(&s_rx_callback_lock);
    return ESP_OK;
}

esp_err_t s3_ble_set_rx_callback(s3_ble_rx_callback_t callback, void *context)
{
    return s3_ble_register_rx_callback(callback, context);
}
