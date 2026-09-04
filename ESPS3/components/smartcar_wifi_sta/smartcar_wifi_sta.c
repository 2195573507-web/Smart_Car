#include "smartcar_wifi_sta.h"

#include <string.h>

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"

#ifndef CONFIG_SMARTCAR_WIFI_STA_SSID
#define CONFIG_SMARTCAR_WIFI_STA_SSID ""
#endif
#ifndef CONFIG_SMARTCAR_WIFI_STA_PASSWORD
#define CONFIG_SMARTCAR_WIFI_STA_PASSWORD ""
#endif

#define SMARTCAR_WIFI_STA_CONNECTED_BIT BIT0

static EventGroupHandle_t s_events;
static bool s_initialized;

static void smartcar_wifi_sta_event(void *arg, esp_event_base_t event_base,
                                    int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_events, SMARTCAR_WIFI_STA_CONNECTED_BIT);
        (void)esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_events, SMARTCAR_WIFI_STA_CONNECTED_BIT);
    }
}

static bool smartcar_wifi_sta_credentials_valid(void)
{
    return CONFIG_SMARTCAR_WIFI_STA_SSID[0] != '\0' &&
           CONFIG_SMARTCAR_WIFI_STA_PASSWORD[0] != '\0' &&
           strlen(CONFIG_SMARTCAR_WIFI_STA_SSID) <=
               sizeof(((wifi_config_t *)0)->sta.ssid) &&
           strlen(CONFIG_SMARTCAR_WIFI_STA_PASSWORD) <=
               sizeof(((wifi_config_t *)0)->sta.password);
}

esp_err_t smartcar_wifi_sta_start(void)
{
    wifi_config_t config = {0};
    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret;

    if (s_initialized) {
        return ESP_OK;
    }
    if (!smartcar_wifi_sta_credentials_valid()) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    if (esp_netif_get_handle_from_ifkey("WIFI_STA_DEF") == NULL &&
        esp_netif_create_default_wifi_sta() == NULL) {
        return ESP_ERR_NO_MEM;
    }
    ret = esp_wifi_init(&init_config);
    if (ret != ESP_OK) {
        return ret;
    }
    s_events = xEventGroupCreate();
    if (s_events == NULL) {
        return ESP_ERR_NO_MEM;
    }
    ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                     smartcar_wifi_sta_event, NULL);
    if (ret == ESP_OK) {
        ret = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                         smartcar_wifi_sta_event, NULL);
    }
    if (ret == ESP_OK) {
        ret = esp_wifi_set_mode(WIFI_MODE_STA);
    }
    if (ret == ESP_OK) {
        (void)strncpy((char *)config.sta.ssid, CONFIG_SMARTCAR_WIFI_STA_SSID,
                      sizeof(config.sta.ssid) - 1U);
        (void)strncpy((char *)config.sta.password,
                      CONFIG_SMARTCAR_WIFI_STA_PASSWORD,
                      sizeof(config.sta.password) - 1U);
        ret = esp_wifi_set_config(WIFI_IF_STA, &config);
    }
    if (ret == ESP_OK) {
        ret = esp_wifi_start();
    }
    if (ret != ESP_OK) {
        return ret;
    }
    s_initialized = true;
    ret = esp_wifi_connect();
    return ret == ESP_OK || ret == ESP_ERR_WIFI_CONN ? ESP_OK : ret;
}

bool smartcar_wifi_sta_is_connected(void)
{
    return s_initialized && s_events != NULL &&
           (xEventGroupGetBits(s_events) & SMARTCAR_WIFI_STA_CONNECTED_BIT) != 0U;
}
