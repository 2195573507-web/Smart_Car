#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "radar/radar_uart.h"
#include "radar_control.h"
#include "s3_ble.h"
#include "smartcar_service.h"
#include "stm_uart.h"

static const char *TAG = "MAIN";

#ifndef SMARTCAR_BMI323_DEBUG_ONLY
#define SMARTCAR_BMI323_DEBUG_ONLY 0
#endif

static esp_err_t main_nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ret = nvs_flash_erase();
        if (ret == ESP_OK) {
            ret = nvs_flash_init();
        }
    }
    return ret;
}

void app_main(void)
{
    esp_err_t nvs_ret = main_nvs_init();
    if (nvs_ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(nvs_ret));
        return;
    }

    esp_err_t stm_uart_ret = stm_uart_init();
    if (stm_uart_ret != ESP_OK) {
        ESP_LOGE(TAG, "STM UART2 init failed: %s", esp_err_to_name(stm_uart_ret));
    }

    esp_err_t ble_ret = s3_ble_init();
    if (ble_ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE init failed: %s", esp_err_to_name(ble_ret));
    }

#if !SMARTCAR_BMI323_DEBUG_ONLY
    s3_log_info("RADAR INIT START");
    bool radar_init_ok = true;
    esp_err_t radar_uart_ret = radar_uart_init();
    if (radar_uart_ret != ESP_OK) {
        radar_init_ok = false;
        ESP_LOGE(TAG, "Radar UART1 init failed: %s", esp_err_to_name(radar_uart_ret));
    }
    esp_err_t radar_pwm_ret = radar_pwm_init();
    if (radar_pwm_ret != ESP_OK) {
        radar_init_ok = false;
        ESP_LOGE(TAG, "Radar PWM GPIO4 init failed: %s", esp_err_to_name(radar_pwm_ret));
    } else {
        radar_control_init();
    }
    if (radar_init_ok) {
        s3_log_info("RADAR INIT OK");
    } else {
        s3_log_error("RADAR INIT FAILED");
    }
#endif

    if (stm_uart_ret == ESP_OK) {
        esp_err_t service_ret = smartcar_service_init();
        if (service_ret != ESP_OK) {
            ESP_LOGE(TAG, "SmartCar service init failed: %s", esp_err_to_name(service_ret));
        }
    }

#if SMARTCAR_BMI323_DEBUG_ONLY
    s3_log_info("BMI323 DEBUG LOG FORWARDING");
#else
    s3_log_info("S3 SYSTEM READY");
#endif

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}
