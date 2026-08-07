#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "radar/radar_uart.h"

static const char *TAG = "RADAR";

void app_main(void)
{
    esp_err_t pwm_ret = radar_pwm_init();
    if (pwm_ret != ESP_OK) {
        ESP_LOGE(TAG, "radar PWM init failed: %s", esp_err_to_name(pwm_ret));
    } else {
        esp_err_t pwm_test_ret = radar_pwm_test_sequence();
        if (pwm_test_ret != ESP_OK) {
            ESP_LOGE(TAG, "PWM test failed: %s", esp_err_to_name(pwm_test_ret));
        }
    }

    vTaskDelay(pdMS_TO_TICKS(3000U));

    esp_err_t monitor_ret = radar_gpio_monitor_init();
    if (monitor_ret != ESP_OK) {
        ESP_LOGE(TAG, "radar GPIO monitor init failed: %s", esp_err_to_name(monitor_ret));
    }

    esp_err_t uart_ret = radar_uart_init();
    if (uart_ret != ESP_OK) {
        ESP_LOGE(TAG, "radar UART init failed: %s", esp_err_to_name(uart_ret));
    }

    for (;;) {
        ESP_LOGI(TAG,
                 "RADAR_SYSTEM running uart=%s pwm=%s",
                 radar_uart_is_running() ? "yes" : "no",
                 radar_pwm_is_running() ? "yes" : "no");
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}
