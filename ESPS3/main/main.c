#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "radar/radar_uart.h"
#include "radar/radar_uplink.h"
#include "radar_control.h"
#include "s3_ble.h"
#include "smartcar_service.h"
#include "smartcar_debug_config.h"
#include "stm_uart.h"

static const char *TAG = "MAIN";

/**
 * @brief  初始化 NVS，并在页耗尽或版本不兼容时擦除分区后重试一次。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return ESP_OK 表示 NVS 可用；擦除、首次初始化或重试失败时返回对应 esp_err_t。
 * 调用方式：仅由 app_main() 在任何依赖 NVS 的无线服务之前调用；失败后终止本次应用启动。
 * 线程约束：运行于 ESP-IDF 启动任务，可能执行 Flash 擦除并阻塞；禁止 ISR 调用或并发初始化。
 * 数据影响：兼容性错误路径会擦除整个 NVS 分区，历史配置信息不会保留。
 */
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

/**
 * @brief  按启动依赖顺序初始化 S3 通信、雷达和可选上行服务并保持启动任务存活。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 无；NVS 失败时直接返回，正常路径进入永久延时循环而不返回。
 * 调用方式：由 ESP-IDF 启动框架自动调用一次；按 NVS -> STM UART -> BLE ->
 *           雷达/PWM -> 可选上行 -> 网关服务顺序执行。除 NVS 外的子模块失败会记录日志并继续
 *           初始化独立模块；smartcar_service 仅在 STM UART 成功时启动。
 * 线程约束：运行于系统启动任务，会创建驱动、队列、互斥量和 FreeRTOS 任务并长期阻塞；
 *           禁止 ISR 调用、其他任务重复调用或把初始化日志解释为 UART/BLE/雷达硬件验收。
 */
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

    esp_err_t radar_uplink_ret = radar_uplink_init();
    if (radar_uplink_ret != ESP_OK) {
        ESP_LOGE(TAG, "Radar uplink init failed: %s",
                 esp_err_to_name(radar_uplink_ret));
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
