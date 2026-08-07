#include "radar_uart.h"

#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "RADAR";

#define RADAR_UART_HEX_LOG_BYTES 32U

static TaskHandle_t s_uart_task;
static TaskHandle_t s_gpio_monitor_task;
static bool s_uart_ready;
static bool s_gpio_monitor_ready;
static bool s_pwm_ready;
static uint8_t s_read_buffer[RADAR_UART_READ_BUFFER_SIZE];
static char s_hex_buffer[RADAR_UART_HEX_LOG_BYTES * 3U];

static void radar_uart_log_hex(const uint8_t *data, size_t length)
{
    static const char hex_digits[] = "0123456789ABCDEF";
    size_t bytes_to_log = length;
    if (bytes_to_log > RADAR_UART_HEX_LOG_BYTES) {
        bytes_to_log = RADAR_UART_HEX_LOG_BYTES;
    }

    for (size_t index = 0U; index < bytes_to_log; ++index) {
        const uint8_t value = data[index];
        s_hex_buffer[index * 3U] = hex_digits[value >> 4U];
        s_hex_buffer[index * 3U + 1U] = hex_digits[value & 0x0FU];
        s_hex_buffer[index * 3U + 2U] = ' ';
    }
    if (bytes_to_log > 0U) {
        s_hex_buffer[bytes_to_log * 3U - 1U] = '\0';
    } else {
        s_hex_buffer[0] = '\0';
    }

    ESP_LOGI(TAG, "RADAR_HEX:");
    ESP_LOGI(TAG, "%s", s_hex_buffer);
}

static void radar_uart_task(void *context)
{
    (void)context;

    for (;;) {
        int received = uart_read_bytes(RADAR_UART_PORT,
                                       s_read_buffer,
                                       sizeof(s_read_buffer),
                                       pdMS_TO_TICKS(RADAR_UART_READ_TIMEOUT_MS));
        ESP_LOGI(TAG, "RADAR_UART_RX len=%d", received);
        if (received > 0) {
            size_t length = (size_t)received;
            radar_uart_log_hex(s_read_buffer, length);
        } else if (received < 0) {
            ESP_LOGE(TAG, "UART receive failed: %d", received);
            vTaskDelay(pdMS_TO_TICKS(10U));
        }
    }
}

static void radar_gpio_monitor_task(void *context)
{
    (void)context;

    for (;;) {
        ESP_LOGI(TAG, "RADAR_GPIO44_LEVEL=%d", gpio_get_level(GPIO_NUM_44));
        vTaskDelay(pdMS_TO_TICKS(RADAR_GPIO_MONITOR_INTERVAL_MS));
    }
}

esp_err_t radar_gpio_monitor_init(void)
{
    if (s_gpio_monitor_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << GPIO_NUM_44,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&config);
    if (ret != ESP_OK) {
        return ret;
    }

    BaseType_t created = xTaskCreate(radar_gpio_monitor_task,
                                     "radar_gpio_monitor",
                                     RADAR_GPIO_MONITOR_TASK_STACK_SIZE,
                                     NULL,
                                     RADAR_GPIO_MONITOR_TASK_PRIORITY,
                                     &s_gpio_monitor_task);
    if (created != pdPASS) {
        s_gpio_monitor_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_gpio_monitor_ready = true;
    return ESP_OK;
}

esp_err_t radar_uart_init(void)
{
    if (s_uart_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    const uart_config_t config = {
        .baud_rate = RADAR_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(RADAR_UART_PORT, &config);
    if (ret == ESP_OK) {
        ret = uart_set_pin(RADAR_UART_PORT,
                           RADAR_UART_TX_GPIO,
                           RADAR_UART_RX_GPIO,
                           UART_PIN_NO_CHANGE,
                           UART_PIN_NO_CHANGE);
    }
    if (ret == ESP_OK) {
        ret = uart_driver_install(RADAR_UART_PORT,
                                  RADAR_UART_DRIVER_BUFFER_SIZE,
                                  0,
                                  0,
                                  NULL,
                                  0);
    }
    if (ret != ESP_OK) {
        return ret;
    }

    BaseType_t created = xTaskCreate(radar_uart_task,
                                     "radar_uart",
                                     RADAR_UART_TASK_STACK_SIZE,
                                     NULL,
                                     RADAR_UART_TASK_PRIORITY,
                                     &s_uart_task);
    if (created != pdPASS) {
        (void)uart_driver_delete(RADAR_UART_PORT);
        s_uart_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_uart_ready = true;
    ESP_LOGI(TAG,
             "UART%u ready TX=GPIO%u RX=GPIO%u baud=%u rx_buffer=%u",
             (unsigned int)RADAR_UART_PORT,
             (unsigned int)RADAR_UART_TX_GPIO,
             (unsigned int)RADAR_UART_RX_GPIO,
             (unsigned int)RADAR_UART_BAUD_RATE,
             (unsigned int)RADAR_UART_DRIVER_BUFFER_SIZE);
    return ESP_OK;
}

bool radar_uart_is_running(void)
{
    return s_uart_ready && s_uart_task != NULL;
}

static esp_err_t radar_pwm_test_set_duty(uint32_t duty)
{
    ESP_LOGI(TAG, "[PWM_TEST] duty=%u", (unsigned int)duty);

    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ESP_LOGI(TAG,
             "[PWM_TEST] ledc_set_duty ret=%s (%d)",
             esp_err_to_name(ret),
             (int)ret);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ESP_LOGI(TAG,
             "[PWM_TEST] ledc_update_duty ret=%s (%d)",
             esp_err_to_name(ret),
             (int)ret);
    return ret;
}

esp_err_t radar_pwm_init(void)
{
    if (s_pwm_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = RADAR_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        return ret;
    }

    const ledc_channel_config_t channel_config = {
        .gpio_num = RADAR_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = {0},
    };
    ret = ledc_channel_config(&channel_config);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, RADAR_PWM_DUTY);
    if (ret == ESP_OK) {
        ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
    if (ret != ESP_OK) {
        return ret;
    }

    s_pwm_ready = true;
    ESP_LOGI(TAG,
             "M_CTR PWM ready GPIO=%u frequency=%uHz duty=%u%%",
             (unsigned int)RADAR_PWM_GPIO,
             (unsigned int)RADAR_PWM_FREQUENCY_HZ,
             (unsigned int)RADAR_PWM_DUTY_PERCENT);
    return ESP_OK;
}

esp_err_t radar_pwm_test_sequence(void)
{
    static const uint32_t test_duties[] = {
        RADAR_PWM_TEST_DUTY_LOW,
        RADAR_PWM_TEST_DUTY_MID,
        RADAR_PWM_TEST_DUTY_HIGH,
    };

    if (!s_pwm_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    for (size_t index = 0U; index < sizeof(test_duties) / sizeof(test_duties[0]); ++index) {
        esp_err_t ret = radar_pwm_test_set_duty(test_duties[index]);
        if (ret != ESP_OK) {
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(RADAR_PWM_TEST_HOLD_MS));
    }

    return ESP_OK;
}

bool radar_pwm_is_running(void)
{
    return s_pwm_ready;
}
