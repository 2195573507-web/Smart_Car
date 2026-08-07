#include "stm_uart.h"

#include <stdbool.h>
#include <string.h>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define STM_UART_RX_DRIVER_BUFFER_SIZE 4096U
#define STM_UART_TX_DRIVER_BUFFER_SIZE 4096U
#define STM_UART_TASK_STACK_SIZE 3072U
#define STM_UART_TASK_PRIORITY 9U
#define STM_UART_TASK_READ_SIZE 256U
#define STM_UART_TASK_READ_TIMEOUT_MS 100U
#define STM_UART_STORAGE_SIZE 4096U
#define STM_UART_RECEIVE_TIMEOUT_MS 10U

static TaskHandle_t s_task;
static SemaphoreHandle_t s_storage_mutex;
static SemaphoreHandle_t s_tx_mutex;
static bool s_initialized;
static uint8_t s_storage[STM_UART_STORAGE_SIZE];
static size_t s_storage_head;
static size_t s_storage_tail;
static size_t s_storage_count;
static stm_uart_stats_t s_stats;

static void stm_uart_store(const uint8_t *data, size_t len)
{
    if (xSemaphoreTake(s_storage_mutex, pdMS_TO_TICKS(10U)) != pdTRUE) {
        s_stats.drop += (uint32_t)len;
        return;
    }

    /* Keep the newest bytes when the application is slower than the UART. */
    if (len >= STM_UART_STORAGE_SIZE) {
        s_stats.overflow++;
        s_stats.drop += (uint32_t)(s_storage_count + len - STM_UART_STORAGE_SIZE);
        data += len - STM_UART_STORAGE_SIZE;
        len = STM_UART_STORAGE_SIZE;
        s_storage_head = 0U;
        s_storage_tail = 0U;
        s_storage_count = 0U;
    }
    while (s_storage_count + len > STM_UART_STORAGE_SIZE) {
        ++s_stats.overflow;
        ++s_stats.drop;
        s_storage_tail = (s_storage_tail + 1U) % STM_UART_STORAGE_SIZE;
        --s_storage_count;
    }

    for (size_t index = 0U; index < len; ++index) {
        s_storage[s_storage_head] = data[index];
        s_storage_head = (s_storage_head + 1U) % STM_UART_STORAGE_SIZE;
    }
    s_storage_count += len;
    (void)xSemaphoreGive(s_storage_mutex);
}

static void stm_uart_task(void *context)
{
    (void)context;
    uint8_t read_buffer[STM_UART_TASK_READ_SIZE];

    for (;;) {
        int received = uart_read_bytes(STM_UART_PORT,
                                       read_buffer,
                                       sizeof(read_buffer),
                                       pdMS_TO_TICKS(STM_UART_TASK_READ_TIMEOUT_MS));
        if (received > 0) {
            if (s_storage_mutex != NULL) {
                if (xSemaphoreTake(s_storage_mutex, pdMS_TO_TICKS(10U)) == pdTRUE) {
                    s_stats.rx_bytes += (uint32_t)received;
                    (void)xSemaphoreGive(s_storage_mutex);
                } else {
                    ++s_stats.drop;
                }
            }
            stm_uart_store(read_buffer, (size_t)received);
        } else if (received < 0) {
            if (s_storage_mutex != NULL &&
                xSemaphoreTake(s_storage_mutex, pdMS_TO_TICKS(10U)) == pdTRUE) {
                ++s_stats.hal_error;
                (void)xSemaphoreGive(s_storage_mutex);
            }
            vTaskDelay(pdMS_TO_TICKS(10U));
        }
    }
}

esp_err_t stm_uart_init(void)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const uart_config_t config = {
        .baud_rate = STM_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(STM_UART_PORT, &config);
    if (ret == ESP_OK) {
        ret = uart_set_pin(STM_UART_PORT,
                           STM_UART_TX_GPIO,
                           STM_UART_RX_GPIO,
                           UART_PIN_NO_CHANGE,
                           UART_PIN_NO_CHANGE);
    }
    if (ret == ESP_OK) {
        ret = uart_driver_install(STM_UART_PORT,
                                  STM_UART_RX_DRIVER_BUFFER_SIZE,
                                  STM_UART_TX_DRIVER_BUFFER_SIZE,
                                  0,
                                  NULL,
                                  0);
    }
    if (ret != ESP_OK) {
        return ret;
    }

    s_storage_mutex = xSemaphoreCreateMutex();
    s_tx_mutex = xSemaphoreCreateMutex();
    if (s_storage_mutex == NULL || s_tx_mutex == NULL) {
        if (s_storage_mutex != NULL) {
            vSemaphoreDelete(s_storage_mutex);
            s_storage_mutex = NULL;
        }
        if (s_tx_mutex != NULL) {
            vSemaphoreDelete(s_tx_mutex);
            s_tx_mutex = NULL;
        }
        (void)uart_driver_delete(STM_UART_PORT);
        return ESP_ERR_NO_MEM;
    }

    s_storage_head = 0U;
    s_storage_tail = 0U;
    s_storage_count = 0U;
    (void)memset(&s_stats, 0, sizeof(s_stats));
    BaseType_t created = xTaskCreate(stm_uart_task,
                                     "stm_uart_task",
                                     STM_UART_TASK_STACK_SIZE,
                                     NULL,
                                     STM_UART_TASK_PRIORITY,
                                     &s_task);
    if (created != pdPASS) {
        vSemaphoreDelete(s_storage_mutex);
        vSemaphoreDelete(s_tx_mutex);
        s_storage_mutex = NULL;
        s_tx_mutex = NULL;
        (void)uart_driver_delete(STM_UART_PORT);
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    return ESP_OK;
}

int stm_uart_send(const uint8_t *data, size_t len)
{
    int written;
    esp_err_t wait_status;

    if (!s_initialized || data == NULL || len == 0U || s_tx_mutex == NULL) {
        return -1;
    }
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(STM_UART_TX_TIMEOUT_MS)) != pdTRUE) {
        if (s_storage_mutex != NULL &&
            xSemaphoreTake(s_storage_mutex, pdMS_TO_TICKS(10U)) == pdTRUE) {
            ++s_stats.hal_error;
            (void)xSemaphoreGive(s_storage_mutex);
        }
        return -1;
    }
    written = uart_write_bytes(STM_UART_PORT, data, len);
    if (s_storage_mutex != NULL &&
        xSemaphoreTake(s_storage_mutex, pdMS_TO_TICKS(10U)) == pdTRUE) {
        if (written > 0) {
            s_stats.tx_bytes += (uint32_t)written;
        }
        if (written != (int)len) {
            ++s_stats.short_write;
            if (written < (int)len) {
                s_stats.drop += (uint32_t)(len - (written > 0 ? written : 0));
            }
        }
        (void)xSemaphoreGive(s_storage_mutex);
    }
    wait_status = written == (int)len
                      ? uart_wait_tx_done(STM_UART_PORT,
                                          pdMS_TO_TICKS(STM_UART_TX_TIMEOUT_MS))
                      : ESP_FAIL;
    (void)xSemaphoreGive(s_tx_mutex);
    if (wait_status != ESP_OK) {
        if (s_storage_mutex != NULL &&
            xSemaphoreTake(s_storage_mutex, pdMS_TO_TICKS(10U)) == pdTRUE) {
            ++s_stats.hal_error;
            (void)xSemaphoreGive(s_storage_mutex);
        }
        return -1;
    }
    return written;
}

void stm_uart_get_stats(stm_uart_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    if (s_storage_mutex == NULL ||
        xSemaphoreTake(s_storage_mutex, pdMS_TO_TICKS(10U)) != pdTRUE) {
        (void)memset(stats, 0, sizeof(*stats));
        return;
    }
    *stats = s_stats;
    (void)xSemaphoreGive(s_storage_mutex);
}

static int stm_uart_receive_with_timeout(uint8_t *buffer,
                                         size_t max_len,
                                         TickType_t timeout_ticks)
{
    if (!s_initialized || buffer == NULL || max_len == 0U) {
        return -1;
    }
    if (xSemaphoreTake(s_storage_mutex, timeout_ticks) != pdTRUE) {
        return 0;
    }

    size_t length = s_storage_count < max_len ? s_storage_count : max_len;
    for (size_t index = 0U; index < length; ++index) {
        buffer[index] = s_storage[s_storage_tail];
        s_storage_tail = (s_storage_tail + 1U) % STM_UART_STORAGE_SIZE;
    }
    s_storage_count -= length;
    xSemaphoreGive(s_storage_mutex);
    return (int)length;
}

int stm_uart_receive_nonblock(uint8_t *buffer, size_t max_len)
{
    return stm_uart_receive_with_timeout(buffer, max_len, 0U);
}

int stm_uart_receive(uint8_t *buffer, size_t max_len)
{
    return stm_uart_receive_with_timeout(buffer, max_len,
                                         pdMS_TO_TICKS(STM_UART_RECEIVE_TIMEOUT_MS));
}
