#ifndef S3_RADAR_UART_H
/* 防止雷达 UART 头文件被重复包含。 */
#define S3_RADAR_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

/* 雷达使用 UART1 控制器。 */
#define RADAR_UART_PORT UART_NUM_1
/* X3PRO is receive-only here; its TX output is connected to S3 GPIO44. */
#define RADAR_UART_RX_GPIO GPIO_NUM_44
/* X3PRO UART 通信波特率。 */
#define RADAR_UART_BAUD_RATE 115200
/* UART 驱动接收缓冲区大小，单位为字节。 */
#define RADAR_UART_DRIVER_BUFFER_SIZE 4096U
/* 单次 UART 读取缓冲区大小，单位为字节。 */
#define RADAR_UART_READ_BUFFER_SIZE 512U
/* UART 读取超时时间，单位为毫秒。 */
#define RADAR_UART_READ_TIMEOUT_MS 100U
/* 雷达 UART 接收任务的栈大小，单位为字节。 */
#define RADAR_UART_TASK_STACK_SIZE 4096U
/* 雷达 UART 接收任务优先级。 */
#define RADAR_UART_TASK_PRIORITY 10U

/* GPIO44 电平监测任务参数。 */
#define RADAR_GPIO_MONITOR_TASK_STACK_SIZE 2048U
#define RADAR_GPIO_MONITOR_TASK_PRIORITY 5U
#define RADAR_GPIO_MONITOR_INTERVAL_MS 1000U

/* X3PRO M_CTR 电机 PWM 输出引脚。 */
#define RADAR_PWM_GPIO GPIO_NUM_4
/* 电机 PWM 频率，单位为 Hz。 */
#define RADAR_PWM_FREQUENCY_HZ 10000U
/* LEDC 占空比分辨率的位数。 */
#define RADAR_PWM_DUTY_RESOLUTION_BITS 10U
/* LEDC 占空比计数的最大值。 */
#define RADAR_PWM_DUTY_MAX ((1U << RADAR_PWM_DUTY_RESOLUTION_BITS) - 1U)
/* 占空比百分数的换算基数。 */
#define RADAR_PWM_DUTY_PERCENT_BASE 100U
/* 电机 PWM 目标占空比，单位为百分比。 */
#define RADAR_PWM_DUTY_PERCENT 0U
/* 按目标百分比换算后的 LEDC 占空比计数值。 */
#define RADAR_PWM_DUTY \
    ((RADAR_PWM_DUTY_MAX * RADAR_PWM_DUTY_PERCENT) / RADAR_PWM_DUTY_PERCENT_BASE)

/* Starts the UART driver and its continuous receive/parser task. */
esp_err_t radar_uart_init(void);
bool radar_uart_is_running(void);

/* Configures GPIO44 as the radar RX input and starts its level monitor. */
esp_err_t radar_gpio_monitor_init(void);

/* Starts the X3PRO M_CTR motor output at the required duty and frequency. */
esp_err_t radar_pwm_init(void);
bool radar_pwm_is_running(void);

#endif
