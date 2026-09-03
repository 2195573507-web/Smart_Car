#ifndef S3_RADAR_UART_H
/* 防止雷达 UART 头文件被重复包含。 */
#define S3_RADAR_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/task.h"
#include "radar_parser.h"

/*
 * 雷达 UART1/GPIO44 与 PWM GPIO4 适配层。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * UART1 只接收雷达；STM32 链路使用独立 UART2，两个缓冲和任务不能混用。
 */

/* 雷达使用 UART1 控制器。 */
#define RADAR_UART_PORT UART_NUM_1
/* X3PRO 在此处为只接收；其 TX 输出连接到 S3 GPIO44。 */
#define RADAR_UART_RX_GPIO GPIO_NUM_44
/* X3PRO UART 通信波特率。 */
#define RADAR_UART_BAUD_RATE 115200
/* UART 驱动接收缓冲区大小，单位为字节。 */
#define RADAR_UART_DRIVER_BUFFER_SIZE 4096U
/* 单次 UART 读取缓冲区大小，单位为字节。 */
#define RADAR_UART_READ_BUFFER_SIZE 512U
/* UART 驱动事件队列深度。 */
#define RADAR_UART_EVENT_QUEUE_SIZE 32U
/* RX FIFO 达到该字节数时唤醒接收任务。 */
#define RADAR_UART_RX_FULL_THRESHOLD 120U
/* RX 线空闲达到该符号数时唤醒接收任务。 */
#define RADAR_UART_RX_TIMEOUT_SYMBOLS 10U
/* 空闲时的事件队列等待上限，单位为毫秒。 */
#define RADAR_UART_EVENT_WAIT_MS 100U
/* 雷达 UART 接收任务的栈大小，单位为字节。 */
#define RADAR_UART_TASK_STACK_SIZE 4096U
/* 雷达 UART 接收任务优先级。 */
#define RADAR_UART_TASK_PRIORITY 10U

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

/**
 * @brief 启动 UART1 驱动和持续接收/解析任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return ESP_OK 表示 PSRAM FIFO、mutex、UART1 驱动和接收任务均创建成功；重复调用返回
 *         ESP_ERR_INVALID_STATE，内存/驱动/任务失败返回对应 ESP-IDF 错误。
 * 调用方式：app_main 中先于 radar_uplink_init() 调用；该函数与 radar_pwm_init() 相互独立。
 *           失败时不得启动依赖原始帧的上行服务，也不能据日志宣称雷达在线。
 * 线程约束：只允许系统启动任务调用一次；会分配约 RADAR_FRAME_FIFO_DEPTH 条完整帧的
 *           PSRAM、创建 FreeRTOS 资源和任务，禁止 ISR/多任务并发调用。
 * 硬件边界：只配置 UART1 RX=GPIO44、115200 8N1，不配置 TX，也不接触 STM UART2。
 */
esp_err_t radar_uart_init(void);
/**
 * @brief  查询雷达 UART 初始化标志及接收任务句柄是否有效。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 两项均有效时为 true；不表示 UART 有字节、解析出有效帧或雷达物理在线。
 * 调用方式：启动状态/诊断读取；有效帧健康应结合 parser/FIFO 统计和时间戳判断。
 * 线程约束：无锁快照、不阻塞；只在启动完成后读取。
 */
bool radar_uart_is_running(void);

/**
 * @brief  设置完整帧成功入队后通过 task notification 唤醒的消费者任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  task FreeRTOS 任务句柄；非 NULL 时保存并立即发送一次通知，NULL 表示取消。
 * @return 无。
 * 调用方式：radar_uart_init() 成功且上行任务创建后注册；任务删除前先传 NULL，避免悬空句柄。
 * 线程约束：设计为启动/上行任务串行调用，无内部锁；禁止 ISR 调用或与任务删除并发。
 */
void radar_uart_set_frame_notification_task(TaskHandle_t task);

/**
 * @brief 取出最旧的校验通过原始帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[out] buffer 非 NULL、至少可写 capacity 字节的输出空间。
 * @param  capacity buffer 容量，必须大于 0。
 * @param[out] length 必须非 NULL；成功时为帧长，空队列时为 0，短缓冲时为所需长度；
 *                    参数错误或 mutex 获取失败时保持调用前值。
 * @param[out] sequence 可为 NULL；成功时写入 UART 接收侧递增序号。
 * @param[out] timestamp_ms 可为 NULL；成功时写入 esp_log_timestamp() 采集时间。
 * @param[out] age_ms 可为 NULL；成功时写入当前时间减采集时间，单位 ms，允许无符号回绕。
 * @return true 表示已复制并消费队首；参数错误、未初始化、mutex 超时、空队列或短缓冲返回 false。
 * 调用方式：由单一上行任务调用；false 且 length 变大时扩容重试，其他 false 不应忙等。
 * 线程约束：最多等待 FIFO mutex 1 个 RTOS tick，并复制完整帧；禁止 ISR 调用。
 */
bool radar_uart_pop_frame(uint8_t *buffer,
                          size_t capacity,
                          size_t *length,
                          uint32_t *sequence,
                          uint32_t *timestamp_ms,
                          uint32_t *age_ms);

/**
 * @brief 初始化 X3PRO M_CTR PWM 输出。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return ESP_OK 表示 LEDC timer/channel 已配置并提交默认 0% 占空比；重复调用返回
 *         ESP_ERR_INVALID_STATE，其他值为 LEDC 配置/更新失败。
 * 调用方式：app_main 启动阶段调用；成功后再调用 radar_control_init()，运行期占空比只由
 *           radar_control 状态机修改。本函数不接触 STM32 姿态或车辆运动输出。
 * 线程约束：只允许启动任务调用一次；会配置 LEDC/GPIO4，禁止 ISR/并发调用。
 */
esp_err_t radar_pwm_init(void);
/**
 * @brief  查询 LEDC 雷达 PWM 是否完成初始化。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return LEDC 配置和默认 duty 更新成功后为 true；该值不表示当前占空比非零或电机已旋转。
 * 调用方式：只用于初始化状态诊断；实际门控/占空比读取使用 radar_control 接口。
 * 线程约束：无锁快照、不阻塞；启动完成后读取。
 */
bool radar_pwm_is_running(void);

#endif
