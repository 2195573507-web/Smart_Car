#ifndef STM_UART_H
#define STM_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

/*
 * S3 -> STM32H757 UART2 传输适配层。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 说明：本模块只负责 UART 驱动、有限缓冲和收发统计；SRP 编解码、命令
 *       仲裁和安全状态由上层 smartcar_service/协议层负责。
 * 硬件边界：S3 TX GPIO17 -> STM RX，S3 RX GPIO18 <- STM TX，默认 921600 8N1。
 */
#define STM_UART_PORT UART_NUM_2
#define STM_UART_TX_GPIO GPIO_NUM_17
#define STM_UART_RX_GPIO GPIO_NUM_18
#define STM_UART_BAUD_RATE 921600
#define STM_UART_RX_DRIVER_BUFFER_SIZE 8192U
#define STM_UART_TX_DRIVER_BUFFER_SIZE 2048U
#define STM_UART_BREAK_RECOVERY_THRESHOLD 20U

/** UART2 传输层诊断快照；累计字段为尽力统计，不代表 SRP 或远端业务已成功。 */
typedef struct {
    uint32_t rx_bytes; /**< UART driver 成功读出的累计字节数；后续仍可能在软件 ring 丢弃。 */
    uint32_t tx_bytes; /**< uart_write_bytes() 接受的累计字节数；包含最终 TX 等待失败前的字节。 */
    uint32_t overflow; /**< driver 溢出事件与软件 ring 覆盖动作累计次数，不是丢字节数。 */
    uint32_t drop; /**< RX/TX 路径已知丢弃的累计字节数；32 位自然回绕。 */
    uint32_t short_write; /**< UART 写入字节数不等于请求长度的累计次数。 */
    uint32_t hal_error; /**< UART 事件、driver/等待/锁失败的综合累计次数。 */
    uint32_t tx_queue_drop; /**< 历史异步 TX 队列字段；当前阻塞发送实现不更新，保持 0。 */
    uint32_t sync_guard_drop; /**< SRP 同步门拒绝的运动帧数量，不是字节数。 */
    uint32_t rx_task_reads; /**< RX 任务收到正长度 driver 读取结果的累计次数。 */
    uint32_t tx_write_errors; /**< 短写或 TX 完成等待失败的累计发送次数。 */
    uint32_t rx_error_events; /**< FIFO/缓冲溢出、帧/校验/BREAK 驱动事件累计次数。 */
    uint32_t break_events; /**< UART_BREAK 与 UART_DATA_BREAK 事件累计次数。 */
    uint32_t break_recoveries; /**< 达到连续 BREAK 阈值并置恢复请求的累计次数，不表示恢复已执行。 */
    uint16_t rx_buffered; /**< 读取快照时软件 RX ring 当前字节数，超过 UINT16_MAX 时饱和。 */
    uint16_t tx_queue_pending; /**< 历史异步 TX 深度；当前阻塞发送实现固定为 0。 */
} stm_uart_stats_t;

/**
 * @brief 初始化 UART2、驱动事件队列、接收任务和软件环形存储。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：ESP_OK 表示首次初始化完成；已经初始化时返回 ESP_ERR_INVALID_STATE；
 *          其他值表示驱动、互斥量、队列或任务创建失败。
 * 调用方式：在 app_main 中、任何收发 API 之前调用一次；失败时不得继续发送控制帧。
 * 线程约束：非幂等，只能由启动任务调用一次；禁止从 ISR 调用。
 */
esp_err_t stm_uart_init(void);

/**
 * @brief 将一条完整编码的 SRP 帧交给 UART TX 驱动。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：data 为只读帧起始地址，len 为完整帧字节数；函数不会接管所有权。
 * 返回值：非负值为实际写入字节数，负值表示未写入或参数/驱动错误。
 * 调用方式：仅任务上下文；调用方负责帧边界、重试和同步状态。
 * 线程约束：内部串行化 TX；不要在持有 UART 回调相关锁时递归调用。
 */
int stm_uart_send(const uint8_t *data, size_t len);

/**
 * @brief 复制当前 UART 统计快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：stats 为可写输出；允许为 NULL，此时函数直接返回。
 * 返回值：无；无法取得内部 mutex 时将非 NULL 输出清零，避免返回半更新快照。
 * 调用方式：诊断任务低频调用；快照不是物理链路成功证明。
 * 线程约束：最多等待 storage mutex 10 ms，禁止从 ISR 或持有同一 mutex 的路径调用。
 */
void stm_uart_get_stats(stm_uart_stats_t *stats);

/**
 * @brief 清理 UART 驱动、软件缓冲和待恢复标志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：无。
 * 调用方式：由 S3 服务在 SRP 失步/连续 BREAK 后的任务上下文调用；调用后需重新同步。
 * 线程约束：会 flush 驱动 FIFO、重置事件队列并最多等待 storage mutex 10 ms；
 *           必须在收发状态机已串行化时调用，禁止从 ISR/GATT 回调调用。
 */
void stm_uart_recover(void);

/**
 * @brief 切换 UART 波特率。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：baud_rate 为任意非零 bit/s；传输层本身不校验协议白名单。
 * 返回值：ESP_OK 表示 ESP-IDF 驱动接受切换；未初始化或 0 返回状态错误。
 * 调用方式：仅由服务层在 TLV 校验确认 SRP_BAUDRATE_DEFAULT/DEBUG、双方停止当前
 *           帧收发后调用。普通调用者不得直接绕过服务层选择任意波特率。
 * 线程约束：先执行 recover 再调用 UART 驱动重配置，可能阻塞；禁止从 ISR 调用。
 */
esp_err_t stm_uart_set_baud_rate(uint32_t baud_rate);

/**
 * @brief 无阻塞读取软件接收缓冲。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：buffer 为输出空间，max_len 为容量；buffer=NULL 时返回 0。
 * 返回值：实际复制的字节数；0 表示当前无数据。
 * 调用方式：由唯一 SRP 解析任务轮询；输出内容在返回后归调用方所有。
 * 线程约束：以零等待获取内部 mutex；禁止从 ISR 调用。
 */
int stm_uart_receive_nonblock(uint8_t *buffer, size_t max_len);

/**
 * @brief 带短超时的任务上下文接收。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：buffer/max_len 与无阻塞版本相同。
 * 返回值：实际复制的字节数；不会无限等待。
 * 调用方式：仅服务任务使用，ISR 和 BLE GATT 回调禁止调用。
 * 线程约束：最多等待 storage mutex 10 ms；同一 RX ring 应保持单消费者。
 */
int stm_uart_receive(uint8_t *buffer, size_t max_len);

/**
 * @brief 读取并清除一次接收不连续事件。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：true 表示发生过溢出/线路错误，false 表示没有新事件。
 * 调用方式：SRP 服务在喂解析器前调用；true 时应丢弃半帧并触发恢复策略。
 * 线程约束：使用原子 exchange 读取并清除，可在任务间调用；不建议在 ISR 调用。
 */
bool stm_uart_take_rx_discontinuity(void);
/**
 * @brief 读取并清除一次 BREAK 恢复阈值事件。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：true 表示达到连续 BREAK 阈值；恢复动作由服务任务执行。
 * 调用方式：任务上下文轮询，不在 UART ISR 中直接重置协议状态。
 * 线程约束：使用原子 exchange，可由单一服务任务消费。
 */
bool stm_uart_take_break_recovery(void);
/**
 * @brief 设置 UART 传输层的同步门控。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：synced=true 允许运动类 SRP 帧通过；false 只保留同步/诊断所需流量。
 * 返回值：无。
 * 调用方式：仅由 S3 服务状态机调用；断链、超时或 BUS_OFF 时必须传 false。
 * 线程约束：写入 volatile 标志；调用方仍负责状态机级串行化。
 */
void stm_uart_set_sync_state(bool synced);

#endif /* STM_UART_H */
