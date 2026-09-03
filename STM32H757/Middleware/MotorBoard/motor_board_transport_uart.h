#ifndef MOTOR_BOARD_TRANSPORT_UART_H
#define MOTOR_BOARD_TRANSPORT_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

/*
 * MotorBoard USART6 传输层。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 本层只负责字节收发和有界环形缓冲；协议解析及 PWM/速度安全策略由上层维护。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define MB_TRANSPORT_BAUD_RATE UINT32_C(115200)
#define MB_TRANSPORT_RX_RING_SIZE UINT16_C(512)
#define MB_TRANSPORT_TX_RING_SIZE UINT16_C(512)

/** USART6 寄存器级 transport 的累计计数与当前 ring 深度。 */
typedef struct {
    uint32_t rx_bytes; /**< 从 RDR 读取的字节总数，含随后因满 ring 丢弃者。 */
    uint32_t rx_overflow; /**< RX ring 满时丢弃新字节的次数。 */
    uint32_t tx_bytes; /**< 已写入 USART6 TDR 的字节总数。 */
    uint32_t tx_overflow; /**< TX ring 空间不足而拒绝整条命令的次数。 */
    uint32_t uart_errors; /**< 清理 ORE/FE/NE/PE 的累计事件数。 */
    uint16_t rx_buffered; /**< RX ring 当前待解析字节数。 */
    uint16_t tx_buffered; /**< TX ring 当前待发送字节数。 */
} mb_transport_stats_t;

/**
 * @brief 复位 USART6 软件 ring/counter 并启用寄存器级 RX/TX 中断路径。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。返回值：无；USART6 HAL/MSP 未就绪时保持 not-ready 并记录日志。
 * 调用方式：MX_USART6_UART_Init() 后、MotorBoard 协议/任务启动前调用一次。
 * 线程约束：会重置缓冲和计数；运行中不得与 ISR/Send/Read 并发重复初始化。
 */
void MB_Transport_Init(void);
/**
 * @brief 查询寄存器级 USART6 接收路径是否 ready。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：true 表示最近一次初始化/健康恢复成功；不证明 MotorBoard 已应答。
 * 调用方式：启动/健康任务在创建协议 owner 前或故障诊断时读取。
 * 线程约束：只读 volatile 标志，不阻塞。
 */
bool MB_Transport_IsReady(void);
/**
 * @brief 清除 USART6 线路错误并确保 RXNE/错误中断处于启用状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：true 表示寄存器位已确认，false 表示 handle/硬件配置不可用。
 * 调用方式：任务健康检查或发送前调用；失败时上层应停止当前配置/控制事务。
 * 线程约束：失败路径可能格式化并写日志，禁止从 ISR 调用。
 */
bool MB_Transport_Ensure_Rx_Active(void);
/**
 * @brief 将完整 MotorBoard 文本命令复制进有界 TX ring 并启用 TXE IRQ。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param data 只读命令字节，函数在返回前完成复制。
 * @param length 字节数，必须为 1..MB_TRANSPORT_TX_RING_SIZE 且剩余 ring 足够。
 * @return true 表示已完整排队，不表示物理发送/远端执行完成；false 表示参数、RX
 *         健康或容量不足，容量不足会增加 tx_overflow。
 * 调用方式：协议层格式化完整文本命令后调用；失败时不得等待不存在的响应。
 * 线程约束：任务上下文，使用短临界区；禁止从 ISR 调用。
 */
bool MB_Transport_Send(const uint8_t *data, uint16_t length);
/**
 * @brief 从 RX ring 无阻塞取出一个字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param byte 可写输出；NULL 返回 false。
 * @return true 表示消费一个字节，false 表示参数错误或当前为空。
 * 调用方式：仅由 MotorBoard parser owner 循环调用以增量组帧。
 * 线程约束：使用短临界区；单协议消费者模型，禁止从 ISR 调用。
 */
bool MB_Transport_ReadByte(uint8_t *byte);
/**
 * @brief 丢弃全部待解析 RX 字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：无；不清 TX ring 和累计统计。
 * 调用方式：协议重同步或 MotorBoard 任务启动时调用。
 * 线程约束：使用短临界区，禁止从 ISR 调用。
 */
void MB_Transport_ClearRx(void);
/**
 * @brief 复制 USART6 ring 和错误统计快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param stats 可写输出；NULL 时直接返回。
 * 返回值：无；读取不清零计数。
 * 调用方式：MotorBoard owner 低频采样诊断状态。
 * 线程约束：短临界区复制，禁止从 ISR 调用。
 */
void MB_Transport_GetStats(mb_transport_stats_t *stats);

/**
 * @brief USART6 寄存器级中断入口，清错误、搬运 RX 字节并推进 TX ring。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 无。
 * 调用方式：仅由 USART6_IRQHandler 调用。
 * 线程约束：ISR 上下文，不进入 FreeRTOS 临界区、不执行协议解析或普通日志。
 */
void MB_Transport_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_BOARD_TRANSPORT_UART_H */
