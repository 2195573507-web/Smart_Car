#ifndef BOOT_LOG_H
#define BOOT_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 启动日志接口。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 契约补充日期：2026-08-31。
 * UART1/日志队列就绪前的事件会被有界缓存；本模块不提供并发锁。
 */

/**
 * @brief 建立启动耗时原点，并清空尚未发送的早期事件。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 无。
 * 调用方式：HAL_Init() 之后、CubeMX 外设初始化序列之前调用一次。
 * 线程约束：读取 HAL tick 并重置全局状态，只允许在单线程启动上下文使用，禁止 ISR 调用。
 */
void boot_log_start(void);

/**
 * @brief 标记 UART1 日志路径就绪，并按记录顺序刷新早期事件。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 无。日志队列满时的丢弃结果不通过本接口返回。
 * 调用方式：仅在 bsp_uart_init(BSP_UART_USART1, ...) 成功且
 * log_service_init() 已完成后调用一次。
 * 线程约束：会格式化并入队且无内部并发锁，只允许启动任务调用，禁止 ISR 调用。
 */
void boot_log_uart_ready(void);

/**
 * @brief 记录一条带启动耗时的标准化事件。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param module 模块名字符串；只在调用期间借用，NULL 时忽略该事件。
 * @param status 状态字符串；只在调用期间借用，NULL 时忽略该事件。
 * @return 无。早期缓存或日志队列满时可丢弃记录，不向调用方返回错误。
 * 调用方式：UART1 就绪前仅由单线程启动序列调用；就绪后可在普通任务
 * 上下文入队。
 * 线程约束：早期缓存无锁，UART ready 前不可并发；函数使用 snprintf/日志队列，禁止 ISR 调用。
 */
void boot_log(const char *module, const char *status);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_LOG_H */
