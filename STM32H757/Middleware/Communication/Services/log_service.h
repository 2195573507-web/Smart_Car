#ifndef LOG_SERVICE_H
#define LOG_SERVICE_H

#include <stdint.h>

/* CM7 日志服务公共接口；创建人：待确认（当前维护人：Zhiqin）。 */

#define SMARTCAR_LOG_MAX_PAYLOAD 96U
#define LOG_SERVICE_TEXT_MAX SMARTCAR_LOG_MAX_PAYLOAD

/* LOG payload：source | level | timestamp LE32 | text length LE16 | UTF-8 text。 */
#define LOG_SERVICE_PAYLOAD_HEADER_SIZE UINT16_C(8)

/** CM7 日志队列与 SRP LOG payload 共用的严重级别。 */
typedef enum {
    SMARTCAR_LOG_LEVEL_DEBUG = 0U, /**< 调试细节。 */
    SMARTCAR_LOG_LEVEL_INFO = 1U, /**< 正常状态信息。 */
    SMARTCAR_LOG_LEVEL_WARN = 2U, /**< 可恢复异常或降级警告。 */
    SMARTCAR_LOG_LEVEL_ERROR = 3U /**< 错误或安全相关失败。 */
} smartcar_log_level_t;

/**
 * @brief 将文本截断复制到有界队列，队列满或参数/状态非法时累计 drop。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param level DEBUG..ERROR；越界会作为一次丢弃。
 * @param text NUL 结尾只读文本；NULL 会作为一次丢弃，最多复制 96 字节。
 * @return 无；调用方不能从本接口确认日志最终 UART/SRP 送达。
 * 调用方式：初始化队列后由普通任务或 LOG_* 宏调用；xQueueSend 使用零等待，满队列立即丢弃。
 * 线程约束：FreeRTOS 队列支持多任务提交，但 drop 自增不是跨任务原子统计；禁止从 ISR 调用。
 */
void log_service_write(smartcar_log_level_t level, const char *text);
/**
 * @brief 清零 drop/任务状态并动态创建 24 项日志队列。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 无；队列分配失败通过后续写入 drop 和任务未启动体现。
 * 调用方式：调度器启动前或启动任务中调用一次，再调用 log_service_start()。
 * 线程约束：非幂等，重复调用会丢失旧队列引用；禁止与 write/start/task 并发或从 ISR 调用。
 */
void log_service_init(void);
/**
 * @brief 创建唯一 logger 任务，负责 UART1 输出和周期 RTOS 健康采样。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 无；队列未创建或任务已存在时直接返回，创建失败增加 drop 计数。
 * 调用方式：log_service_init() 后由启动路径调用；UART 未就绪时输出仍可能失败。
 * 线程约束：内部调用 xTaskCreate()，仅启动任务调用，禁止从 ISR 调用。
 */
void log_service_start(void);
/**
 * @brief 读取自最近一次 log_service_init() 以来的丢弃累计值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 当前 volatile 计数，只读不清零；多任务同时递增时仅作近似诊断。
 * 调用方式：低频健康/调试任务读取，不作为单条日志送达确认。
 * 线程约束：无锁快照、不阻塞，可与写入并发但不保证计数原子一致。
 */
uint32_t log_service_get_drop_count(void);

#define LOG_DEBUG(text) log_service_write(SMARTCAR_LOG_LEVEL_DEBUG, (text))
#define LOG_INFO(text) log_service_write(SMARTCAR_LOG_LEVEL_INFO, (text))
#define LOG_WARN(text) log_service_write(SMARTCAR_LOG_LEVEL_WARN, (text))
#define LOG_ERROR(text) log_service_write(SMARTCAR_LOG_LEVEL_ERROR, (text))

#endif /* LOG_SERVICE_H */
