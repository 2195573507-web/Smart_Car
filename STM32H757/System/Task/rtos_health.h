#ifndef RTOS_HEALTH_H
#define RTOS_HEALTH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* FreeRTOS 运行健康采样接口；创建人：待确认（当前维护人：Zhiqin）。 */

#define RTOS_HEALTH_TASK_NAME_LENGTH UINT32_C(16)
#define RTOS_HEALTH_MONITORED_TASK_COUNT UINT32_C(7)
#define RTOS_HEALTH_FLAG_STACK_OVERFLOW UINT32_C(0x00000001)
#define RTOS_HEALTH_FLAG_MALLOC_FAILED UINT32_C(0x00000002)

/** 可写入 `.noinit` 健康记录的致命 RTOS 事件。 */
typedef enum {
    RTOS_HEALTH_EVENT_NONE = 0U, /**< 尚无致命事件。 */
    RTOS_HEALTH_EVENT_STACK_OVERFLOW = 1U, /**< FreeRTOS 栈溢出 hook。 */
    RTOS_HEALTH_EVENT_MALLOC_FAILED = 2U /**< FreeRTOS heap 分配失败 hook。 */
} rtos_health_event_t;

/** 单个受监控任务的当前/历史最小剩余栈。 */
typedef struct {
    char task_name[RTOS_HEALTH_TASK_NAME_LENGTH]; /**< 零结尾 FreeRTOS 任务名副本。 */
    uint32_t current_free_stack_words; /**< 最近采样的剩余栈，单位 word。 */
    uint32_t minimum_free_stack_words; /**< 记录生命周期内最小剩余栈，单位 word。 */
    uint32_t sample_count; /**< 成功找到该任务并采样的次数。 */
} rtos_health_stack_watermark_t;

/** 跨软复位保留并由 checksum 保护的 RTOS 健康逻辑快照。 */
typedef struct {
    uint32_t event_flags; /**< 栈溢出/malloc 失败位图。 */
    uint32_t stack_overflow_count; /**< 栈溢出 hook 累计次数。 */
    uint32_t malloc_failed_count; /**< malloc 失败 hook 累计次数。 */
    rtos_health_event_t last_event; /**< 最近提交的致命事件类型。 */
    uintptr_t last_task_handle; /**< 最近栈溢出任务句柄整数快照，不可解引用。 */
    char last_task_name[RTOS_HEALTH_TASK_NAME_LENGTH]; /**< 最近相关任务名或 `UNKNOWN`。 */
    uint32_t current_free_heap_bytes; /**< 最近 FreeRTOS 空闲 heap，单位 byte。 */
    uint32_t minimum_free_heap_bytes; /**< 启动以来最小空闲 heap，单位 byte。 */
    rtos_health_stack_watermark_t stacks[RTOS_HEALTH_MONITORED_TASK_COUNT]; /**< 固定任务表的栈水位。 */
} rtos_health_snapshot_t;

/**
 * @brief 采样受监控任务的栈水位和 FreeRTOS 堆余量，并提交到持久健康记录。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * 传入参数：无。
 * @return 无；调度器尚未启动时不采样并直接返回。
 * 调用方式：由低优先级日志任务按健康检查周期调用；任务名必须与监控表一致。
 * 线程约束：会调用 FreeRTOS task/heap 查询 API，禁止在 ISR、故障处理或 RTOS hook 中调用；
 *           不是可重入接口，应保持单一采样 owner。
 */
void rtos_health_sample(void);

/**
 * @brief 校验 `.noinit` 健康记录并复制当前快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * @param snapshot 可写输出；不得为 NULL，失败时内容保持不变。
 * @return true 表示 magic/version/size/checksum 有效且已复制；参数无效或记录正在提交时返回 false。
 * 调用方式：启动阶段读取复位前故障，或由健康采样 owner 在普通任务上下文读取。
 * 线程约束：当前为无锁复制；不得与 sample/clear/hook 记录并发使用以要求强一致快照。
 */
bool rtos_health_get_snapshot(rtos_health_snapshot_t *snapshot);

/**
 * @brief 查询校验有效的 `.noinit` 记录中是否保留栈溢出或 malloc 失败标志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * 传入参数：无。
 * @return true 表示存在致命事件标志；记录无效或标志为零时返回 false。
 * 调用方式：可在调度器启动前读取；软复位后 RAM 未被清除时记录可保留，掉电后不保证保留。
 * 线程约束：无锁只读；不得与记录提交并发使用以要求确定结果。
 */
bool rtos_health_has_fatal_event(void);
/**
 * @brief 清零健康快照并重建版本、尺寸和校验和。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * 传入参数：无。
 * @return 无；只清诊断记录，不恢复已停止任务，也不改变外设或电机状态。
 * 调用方式：启动代码确认并上报历史故障后，或在受控维护流程中调用。
 * 线程约束：禁止从 ISR/hook 调用；必须与采样、快照读取和事件提交串行化。
 */
void rtos_health_clear(void);

/* Hook-only 入口；不得调用 FreeRTOS、HAL、libc 或 heap API。 */
/**
 * @brief 在 FreeRTOS 栈溢出 hook 中提交持久故障事件。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * @param task_handle 溢出任务句柄的整数快照；仅记录，不解引用。
 * @param task_name 任务名，只读取并复制最多 15 个字符；NULL 记录为 `UNKNOWN`。
 * @return 无；记录完成后调用方必须立即进入不可返回的 fail-stop 路径。
 * 调用方式：只允许 vApplicationStackOverflowHook() 调用，随后调用 rtos_health_halt()。
 * 线程约束：hook-only、不可重入；不得从普通任务/ISR调用，也不得并发提交其他事件。
 */
void rtos_health_record_stack_overflow(uintptr_t task_handle,
                                       const char *task_name);
/**
 * @brief 在 FreeRTOS malloc 失败 hook 中提交持久故障事件。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * 传入参数：无。
 * @return 无；记录完成后调用方必须立即进入不可返回的 fail-stop 路径。
 * 调用方式：只允许 vApplicationMallocFailedHook() 调用，随后调用 rtos_health_halt()。
 * 线程约束：hook-only、不可重入；实现不分配内存，也不调用 FreeRTOS/HAL/libc API。
 */
void rtos_health_record_malloc_failed(void);
/**
 * @brief 全局禁止中断并永久进入 WFI fail-stop 循环。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * 传入参数：无。
 * @return 不返回。
 * 调用方式：RTOS 致命 hook 完成最小持久记录后调用。
 * 线程约束：可用于调度器失效后的 hook 上下文；函数不调用 RTOS/HAL，且不会主动发送电机停机帧。
 */
void rtos_health_halt(void) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif /* RTOS_HEALTH_H */
