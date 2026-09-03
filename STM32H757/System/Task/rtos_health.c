#include "rtos_health.h"

/* RTOS 健康快照实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx.h"

#define RTOS_HEALTH_MAGIC UINT32_C(0x5254484C)
#define RTOS_HEALTH_VERSION UINT32_C(1)
#define RTOS_HEALTH_CHECKSUM_SEED UINT32_C(0xA16E4F39)

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    rtos_health_snapshot_t snapshot;
    uint32_t checksum;
} rtos_health_record_t;

static volatile rtos_health_record_t g_rtos_health_record
    __attribute__((section(".noinit"), used, aligned(8)));

_Static_assert(configMAX_TASK_NAME_LEN == RTOS_HEALTH_TASK_NAME_LENGTH,
               "RTOS health task-name storage must match FreeRTOS");

static const char *const s_monitored_task_names[RTOS_HEALTH_MONITORED_TASK_COUNT] = {
    "imu_task",
    "uart_link",
    "s3_service",
    "logger",
    "protocol",
    "imu_data_logger",
    "bmi323_task",
};

/**
 * @brief 对持久 RTOS 健康记录中除 magic/checksum 外的 32 位字执行旋转异或校验。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] record 调用期间逐字只读的 volatile 记录；必须非 NULL、按 32 位对齐且完整。
 * @return 返回由固定 seed 计算的 32 位校验值；非法指针没有防御性失败输出，记录被并发
 *         修改时结果可能只对应混合快照。
 * 调用方式：记录校验、清空提交、fatal 事件提交和周期采样提交时调用。
 * 线程约束：固定长度 volatile 读取、无 RTOS/HAL/堆调用，不阻塞也不获取 mutex；可用于
 *           fatal hook 同步路径但禁止普通 ISR/并发 writer，函数不保存记录指针或接管所有权。
 */
static uint32_t rtos_health_checksum(const volatile rtos_health_record_t *record)
{
    const volatile uint32_t *word = (const volatile uint32_t *)record;
    uint32_t checksum = RTOS_HEALTH_CHECKSUM_SEED;
    uint32_t index;

    for (index = 0U;
         index < ((sizeof(rtos_health_record_t) / sizeof(uint32_t)) - 2U);
         ++index) {
        checksum = (checksum << 5U) | (checksum >> 27U);
        checksum ^= word[index + 1U];
    }

    return checksum;
}

/**
 * @brief 校验 `.noinit` 记录的 magic、版本、尺寸和旋转异或校验和。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 全部字段匹配时为 true；上电随机 RAM、提交中记录或损坏记录返回 false。
 * 调用方式：启动读取、普通采样和 fatal hook 提交前调用。
 * 线程约束：无锁读取全局 volatile 记录且不阻塞；写侧先清 magic，通常使并发提交保守
 *           返回 false，但不是事务快照；可用于 fatal hook，禁止普通 ISR，无对象所有权。
 */
static bool rtos_health_record_is_valid(void)
{
    if (g_rtos_health_record.magic != RTOS_HEALTH_MAGIC) {
        return false;
    }
    if ((g_rtos_health_record.version != RTOS_HEALTH_VERSION) ||
        (g_rtos_health_record.size != sizeof(g_rtos_health_record))) {
        return false;
    }

    return g_rtos_health_record.checksum ==
           rtos_health_checksum(&g_rtos_health_record);
}

/**
 * @brief 将任务名有界复制到固定 volatile 字段，不足部分补零并保证 NUL 终止。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] destination 调用方拥有且至少可写 `RTOS_HEALTH_TASK_NAME_LENGTH` 字节的
 *                         volatile 目标；不可为 NULL。
 * @param[in] source 调用期间只读的任务名；NULL 时改用常量 `UNKNOWN`，最多读取/复制
 *                   `RTOS_HEALTH_TASK_NAME_LENGTH-1` 字符。
 * @return 无返回值；有效目标总写满固定字段，非法目标无防御性失败输出。
 * 调用方式：周期采样写监控任务名，或 fatal 事件提交写最后任务名。
 * 线程约束：有界 volatile 字节写、不阻塞、不获取 mutex；可用于 fatal hook 同步路径但
 *           禁止普通 ISR/并发写同一记录，不保存或接管 source/destination 所有权。
 */
static void rtos_health_copy_task_name(volatile char *destination,
                                       const char *source)
{
    uint32_t index;

    if (source == NULL) {
        source = "UNKNOWN";
    }

    for (index = 0U; index < (RTOS_HEALTH_TASK_NAME_LENGTH - 1U); ++index) {
        const char character = source[index];

        destination[index] = character;
        if (character == '\0') {
            break;
        }
    }
    for (; index < RTOS_HEALTH_TASK_NAME_LENGTH; ++index) {
        destination[index] = '\0';
    }
}

/**
 * @brief 先失效 magic，再逐字清零并按 version/size/checksum/magic 顺序提交空记录。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；函数无失败分支，完成后记录包含当前版本/尺寸和空快照有效校验。
 * 调用方式：记录无效或用户清除时调用；DMB 保证读取方不会接受半提交记录。
 * 线程约束：不使用 RTOS/HAL/libc/mutex且不阻塞，可供 fatal hook 同步路径使用；DMB 只
 *           约束内存顺序不提供互斥，调用方必须避免多个写者/普通 ISR 并发，无外部所有权。
 */
static void rtos_health_clear_record(void)
{
    volatile uint32_t *word = (volatile uint32_t *)&g_rtos_health_record;
    uint32_t index;

    g_rtos_health_record.magic = 0U;
    __DMB();
    for (index = 1U;
         index < (sizeof(g_rtos_health_record) / sizeof(uint32_t));
         ++index) {
        word[index] = 0U;
    }
    g_rtos_health_record.version = RTOS_HEALTH_VERSION;
    g_rtos_health_record.size = sizeof(g_rtos_health_record);
    g_rtos_health_record.checksum = rtos_health_checksum(&g_rtos_health_record);
    __DMB();
    g_rtos_health_record.magic = RTOS_HEALTH_MAGIC;
}

/**
 * @brief 校验 `.noinit` 健康记录，并在无效时原地重建为空的有效记录。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；记录有效时不写入，无效时调用 clear 重建，无法区分首次上电与损坏。
 * 调用方式：仅 `rtos_health_sample()` 在修改周期健康快照前调用。
 * 线程约束：无内部 mutex且不阻塞；可能写全局持久记录，只允许正常健康采样单 writer，
 *           禁止 ISR/与 fatal 提交并发调用，不涉及外部对象所有权。
 */
static void rtos_health_ensure_initialized(void)
{
    if (!rtos_health_record_is_valid()) {
        rtos_health_clear_record();
    }
}

/**
 * @brief 在 RTOS fatal hook 中以两阶段 magic 提交一次栈溢出或 malloc 失败事件。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] event 只接受 STACK_OVERFLOW 或 MALLOC_FAILED，其他值只更新 last_event。
 * @param[in] task_handle 任务句柄整数快照，仅保存、不解引用。
 * @param[in] task_name 调用期间只读的可选任务名，最多复制 15 字符，NULL 写 `UNKNOWN`。
 * @return 无；计数在 uint32_t 上自然回绕。
 * 调用方式：仅两个 RTOS hook 包装调用，返回后立即进入 rtos_health_halt()。
 * 线程约束：不调用 FreeRTOS/HAL/libc/heap/mutex且不阻塞，可在 fatal hook 同步路径运行；
 *           不可重入，fatal hook 必须是唯一 writer，禁止普通 ISR，任务名所有权归调用方。
 */
static void rtos_health_commit_event(rtos_health_event_t event,
                                     uintptr_t task_handle,
                                     const char *task_name)
{
    if (!rtos_health_record_is_valid()) {
        rtos_health_clear_record();
    }

    const uint32_t previous_overflow_count =
        g_rtos_health_record.snapshot.stack_overflow_count;
    const uint32_t previous_malloc_failure_count =
        g_rtos_health_record.snapshot.malloc_failed_count;

    g_rtos_health_record.magic = 0U;
    __DMB();
    g_rtos_health_record.version = RTOS_HEALTH_VERSION;
    g_rtos_health_record.size = sizeof(g_rtos_health_record);
    g_rtos_health_record.snapshot.last_event = event;
    g_rtos_health_record.snapshot.last_task_handle = task_handle;
    rtos_health_copy_task_name(g_rtos_health_record.snapshot.last_task_name,
                               task_name);

    if (event == RTOS_HEALTH_EVENT_STACK_OVERFLOW) {
        g_rtos_health_record.snapshot.event_flags |=
            RTOS_HEALTH_FLAG_STACK_OVERFLOW;
        g_rtos_health_record.snapshot.stack_overflow_count =
            previous_overflow_count + 1U;
    } else if (event == RTOS_HEALTH_EVENT_MALLOC_FAILED) {
        g_rtos_health_record.snapshot.event_flags |=
            RTOS_HEALTH_FLAG_MALLOC_FAILED;
        g_rtos_health_record.snapshot.malloc_failed_count =
            previous_malloc_failure_count + 1U;
    }

    g_rtos_health_record.checksum = rtos_health_checksum(&g_rtos_health_record);
    __DMB();
    g_rtos_health_record.magic = RTOS_HEALTH_MAGIC;
}

/** 在任务上下文采样堆和受监控任务栈水位。 */
void rtos_health_sample(void)
{
    uint32_t index;

    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return;
    }

    rtos_health_ensure_initialized();
    g_rtos_health_record.magic = 0U;
    __DMB();
    g_rtos_health_record.snapshot.current_free_heap_bytes =
        (uint32_t)xPortGetFreeHeapSize();
    g_rtos_health_record.snapshot.minimum_free_heap_bytes =
        (uint32_t)xPortGetMinimumEverFreeHeapSize();

    for (index = 0U; index < RTOS_HEALTH_MONITORED_TASK_COUNT; ++index) {
        TaskHandle_t task = xTaskGetHandle(s_monitored_task_names[index]);
        volatile rtos_health_stack_watermark_t *watermark =
            &g_rtos_health_record.snapshot.stacks[index];

        rtos_health_copy_task_name(watermark->task_name,
                                   s_monitored_task_names[index]);
        if (task != NULL) {
            const uint32_t free_stack_words =
                (uint32_t)uxTaskGetStackHighWaterMark(task);

            watermark->current_free_stack_words = free_stack_words;
            if ((watermark->sample_count == 0U) ||
                (free_stack_words < watermark->minimum_free_stack_words)) {
                watermark->minimum_free_stack_words = free_stack_words;
            }
            ++watermark->sample_count;
        }
    }

    g_rtos_health_record.checksum = rtos_health_checksum(&g_rtos_health_record);
    __DMB();
    g_rtos_health_record.magic = RTOS_HEALTH_MAGIC;
}

/** 复制带校验的持久健康快照。 */
bool rtos_health_get_snapshot(rtos_health_snapshot_t *snapshot)
{
    if (snapshot == NULL || !rtos_health_record_is_valid()) {
        return false;
    }

    *snapshot = g_rtos_health_record.snapshot;
    return true;
}

/** 查询是否存在复位前保留的致命事件。 */
bool rtos_health_has_fatal_event(void)
{
    return rtos_health_record_is_valid() &&
           (g_rtos_health_record.snapshot.event_flags != 0U);
}

/** 清除持久健康记录并重建校验和。 */
void rtos_health_clear(void)
{
    rtos_health_clear_record();
}

/** 在 RTOS hook 中记录栈溢出并更新校验和。 */
void rtos_health_record_stack_overflow(uintptr_t task_handle,
                                       const char *task_name)
{
    rtos_health_commit_event(RTOS_HEALTH_EVENT_STACK_OVERFLOW,
                             task_handle, task_name);
}

/** 在 malloc hook 中记录分配失败。 */
void rtos_health_record_malloc_failed(void)
{
    rtos_health_commit_event(RTOS_HEALTH_EVENT_MALLOC_FAILED, 0U, NULL);
}

/** 进入不可恢复的安全停机循环；函数不返回。 */
void rtos_health_halt(void)
{
    __asm volatile(
        "cpsid i\n"
        "dsb sy\n"
        "1:\n"
        "wfi\n"
        "b 1b\n");
    __builtin_unreachable();
}
