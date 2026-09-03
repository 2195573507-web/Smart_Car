#include "cm7_raw_diag.h"

#include <stddef.h>

#include "stm32h7xx.h"
#include "smartcar_debug_config.h"

/* CM7 无依赖原始诊断实现；创建人：待确认（当前维护人：Zhiqin）。 */

typedef struct
{
    const char *name;
    uint32_t count;
    uint8_t used;
} raw_diag_slot_t;

#if SMARTCAR_RAW_DIAGNOSTICS

static raw_diag_slot_t s_slots[CM7_RAW_DIAG_SLOT_COUNT];

/**
 * @brief 对两个 NUL 结尾字符串执行区分大小写的完整相等比较。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] left 调用期间只读的左字符串；允许为 NULL。
 * @param[in] right 调用期间只读的右字符串；允许为 NULL。
 * @return 两个非 NULL 字符串逐字节相同且同时结束时返回 1，否则返回 0；缺失 NUL 的
 *         非法字符串可能导致越界读取且无错误码。
 * 调用方式：仅 `raw_find_slot()` 扫描已登记诊断名称时调用。
 * 线程约束：不阻塞、不获取 mutex 且函数本身可重入，但执行时间随文本长度增长；仅 raw
 *           诊断任务/故障路径调用，禁止普通 ISR，不保存或接管字符串所有权。
 */
static uint32_t raw_string_equal(const char *left, const char *right)
{
    size_t index = 0U;

    if (left == NULL || right == NULL) {
        return 0U;
    }
    while (left[index] != '\0' && right[index] != '\0') {
        if (left[index] != right[index]) {
            return 0U;
        }
        ++index;
    }
    return left[index] == right[index] ? 1U : 0U;
}

/**
 * @brief 按字符串内容查找或分配固定诊断槽，并保存 name 原指针。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] name 固件生命周期内持续有效的 NUL 结尾名称；NULL 不会匹配已有槽，函数
 *                 保存原指针而不复制文本。
 * @return 已有/新槽地址，容量耗尽时返回 NULL。
 * 调用方式：仅 once/counted 诊断路径在更新计数前调用；返回槽仍归模块静态表所有。
 * 线程约束：访问无锁 `s_slots`，不阻塞、不可重入；仅 raw 诊断任务/故障路径串行调用，
 *           禁止普通 ISR 和并发首次登记，调用方保留 name 文本所有权且不得提前失效。
 */
static raw_diag_slot_t *raw_find_slot(const char *name)
{
    raw_diag_slot_t *free_slot = NULL;
    uint32_t index;

    for (index = 0U; index < CM7_RAW_DIAG_SLOT_COUNT; ++index) {
        if (s_slots[index].used != 0U &&
            raw_string_equal(s_slots[index].name, name) != 0U) {
            return &s_slots[index];
        }
        if (free_slot == NULL && s_slots[index].used == 0U) {
            free_slot = &s_slots[index];
        }
    }
    if (free_slot != NULL) {
        free_slot->name = name;
        free_slot->count = 0U;
        free_slot->used = 1U;
    }
    return free_slot;
}

/**
 * @brief 有界轮询 USART1 状态寄存器，等待指定状态位出现。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] flag 待检测的 `USART1->ISR` 位掩码，当前调用为 TXE/TXFNF 或 TC。
 * @return 在 `CM7_RAW_DIAG_WAIT_SPINS` 次轮询内检测到任一掩码位返回 1，超限返回 0；
 *         函数不清状态位也不报告具体等待次数。
 * 调用方式：由 `raw_putc()` 等待发送寄存器，以及 `raw_write()` 等待整帧发送完成。
 * 线程约束：直接访问外设并忙等、不获取 mutex；通常在 PRIMASK 已关闭中断时运行，可能
 *           增加实时抖动，禁止普通 ISR/生产实时路径调用，不涉及对象所有权。
 */
static uint32_t raw_wait_flag(uint32_t flag)
{
    uint32_t spins = 0U;

    while ((USART1->ISR & flag) == 0U && spins < CM7_RAW_DIAG_WAIT_SPINS) {
        ++spins;
    }
    return (USART1->ISR & flag) != 0U ? 1U : 0U;
}

/**
 * @brief 在 USART1 已使能时有界等待 TX 寄存器并轮询写出一个字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] value 待发送字符，仅低 8 位写入 `USART1->TDR`。
 * @return 无返回值；USART 未启用或 TX flag 等待超限时静默丢弃该字符。
 * 调用方式：仅 `raw_write()` 和 `raw_write_u32_hex()` 在 raw diagnostics 开启时调用。
 * 线程约束：直接访问 USART1 且可能有界忙等，不获取 mutex；调用者通常已关闭 IRQ，禁止
 *           普通 ISR/并发 UART writer，不涉及指针或对象所有权。
 */
static void raw_putc(char value)
{
    if ((USART1->CR1 & (USART_CR1_UE | USART_CR1_TE)) !=
        (USART_CR1_UE | USART_CR1_TE)) {
        return;
    }
    if (raw_wait_flag(USART_ISR_TXE_TXFNF) == 0U) {
        return;
    }
    USART1->TDR = (uint8_t)value;
}

/**
 * @brief 保存 PRIMASK、关闭 IRQ，并逐字节轮询 USART1 输出零结尾文本。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] text 调用期间只读的 NUL 结尾文本；NULL 时返回，不保存该指针。
 * @return 无；UART 未启用或任一 flag 忙等超限时可能产生截断输出。
 * 调用方式：仅 SMARTCAR_RAW_DIAGNOSTICS 诊断路径使用，不作为普通日志实现。
 * 线程约束：输出全文期间保存/关闭/恢复 PRIMASK，每字节最多忙等配置次数；不使用 mutex、
 *           不可重入且会增加控制抖动，禁止普通 ISR/并发 writer，文本所有权归调用方，
 *           生产镜像必须禁用此路径。
 */
static void raw_write(const char *text)
{
    uint32_t primask;
    size_t index;

    if (text == NULL) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    for (index = 0U; text[index] != '\0'; ++index) {
        raw_putc(text[index]);
    }
    if ((USART1->CR1 & (USART_CR1_UE | USART_CR1_TE)) ==
        (USART_CR1_UE | USART_CR1_TE)) {
        (void)raw_wait_flag(USART_ISR_TC);
    }
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 将 32 位值格式化为固定 8 位大写十六进制并轮询写入 USART1。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] value 待输出的 32 位值；函数不添加 `0x` 前缀或换行。
 * @return 无返回值；任一字符因 UART 未启用/等待超限而丢失时不向调用方报告。
 * 调用方式：由 value/counted/assert raw 诊断格式化路径输出数值字段。
 * 线程约束：使用固定 8 字节栈缓冲，保存并关闭 IRQ 后逐字节有界忙等，不使用 mutex；
 *           禁止普通 ISR/并发 writer，按值传参不涉及所有权。
 */
static void raw_write_u32_hex(uint32_t value)
{
    static const char digits[] = "0123456789ABCDEF";
    char buffer[8];
    uint32_t primask;
    uint32_t index;

    for (index = 0U; index < 8U; ++index) {
        buffer[7U - index] = digits[value & 0x0FU];
        value >>= 4U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    for (index = 0U; index < 8U; ++index) {
        raw_putc(buffer[index]);
    }
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 以 `[RAW] label=0xXXXXXXXX` 格式输出一个名称和值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] label 调用期间只读的 NUL 结尾标签；NULL 会仅缺失标签片段而继续输出其余字段。
 * @param[in] value 待输出的 32 位十六进制值。
 * @return 无返回值；底层 UART 丢字节、NULL 标签或分段输出被打断均不报告。
 * 调用方式：仅公共 `cm7_raw_diag_value()` 在 raw diagnostics 打开时调用。
 * 线程约束：由多个分别关闭 IRQ 且轮询 UART 有界忙等的 raw 写操作组成，没有覆盖整行的
 *           mutex/原子锁，可能与并发 writer 交错；禁止普通 ISR/并发调用，不保存或接管
 *           label 所有权。
 */
static void raw_write_value(const char *label, uint32_t value)
{
    raw_write("[RAW] ");
    raw_write(label);
    raw_write("=0x");
    raw_write_u32_hex(value);
    raw_write("\r\n");
}

/**
 * @brief 对名称事件计数，只输出前 3 次及之后每 1000 次，并可附带十六进制值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] prefix 调用期间只读的 NUL 结尾日志类别文本；底层允许 NULL 但会产生残缺行。
 * @param[in] name 固件生命周期有效的槽名称，不得为 NULL；槽表会保存原指针。
 * @param[in] value 可选关联值。
 * @param[in] include_value 非零时输出 value 字段，0 时忽略 value。
 * @return 无；槽满时静默丢弃。
 * 调用方式：任务循环和 UART TX 阶段诊断调用。
 * 线程约束：访问无锁槽表并通过多次 `raw_write()` 分段关闭 IRQ，不阻塞 RTOS 但会忙等；
 *           禁止普通 ISR、并发和实时控制路径，name/prefix 所有权归调用方且 name 必须常驻。
 */
static void raw_counted(const char *prefix, const char *name, uint32_t value,
                        uint32_t include_value)
{
    raw_diag_slot_t *slot = raw_find_slot(name);

    if (slot == NULL) {
        return;
    }
    ++slot->count;
    if (slot->count > 3U && (slot->count % 1000U) != 0U) {
        return;
    }
    raw_write("[RAW] ");
    raw_write(prefix);
    raw_write("=");
    raw_write(name);
    if (include_value != 0U) {
        raw_write(" value=0x");
        raw_write_u32_hex(value);
    }
    raw_write(" count=0x");
    raw_write_u32_hex(slot->count);
    raw_write("\r\n");
}

#endif

/** 输出固定 marker；诊断关闭时为空操作。 */
void cm7_raw_diag_marker(const char *marker)
{
#if SMARTCAR_RAW_DIAGNOSTICS
    raw_write("[RAW] ");
    raw_write(marker);
    raw_write("\r\n");
#else
    (void)marker;
#endif
}

/** 输出 label=hex(value) 诊断项。 */
void cm7_raw_diag_value(const char *label, uint32_t value)
{
#if SMARTCAR_RAW_DIAGNOSTICS
    raw_write_value(label, value);
#else
    (void)label;
    (void)value;
#endif
}

/** 仅第一次输出 marker，避免启动日志洪泛。 */
void cm7_raw_diag_once(const char *marker)
{
#if SMARTCAR_RAW_DIAGNOSTICS
    raw_diag_slot_t *slot = raw_find_slot(marker);

    if (slot != NULL && slot->count == 0U) {
        slot->count = 1U;
        cm7_raw_diag_marker(marker);
    }
#else
    (void)marker;
#endif
}

/** 记录任务首次进入事件。 */
void cm7_raw_diag_task_enter(const char *task_name)
{
    cm7_raw_diag_marker("TASK_ENTER");
    cm7_raw_diag_value("TASK_NAME_PTR", (uint32_t)(uintptr_t)task_name);
    cm7_raw_diag_once(task_name);
}

/** 记录任务循环事件。 */
void cm7_raw_diag_task_loop(const char *task_name)
{
#if SMARTCAR_RAW_DIAGNOSTICS
    raw_counted("TASK_LOOP", task_name, 0U, 0U);
#else
    (void)task_name;
#endif
}

/** 记录 UART TX 阶段和值。 */
void cm7_raw_diag_tx_phase(const char *phase, uint32_t value)
{
#if SMARTCAR_RAW_DIAGNOSTICS
    raw_counted("UART2_TX", phase, value, 1U);
#else
    (void)phase;
    (void)value;
#endif
}

/** 默认中断处理路径；输出 marker 后停机。 */
void cm7_raw_diag_default_handler(void)
{
    cm7_raw_diag_marker("DEFAULT_HANDLER");
    for (;;) {
        __asm volatile("wfi");
    }
}

/** RTOS assert 失败路径；记录文件/行并停机。 */
void cm7_rtos_assert_failed(const char *file, uint32_t line)
{
#if SMARTCAR_RAW_DIAGNOSTICS
    raw_write("[RAW] [ERROR] RTOS_ASSERT file_ptr=0x");
    raw_write_u32_hex((uint32_t)(uintptr_t)file);
    raw_write(" line=0x");
    raw_write_u32_hex(line);
    raw_write("\r\n");
#else
    (void)file;
    (void)line;
#endif

    __disable_irq();
    for (;;) {
        __asm volatile("wfi");
    }
}
