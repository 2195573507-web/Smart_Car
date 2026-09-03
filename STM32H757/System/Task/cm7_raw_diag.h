#ifndef CM7_RAW_DIAG_H
#define CM7_RAW_DIAG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CM7 无依赖 USART1 原始诊断接口；创建人：待确认（当前维护人：Zhiqin）。
 * 仅在 SMARTCAR_RAW_DIAGNOSTICS=1 的诊断镜像中生效，默认实现为空操作。 */

/**
 * @brief 通过无依赖 USART1 轮询路径输出 `[RAW] marker`。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * @param marker 调用期间只读的零结尾文本；允许 NULL，但只输出空标记前后缀。
 * @return 无；UART 未启用或轮询超限时静默丢弃剩余输出。
 * 调用方式：仅用于诊断镜像的启动/故障边界；诊断关闭时为空操作。
 * 线程约束：诊断开启时逐段关闭 IRQ 并忙等 USART1，不可重入，不得用于实时控制路径。
 */
void cm7_raw_diag_marker(const char *marker);
/**
 * @brief 通过 USART1 输出 `[RAW] label=0xXXXXXXXX` 十六进制诊断项。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * @param label 调用期间只读的零结尾标签；应使用静态字符串。
 * @param value 以固定 8 位十六进制输出的 32 位值。
 * @return 无；诊断关闭时为空操作。
 * 调用方式：诊断镜像中记录寄存器、计数或指针低 32 位，不得作为业务日志通道。
 * 线程约束：多次关闭 IRQ 并轮询 USART1，不可重入，输出不是跨调用原子帧。
 */
void cm7_raw_diag_value(const char *label, uint32_t value);
/**
 * @brief 按字符串内容仅输出一次 marker，并占用一个固定诊断槽位。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * @param marker 生命周期必须覆盖整个固件运行期的零结尾文本，建议传入字符串常量；不得为 NULL。
 * @return 无；槽位耗尽或诊断关闭时不输出。
 * 调用方式：用于记录一次性启动边界；最多跟踪 CM7_RAW_DIAG_SLOT_COUNT 个不同名称。
 * 线程约束：槽表无锁且保存原指针，不可重入，禁止并发首次登记和实时控制路径调用。
 */
void cm7_raw_diag_once(const char *marker);
/**
 * @brief 输出任务进入标记、任务名指针，并按名称记录首次进入。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * @param task_name 生命周期覆盖任务运行期的零结尾名称，建议传入字符串常量。
 * @return 无；诊断关闭时为空操作。
 * 调用方式：诊断任务入口调用一次，用于确认调度边界，不用于业务状态上报。
 * 线程约束：会关闭 IRQ、忙等 USART1 并访问无锁槽表，仅限台架诊断镜像。
 */
void cm7_raw_diag_task_enter(const char *task_name);
/**
 * @brief 对任务循环计数，并输出前 3 次及之后每 1000 次事件。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * @param task_name 生命周期覆盖任务运行期的零结尾名称；不得为 NULL。
 * @return 无；槽位耗尽或诊断关闭时不输出。
 * 调用方式：只在定位任务卡死/调度问题时插入，问题关闭后保持宏为 0。
 * 线程约束：槽表无锁，输出时关闭 IRQ 并忙等；不得用于正式车辆控制镜像。
 */
void cm7_raw_diag_task_loop(const char *task_name);
/**
 * @brief 对 UART2 发送阶段计数，并限频输出阶段名和 32 位关联值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * @param phase 生命周期覆盖固件运行期的阶段名字符串；不得为 NULL。
 * @param value 阶段关联值，当前用于长度、ready 状态或 HAL 返回码。
 * @return 无；每个阶段只输出前 3 次及之后每 1000 次，槽满时丢弃。
 * 调用方式：由 UART Link 阻塞发送路径在关键边界调用；仅限静止台架定位。
 * 线程约束：增加 UART2 路径时延，并在 USART1 输出时关闭 IRQ；生产镜像必须禁用。
 */
void cm7_raw_diag_tx_phase(const char *phase, uint32_t value);
/**
 * @brief 输出 RTOS assert 的文件指针和行号，随后禁止中断并永久 WFI。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * @param file `__FILE__` 指针，仅以地址低 32 位输出，不解引用文本；允许 NULL。
 * @param line assert 行号，以 8 位十六进制输出。
 * @return 不返回；诊断关闭时跳过输出但仍进入 fail-stop。
 * 调用方式：只由 configASSERT 失败宏调用；本函数不会主动发送电机停机帧。
 * 线程约束：适用于 RTOS 已不可信的故障上下文；调用后所有 IRQ 永久关闭。
 */
void cm7_rtos_assert_failed(const char *file, uint32_t line)
    __attribute__((noreturn));

/**
 * @brief 在未实现中断的默认处理路径输出标记并永久 WFI。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（本次补充接口契约）。
 * 传入参数：无。
 * @return 不返回。
 * 调用方式：只由启动汇编 Default_Handler 跳转调用；诊断关闭时直接进入循环。
 * 线程约束：当前实现不主动设置 PRIMASK，也不发送电机停机帧；输出路径可能忙等 USART1。
 */
void cm7_raw_diag_default_handler(void) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif /* CM7_RAW_DIAG_H */
