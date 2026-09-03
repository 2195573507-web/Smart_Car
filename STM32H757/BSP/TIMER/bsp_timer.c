#include "bsp_timer.h"

/* 通用定时器 BSP 实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include "main.h"

static volatile uint8_t timer_ready;
static uint32_t timer_last_cycles;
static uint64_t timer_cycle_high;

/**
 * @brief 在调用方已屏蔽中断时启用并清零 DWT 周期计数及 64 位回绕扩展状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无；不验证 DWT 是否实际启用成功，失败只能由后续时间不推进间接体现。
 * 调用方式：仅由 bsp_timer_init() 在确认 timer_ready 为 0 且保存、屏蔽 IRQ 后调用；再次执行会重置微秒时基。
 * 线程约束：自身不阻塞、不使用 mutex，也不建立临界区；调用方独占 DWT 和扩展状态并负责 IRQ 屏蔽，禁止从 ISR 或其他模块直接取得该硬件所有权。
 */
static void bsp_timer_initialize_locked(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    timer_last_cycles = 0U;
    timer_cycle_high = 0U;
    timer_ready = 1U;
}

/** 初始化通用时间基准。 */
bsp_status_t bsp_timer_init(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (timer_ready == 0U) {
        bsp_timer_initialize_locked();
    }
    __set_PRIMASK(primask);
    return BSP_STATUS_OK;
}

/** 返回单调微秒时间。 */
uint64_t bsp_timer_get_us(void)
{
    uint32_t cycles;
    uint32_t cycles_per_us;
    uint32_t primask;
    uint64_t elapsed_cycles;

    if (timer_ready == 0U) {
        (void)bsp_timer_init();
    }

    /* Both IMU tasks read this DWT extension. Keep the wrap accounting atomic
     * so a task switch around CYCCNT rollover cannot add the high word twice. */
    primask = __get_PRIMASK();
    __disable_irq();
    cycles = DWT->CYCCNT;
    if (cycles < timer_last_cycles) {
        timer_cycle_high += (UINT64_C(1) << 32);
    }
    timer_last_cycles = cycles;
    elapsed_cycles = timer_cycle_high + cycles;
    __set_PRIMASK(primask);

    cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U) {
        return 0U;
    }
    return elapsed_cycles / cycles_per_us;
}

/** 返回单调毫秒时间。 */
uint32_t bsp_timer_get_ms(void)
{
    return HAL_GetTick();
}
