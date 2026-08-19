#include "bsp_timer.h"

#include "main.h"

static volatile uint8_t timer_ready;
static uint32_t timer_last_cycles;
static uint64_t timer_cycle_high;

static void bsp_timer_initialize_locked(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    timer_last_cycles = 0U;
    timer_cycle_high = 0U;
    timer_ready = 1U;
}

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

uint32_t bsp_timer_get_ms(void)
{
    return HAL_GetTick();
}
