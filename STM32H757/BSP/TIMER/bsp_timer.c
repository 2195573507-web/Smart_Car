#include "bsp_timer.h"

#include "main.h"

static uint8_t timer_ready;
static uint32_t timer_last_cycles;
static uint64_t timer_cycle_high;

bsp_status_t bsp_timer_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    timer_last_cycles = 0U;
    timer_cycle_high = 0U;
    timer_ready = 1U;
    return BSP_STATUS_OK;
}

uint64_t bsp_timer_get_us(void)
{
    uint32_t cycles;
    uint32_t cycles_per_us;
    if (timer_ready == 0U) {
        (void)bsp_timer_init();
    }
    cycles = DWT->CYCCNT;
    if (cycles < timer_last_cycles) {
        timer_cycle_high += (UINT64_C(1) << 32);
    }
    timer_last_cycles = cycles;
    cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U) {
        return 0U;
    }
    return (timer_cycle_high + cycles) / cycles_per_us;
}

uint32_t bsp_timer_get_ms(void)
{
    return HAL_GetTick();
}
