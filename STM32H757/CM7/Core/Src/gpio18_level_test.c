#include "stm32h7xx_hal.h"

#include <stdint.h>

#define PA2_LEVEL_TEST_HALF_PERIOD_MS UINT32_C(500)

static void error_halt(void)
{
    __disable_irq();
    for (;;) {
    }
}

static void system_clock_config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    oscillator.HSIState = RCC_HSI_DIV1;
    oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    oscillator.PLL.PLLM = 4;
    oscillator.PLL.PLLN = 60;
    oscillator.PLL.PLLP = 2;
    oscillator.PLL.PLLQ = 4;
    oscillator.PLL.PLLR = 2;
    oscillator.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    oscillator.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    oscillator.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
        error_halt();
    }

    clocks.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                       RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clocks.AHBCLKDivider = RCC_HCLK_DIV2;
    clocks.APB3CLKDivider = RCC_APB3_DIV2;
    clocks.APB1CLKDivider = RCC_APB1_DIV2;
    clocks.APB2CLKDivider = RCC_APB2_DIV2;
    clocks.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_4) != HAL_OK) {
        error_halt();
    }
}

static void pa2_level_test_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

int main(void)
{
    HAL_Init();
    system_clock_config();
    pa2_level_test_init();

    for (;;) {
        HAL_Delay(PA2_LEVEL_TEST_HALF_PERIOD_MS);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
    }
}
