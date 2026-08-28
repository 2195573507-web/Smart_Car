#include "stm32h7xx_hal.h"

#include <stdint.h>

#define UART2_ECHO_TEST_BAUD_RATE UINT32_C(115200)
#define UART2_ECHO_TEST_TIMEOUT_MS UINT32_C(10)
#define UART2_ECHO_TEST_PING_PERIOD_MS UINT32_C(1000)

static UART_HandleTypeDef s_huart1;
static UART_HandleTypeDef s_huart2;

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

void HAL_UART_MspInit(UART_HandleTypeDef *uart)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef peripheral_clock = {0};

    if (uart->Instance == USART1) {
        peripheral_clock.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        peripheral_clock.Usart16ClockSelection = RCC_USART1CLKSOURCE_D2PCLK2;
        if (HAL_RCCEx_PeriphCLKConfig(&peripheral_clock) != HAL_OK) {
            error_halt();
        }
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &gpio);
    } else if (uart->Instance == USART2) {
        peripheral_clock.PeriphClockSelection = RCC_PERIPHCLK_USART2;
        peripheral_clock.Usart234578ClockSelection =
            RCC_USART234578CLKSOURCE_D2PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&peripheral_clock) != HAL_OK) {
            error_halt();
        }
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &gpio);
    }
}

static void uart_init(UART_HandleTypeDef *uart, USART_TypeDef *instance)
{
    uart->Instance = instance;
    uart->Init.BaudRate = UART2_ECHO_TEST_BAUD_RATE;
    uart->Init.WordLength = UART_WORDLENGTH_8B;
    uart->Init.StopBits = UART_STOPBITS_1;
    uart->Init.Parity = UART_PARITY_NONE;
    uart->Init.Mode = UART_MODE_TX_RX;
    uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart->Init.OverSampling = UART_OVERSAMPLING_16;
    uart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
    uart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(uart) != HAL_OK) {
        error_halt();
    }
}

static void uart1_write(const uint8_t *data, uint16_t length)
{
    (void)HAL_UART_Transmit(&s_huart1, (uint8_t *)data, length,
                            UART2_ECHO_TEST_TIMEOUT_MS);
}

static void uart1_log_echo(uint8_t byte)
{
    static const uint8_t prefix[] = "[STM_RX_ECHO] 0x";
    static const uint8_t suffix[] = "\r\n";
    static const uint8_t hex[] = "0123456789ABCDEF";
    uint8_t value[2];

    value[0] = hex[byte >> 4U];
    value[1] = hex[byte & 0x0FU];
    uart1_write(prefix, sizeof(prefix) - 1U);
    uart1_write(value, sizeof(value));
    uart1_write(suffix, sizeof(suffix) - 1U);
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

int main(void)
{
    static const uint8_t banner[] =
        "[UART2_ECHO_TEST] USART2 PA2/PA3 115200-8N1\r\n";
    static const uint8_t ping[] = "STM_PING\n";
    uint32_t last_ping_ms;

    HAL_Init();
    system_clock_config();
    uart_init(&s_huart1, USART1);
    uart_init(&s_huart2, USART2);
    uart1_write(banner, sizeof(banner) - 1U);
    last_ping_ms = HAL_GetTick();

    for (;;) {
        uint8_t byte;

        if (HAL_UART_Receive(&s_huart2, &byte, 1U,
                             UART2_ECHO_TEST_TIMEOUT_MS) == HAL_OK) {
            (void)HAL_UART_Transmit(&s_huart2, &byte, 1U,
                                    UART2_ECHO_TEST_TIMEOUT_MS);
            uart1_log_echo(byte);
        }
        if ((uint32_t)(HAL_GetTick() - last_ping_ms) >=
            UART2_ECHO_TEST_PING_PERIOD_MS) {
            last_ping_ms = HAL_GetTick();
            (void)HAL_UART_Transmit(&s_huart2, (uint8_t *)ping,
                                    sizeof(ping) - 1U,
                                    UART2_ECHO_TEST_TIMEOUT_MS);
        }
    }
}
