#include "bsp_gpio.h"

#include "main.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t output;
} bsp_gpio_map_t;

static const bsp_gpio_map_t gpio_map[BSP_GPIO_COUNT] = {
    [BSP_GPIO_BMI323_CS] = {GPIOC, GPIO_PIN_4, 1U},
    [BSP_GPIO_BMI323_INT1] = {GPIOB, GPIO_PIN_2, 0U},
    [BSP_GPIO_LF_INT1] = {GPIOA, GPIO_PIN_10, 0U},
    [BSP_GPIO_LF_INT2] = {GPIOA, GPIO_PIN_13, 0U},
    [BSP_GPIO_LB_INT1] = {GPIOB, GPIO_PIN_8, 0U},
    [BSP_GPIO_LB_INT2] = {GPIOB, GPIO_PIN_9, 0U},
    [BSP_GPIO_STM_RX] = {GPIOD, GPIO_PIN_4, 0U},
    [BSP_GPIO_STM_TX] = {GPIOD, GPIO_PIN_3, 1U},
    [BSP_GPIO_AT1_AIN2] = {GPIOC, GPIO_PIN_5, 1U},
    [BSP_GPIO_AT1_BIN2] = {GPIOC, GPIO_PIN_1, 1U},
    [BSP_GPIO_AT2_AIN2] = {GPIOB, GPIO_PIN_14, 1U},
    [BSP_GPIO_AT2_BIN2] = {GPIOB, GPIO_PIN_15, 1U}
};

static bsp_status_t gpio_validate(bsp_gpio_pin_t pin, const bsp_gpio_map_t **entry)
{
    if ((pin < 0) || (pin >= BSP_GPIO_COUNT) || (entry == NULL)) {
        return BSP_STATUS_INVALID_ARG;
    }
    *entry = &gpio_map[pin];
    return BSP_STATUS_OK;
}

bsp_status_t bsp_gpio_init(void)
{
    GPIO_InitTypeDef config = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    config.Mode = GPIO_MODE_INPUT;
    config.Pull = GPIO_NOPULL;
    config.Speed = GPIO_SPEED_FREQ_LOW;
    config.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOB, &config);
    config.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &config);
    config.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &config);
    /* PA13 remains under the IOC's Serial-Wire assignment. LF_INT2 can only
       be used after that physical/debug conflict is resolved. */
    config.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOD, &config);

    config.Mode = GPIO_MODE_OUTPUT_PP;
    config.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOC, &config);
    config.Pin = GPIO_PIN_1 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &config);
    config.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &config);
    config.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOD, &config);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1 | GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    return BSP_STATUS_OK;
}

bsp_status_t bsp_gpio_write(bsp_gpio_pin_t pin, bsp_gpio_level_t level)
{
    const bsp_gpio_map_t *entry;
    bsp_status_t status = gpio_validate(pin, &entry);
    if (status != BSP_STATUS_OK) {
        return status;
    }
    if (entry->output == 0U || (level != BSP_GPIO_LOW && level != BSP_GPIO_HIGH)) {
        return BSP_STATUS_INVALID_ARG;
    }
    HAL_GPIO_WritePin(entry->port, entry->pin,
                      level == BSP_GPIO_HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return BSP_STATUS_OK;
}

bsp_status_t bsp_gpio_read(bsp_gpio_pin_t pin, bsp_gpio_level_t *level)
{
    const bsp_gpio_map_t *entry;
    bsp_status_t status = gpio_validate(pin, &entry);
    if (status != BSP_STATUS_OK || level == NULL) {
        return status != BSP_STATUS_OK ? status : BSP_STATUS_INVALID_ARG;
    }
    *level = HAL_GPIO_ReadPin(entry->port, entry->pin) == GPIO_PIN_SET
                 ? BSP_GPIO_HIGH : BSP_GPIO_LOW;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_gpio_toggle(bsp_gpio_pin_t pin)
{
    const bsp_gpio_map_t *entry;
    bsp_status_t status = gpio_validate(pin, &entry);
    if (status != BSP_STATUS_OK) {
        return status;
    }
    if (entry->output == 0U) {
        return BSP_STATUS_INVALID_ARG;
    }
    HAL_GPIO_TogglePin(entry->port, entry->pin);
    return BSP_STATUS_OK;
}
