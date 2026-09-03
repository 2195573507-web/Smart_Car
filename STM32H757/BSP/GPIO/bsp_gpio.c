#include "bsp_gpio.h"

/* GPIO BSP 实现；创建人：待确认（当前维护人：Zhiqin）。 */

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

/**
 * @brief 校验 BSP GPIO 枚举和输出地址，并返回对应的内部引脚映射。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param pin 待校验的 BSP GPIO 枚举，必须位于 0..BSP_GPIO_COUNT-1。
 * @param[out] entry 非 NULL 的映射输出地址；成功时指向只读 gpio_map 项，失败时不改写。
 * @return BSP_STATUS_OK，或在 pin 越界、entry 为 NULL 时返回 BSP_STATUS_INVALID_ARG。
 * 调用方式：仅由本文件读、写和翻转入口在访问 HAL GPIO 前调用；返回映射不转移端口所有权。
 * 线程约束：纯查表，不阻塞、不使用 mutex；函数自身可在 ISR 调用栈执行，但引脚所有权和后续 HAL 操作约束由外层接口负责。
 */
static bsp_status_t gpio_validate(bsp_gpio_pin_t pin, const bsp_gpio_map_t **entry)
{
    if ((pin < 0) || (pin >= BSP_GPIO_COUNT) || (entry == NULL)) {
        return BSP_STATUS_INVALID_ARG;
    }
    *entry = &gpio_map[pin];
    return BSP_STATUS_OK;
}

/** 初始化项目 GPIO 映射。 */
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

/** 写入一个已拥有 GPIO 的逻辑电平。 */
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

/** 读取一个 GPIO 逻辑电平。 */
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

/** 翻转一个可写 GPIO。 */
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
