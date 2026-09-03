#include "bsp_i2c.h"

/* I2C BSP 实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include "main.h"

extern I2C_HandleTypeDef hi2c4;

static uint8_t i2c_ready;

/**
 * @brief 将 HAL I2C 调用状态收敛为 BSP 通用状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param status 待转换的 HAL_StatusTypeDef 返回值。
 * @return HAL_OK 映射为 BSP_STATUS_OK，HAL_TIMEOUT 映射为 BSP_STATUS_TIMEOUT，其余状态映射为 BSP_STATUS_ERROR。
 * 调用方式：仅由本文件在 I2C4 阻塞 HAL 事务返回后调用，不读取或修改事务缓冲。
 * 线程约束：纯值转换，不阻塞、不使用 mutex；函数自身可在 ISR 调用栈执行，但 I2C4 所有权及 HAL 阻塞调用仅属于外层任务上下文。
 */
static bsp_status_t i2c_map_hal_status(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) {
        return BSP_STATUS_OK;
    }
    if (status == HAL_TIMEOUT) {
        return BSP_STATUS_TIMEOUT;
    }
    return BSP_STATUS_ERROR;
}

/**
 * @brief 校验调用方提供的 I2C 7 位设备地址范围。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param address_7bit 未包含 R/W 位的设备地址。
 * @return 0x00..0x7F 返回 BSP_STATUS_OK，超出范围返回 BSP_STATUS_INVALID_ARG。
 * 调用方式：由本文件所有 I2C4 事务入口在地址左移和访问 HAL 前调用。
 * 线程约束：纯范围判断，不阻塞、不使用 mutex；函数自身可在 ISR 调用栈执行，但不授予共享 I2C4 总线所有权。
 */
static bsp_status_t i2c_validate_address(uint16_t address_7bit)
{
    return address_7bit <= 0x7FU ? BSP_STATUS_OK : BSP_STATUS_INVALID_ARG;
}

/** 初始化共享 I2C 总线。 */
bsp_status_t bsp_i2c_init(void)
{
    if (hi2c4.Instance != I2C4) {
        return BSP_STATUS_NOT_READY;
    }
    if (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
        return BSP_STATUS_NOT_READY;
    }
    i2c_ready = 1U;
    return BSP_STATUS_OK;
}

/** 任务上下文阻塞写事务；当前实现不提供总线互斥。 */
bsp_status_t bsp_i2c_write(uint16_t address_7bit, const uint8_t *data,
                           size_t size, uint32_t timeout_ms)
{
    if (i2c_validate_address(address_7bit) != BSP_STATUS_OK || data == NULL ||
        size == 0U || size > UINT16_MAX) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (i2c_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    return i2c_map_hal_status(HAL_I2C_Master_Transmit(&hi2c4, address_7bit << 1,
                                                      (uint8_t *)data, (uint16_t)size,
                                                      timeout_ms));
}

/** 任务上下文读事务；失败输出无效。 */
bsp_status_t bsp_i2c_read(uint16_t address_7bit, uint8_t *data,
                          size_t size, uint32_t timeout_ms)
{
    if (i2c_validate_address(address_7bit) != BSP_STATUS_OK || data == NULL ||
        size == 0U || size > UINT16_MAX) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (i2c_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    return i2c_map_hal_status(HAL_I2C_Master_Receive(&hi2c4, address_7bit << 1,
                                                     data, (uint16_t)size, timeout_ms));
}

/** 顺序执行独立的写、读 HAL 事务；不保证 repeated-start。 */
bsp_status_t bsp_i2c_write_read(uint16_t address_7bit, const uint8_t *tx_data,
                                size_t tx_size, uint8_t *rx_data,
                                size_t rx_size, uint32_t timeout_ms)
{
    HAL_StatusTypeDef hal_status;
    if (i2c_validate_address(address_7bit) != BSP_STATUS_OK || tx_data == NULL ||
        rx_data == NULL || tx_size == 0U || rx_size == 0U ||
        tx_size > UINT16_MAX || rx_size > UINT16_MAX) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (i2c_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    hal_status = HAL_I2C_Master_Transmit(&hi2c4, address_7bit << 1,
                                         (uint8_t *)tx_data, (uint16_t)tx_size,
                                         timeout_ms);
    if (hal_status != HAL_OK) {
        return i2c_map_hal_status(hal_status);
    }
    return i2c_map_hal_status(HAL_I2C_Master_Receive(&hi2c4, address_7bit << 1,
                                                     rx_data, (uint16_t)rx_size,
                                                     timeout_ms));
}

/** 在有限次数内探测 7 位设备地址。 */
bsp_status_t bsp_i2c_probe(uint16_t address_7bit, uint32_t trials, uint32_t timeout_ms)
{
    if (i2c_validate_address(address_7bit) != BSP_STATUS_OK || trials == 0U) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (i2c_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    return i2c_map_hal_status(HAL_I2C_IsDeviceReady(&hi2c4, address_7bit << 1,
                                                    trials, timeout_ms));
}
