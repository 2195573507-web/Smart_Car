#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdint.h>
#include "bsp_status.h"

/* GPIO BSP；创建人：待确认（当前维护人：Zhiqin）。
 * BSP_GPIO_LF_INT2 当前映射 PA13/SWDIO，初始化会保留调试复用而不会把它配置成
 * 传感器输入；枚举存在仅代表历史源码兼容，不能据此宣称该信号可用。 */

#ifdef __cplusplus
extern "C" {
#endif

/** 项目自有 GPIO 逻辑索引；实际端口/引脚由 BSP 映射表决定。 */
typedef enum {
    BSP_GPIO_BMI323_CS = 0, /**< BMI323 SPI 片选输出。 */
    BSP_GPIO_BMI323_INT1, /**< BMI323 INT1 输入。 */
    BSP_GPIO_LF_INT1, /**< 左前传感器 INT1 输入。 */
    BSP_GPIO_LF_INT2, /**< 左前 INT2 历史项；PA13 受 SWDIO 占用。 */
    BSP_GPIO_LB_INT1, /**< 左后传感器 INT1 输入。 */
    BSP_GPIO_LB_INT2, /**< 左后传感器 INT2 输入。 */
    BSP_GPIO_STM_RX, /**< STM 链路接收脚的逻辑索引。 */
    BSP_GPIO_STM_TX, /**< STM 链路发送脚的逻辑索引。 */
    BSP_GPIO_AT1_AIN2, /**< 电机驱动 AT1 AIN2 输出。 */
    BSP_GPIO_AT1_BIN2, /**< 电机驱动 AT1 BIN2 输出。 */
    BSP_GPIO_AT2_AIN2, /**< 电机驱动 AT2 AIN2 输出。 */
    BSP_GPIO_AT2_BIN2, /**< 电机驱动 AT2 BIN2 输出。 */
    BSP_GPIO_COUNT /**< GPIO 枚举数量哨兵，不是有效引脚。 */
} bsp_gpio_pin_t;

/** GPIO 逻辑电平。 */
typedef enum {
    BSP_GPIO_LOW = 0, /**< 逻辑低电平。 */
    BSP_GPIO_HIGH = 1 /**< 逻辑高电平。 */
} bsp_gpio_level_t;

/**
 * @brief  初始化本 BSP 拥有的输入/输出 GPIO，并设置输出安全初值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return BSP_STATUS_OK；当前实现没有可报告的 HAL 初始化返回码。
 * 调用方式：系统时钟和 GPIO 端口可访问后、SPI/传感器驱动初始化前调用。
 * 线程约束：会重配多个 GPIO 寄存器，只允许启动阶段调用；禁止从 ISR 或运行期并发调用。
 * 硬件边界：PA13 保留给 SWD，不会按 BSP_GPIO_LF_INT2 重新配置。
 */
bsp_status_t bsp_gpio_init(void);
/**
 * @brief  向 BSP 标记为输出的引脚写入逻辑电平。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  pin BSP 引脚枚举；必须在范围内且映射项必须是输出。
 * @param  level 仅允许 BSP_GPIO_LOW 或 BSP_GPIO_HIGH。
 * @return BSP_STATUS_OK，或在枚举/方向/电平无效时返回 BSP_STATUS_INVALID_ARG。
 * 调用方式：在 bsp_gpio_init() 后，由拥有该引脚的驱动调用；不得跨模块抢占 GPIO。
 * 线程约束：同步寄存器写、不阻塞且无内部锁；并发写同一引脚时由调用方串行化。
 */
bsp_status_t bsp_gpio_write(bsp_gpio_pin_t pin, bsp_gpio_level_t level);
/**
 * @brief  读取 BSP 映射引脚的当前物理逻辑电平。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  pin BSP 引脚枚举；输入和输出映射均可读取。
 * @param[out] level 必须为非 NULL；成功时写入 HIGH/LOW。
 * @return BSP_STATUS_OK；引脚越界或 level 为 NULL 时返回 BSP_STATUS_INVALID_ARG。
 * 调用方式：在对应端口完成配置后读取；PA13/LF_INT2 仍受 SWD 所有权限制。
 * 线程约束：同步寄存器读、不阻塞且无内部锁；可重入，但不能替代信号去抖/时效检查。
 */
bsp_status_t bsp_gpio_read(bsp_gpio_pin_t pin, bsp_gpio_level_t *level);
/**
 * @brief  翻转 BSP 标记为输出的引脚。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  pin BSP 输出引脚枚举。
 * @return BSP_STATUS_OK；越界或输入引脚返回 BSP_STATUS_INVALID_ARG。
 * 调用方式：仅用于拥有该引脚的模块；安全输出建议显式调用 bsp_gpio_write()。
 * 线程约束：无内部锁；并发翻转/写入可能互相覆盖，调用方必须串行化，禁止据此实现跨上下文原子协议。
 */
bsp_status_t bsp_gpio_toggle(bsp_gpio_pin_t pin);

/* 短名称仅作小型驱动兼容包装；所有权、返回值和上下文约束继承对应 bsp_* API。 */
/** @copydoc bsp_gpio_init
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t gpio_init(void) { return bsp_gpio_init(); }
/** @copydoc bsp_gpio_write
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t gpio_write(bsp_gpio_pin_t pin, bsp_gpio_level_t level)
{
    return bsp_gpio_write(pin, level);
}
/** @copydoc bsp_gpio_read
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t gpio_read(bsp_gpio_pin_t pin, bsp_gpio_level_t *level)
{
    return bsp_gpio_read(pin, level);
}
/** @copydoc bsp_gpio_toggle
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t gpio_toggle(bsp_gpio_pin_t pin) { return bsp_gpio_toggle(pin); }

#ifdef __cplusplus
}
#endif

#endif /* BSP_GPIO_H */
