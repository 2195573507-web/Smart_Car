#ifndef BSP_STATUS_H
#define BSP_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  BSP 公共返回状态；创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（状态契约补充）。
 * 约定：OK 只证明本次软件调用完成；不等同于传感器数据有效、线缆连通或执行器安全验收。
 * TIMEOUT 表示有界等待耗尽；NOT_READY 表示前置初始化/当前 HAL 状态不满足；
 * UNSUPPORTED 表示当前 IOC/板级映射没有实现该能力，调用方不得无界重试。
 */
typedef enum {
    BSP_STATUS_OK = 0,       /* 本次操作按当前软件契约完成。 */
    BSP_STATUS_ERROR,        /* HAL/资源等未细分的一般失败。 */
    BSP_STATUS_INVALID_ARG,  /* 参数、枚举、长度或方向不符合接口约束。 */
    BSP_STATUS_NOT_READY,    /* 外设/资源未初始化或当前状态不允许操作。 */
    BSP_STATUS_TIMEOUT,      /* 阻塞事务或锁等待超过调用方给定预算。 */
    BSP_STATUS_UNSUPPORTED   /* 当前硬件配置或 BSP 实现不提供该能力。 */
} bsp_status_t;

#ifdef __cplusplus
}
#endif

#endif /* BSP_STATUS_H */
