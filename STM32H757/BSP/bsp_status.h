#ifndef BSP_STATUS_H
#define BSP_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_STATUS_OK = 0,
    BSP_STATUS_ERROR,
    BSP_STATUS_INVALID_ARG,
    BSP_STATUS_NOT_READY,
    BSP_STATUS_TIMEOUT,
    BSP_STATUS_UNSUPPORTED
} bsp_status_t;

#ifdef __cplusplus
}
#endif

#endif /* BSP_STATUS_H */
