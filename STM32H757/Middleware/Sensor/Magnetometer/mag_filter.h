#ifndef MAG_FILTER_H
#define MAG_FILTER_H

#include <stdbool.h>

#include "imu_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float mx;
    float my;
    float mz;
} mag_filter_data_t;

void mag_filter_init(void);
void mag_filter_update(const lsm_mag_data_t *raw);
bool mag_filter_get(mag_filter_data_t *out);

#ifdef __cplusplus
}
#endif

#endif /* MAG_FILTER_H */
