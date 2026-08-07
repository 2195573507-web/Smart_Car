#include "mag_filter.h"

#if defined(IMU_MANAGER_USE_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
#endif

#define MAG_FILTER_ALPHA 0.1f

static mag_filter_data_t mag_filter_output;
static bool mag_filter_initialized;

static void mag_filter_lock(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    taskENTER_CRITICAL();
#endif
}

static void mag_filter_unlock(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    taskEXIT_CRITICAL();
#endif
}

void mag_filter_init(void)
{
    mag_filter_lock();
    mag_filter_output = (mag_filter_data_t){0};
    mag_filter_initialized = false;
    mag_filter_unlock();
}

void mag_filter_update(const lsm_mag_data_t *raw)
{
    if (raw == NULL) {
        return;
    }

    mag_filter_lock();
    if (!mag_filter_initialized) {
        /* Seed with the first sample so the output starts at the sensor value. */
        mag_filter_output.mx = raw->mx;
        mag_filter_output.my = raw->my;
        mag_filter_output.mz = raw->mz;
        mag_filter_initialized = true;
    } else {
        mag_filter_output.mx = (MAG_FILTER_ALPHA * raw->mx) +
                               ((1.0f - MAG_FILTER_ALPHA) * mag_filter_output.mx);
        mag_filter_output.my = (MAG_FILTER_ALPHA * raw->my) +
                               ((1.0f - MAG_FILTER_ALPHA) * mag_filter_output.my);
        mag_filter_output.mz = (MAG_FILTER_ALPHA * raw->mz) +
                               ((1.0f - MAG_FILTER_ALPHA) * mag_filter_output.mz);
    }
    mag_filter_unlock();
}

bool mag_filter_get(mag_filter_data_t *out)
{
    bool ready;

    if (out == NULL) {
        return false;
    }

    mag_filter_lock();
    ready = mag_filter_initialized;
    if (ready) {
        *out = mag_filter_output;
    }
    mag_filter_unlock();
    return ready;
}
