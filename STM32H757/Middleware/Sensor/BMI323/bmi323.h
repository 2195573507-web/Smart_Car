#ifndef SMARTCAR_SENSOR_BMI323_H
#define SMARTCAR_SENSOR_BMI323_H

#include <stdbool.h>
#include <stdint.h>

#include "bmi323_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BMI323_ERROR_NONE = 0U,
    BMI323_ERROR_SPI_INIT = 1U,
    BMI323_ERROR_WHO_AM_I_READ = 2U,
    BMI323_ERROR_WHO_AM_I_MISMATCH = 3U,
    BMI323_ERROR_SOFT_RESET = 4U,
    BMI323_ERROR_POST_RESET_READ = 5U,
    BMI323_ERROR_ACCEL_CONFIG = 6U,
    BMI323_ERROR_GYRO_CONFIG = 7U,
    BMI323_ERROR_DATA_READ = 8U,
    BMI323_ERROR_DATA_WRITE = 9U,
    BMI323_ERROR_WHO_AM_I_TIMEOUT = 10U,
    BMI323_ERROR_SPI_TX_FAIL = 11U,
    BMI323_ERROR_SPI_RX_FAIL = 12U,
    BMI323_ERROR_WHO_AM_I_VALUE = 13U
} bmi323_error_t;

typedef enum
{
    BMI323_DIAG_STATUS_OK = 0U,
    BMI323_DIAG_STATUS_WHO_AM_I_FAIL,
    BMI323_DIAG_STATUS_SPI_READ_FAIL,
    BMI323_DIAG_STATUS_CONFIG_FAIL,
    BMI323_DIAG_STATUS_DATA_NOT_READY,
    BMI323_DIAG_STATUS_WHO_AM_I_TIMEOUT,
    BMI323_DIAG_STATUS_SPI_TX_FAIL,
    BMI323_DIAG_STATUS_SPI_RX_FAIL,
    BMI323_DIAG_STATUS_WHO_AM_I_VALUE_ERROR,
    BMI323_DIAG_STATUS_WHO_AM_I_LOW_BYTE_ERROR,
    BMI323_DIAG_STATUS_WHO_AM_I_HIGH_BYTE_ERROR
} bmi323_diag_status_t;

/* Supported output data rates for the independent BMI323 acquisition task. */
typedef enum
{
    BMI323_SAMPLE_RATE_100HZ = 100U,
    BMI323_SAMPLE_RATE_200HZ = 200U,
    BMI323_SAMPLE_RATE_400HZ = 400U,
    BMI323_SAMPLE_RATE_800HZ = 800U
} bmi323_sample_rate_t;

typedef struct
{
    /* CHIP_ID confirmed by the one-shot low-speed raw probe. */
    uint8_t who_am_i;
    /* CHIP_ID read through the normal driver path after soft reset. */
    uint8_t post_reset_who_am_i;
    /* 0 means full initialization success; a negative BMI323 error is failure. */
    int32_t init_result;
    uint16_t ctrl_acc;
    uint16_t ctrl_gyr;
    uint16_t acc_conf;
    uint16_t gyr_conf;
    uint32_t spi_error_count;
    uint32_t spi_error;
    uint32_t spi_read_count;
    uint32_t spi_read_success;
    uint32_t spi_read_fail;
    uint32_t spi_tx_fail;
    uint32_t spi_rx_fail;
    uint32_t write_count;
    uint32_t write_ok;
    uint32_t write_fail;
    uint32_t whoami_fail;
    uint32_t read_ok;
    uint32_t read_fail;
    int16_t accel_raw_x;
    int16_t accel_raw_y;
    int16_t accel_raw_z;
    int16_t gyro_raw_x;
    int16_t gyro_raw_y;
    int16_t gyro_raw_z;
    uint8_t last_whoami;
    uint8_t last_rx0;
    uint8_t last_rx1;
    uint8_t last_rx2;
    uint8_t last_rx3;
    uint16_t sample_rate_hz;
    bmi323_error_t last_error;
    bmi323_diag_status_t last_status;
} bmi323_diagnostics_t;

/* Cached raw bytes from the one-shot low-speed WHO_AM_I probe of this init attempt. */
typedef struct
{
    /* Set only when the raw probe completed and RX[2] was CHIP_ID 0x43. */
    uint8_t valid;
    uint8_t whoami;
    uint8_t rx0;
    uint8_t rx1;
    uint8_t rx2;
    uint8_t rx3;
    uint8_t spi_status;
} bmi323_diag_t;

/* Output units are m/s^2 for acceleration and rad/s for angular rate. */
bool bmi323_init(void);
/* One-shot hardware bring-up probe, invoked once by the BMI323 startup path. */
bool bmi323_spi_probe(void);
bool bmi323_read_reg(uint8_t reg, uint8_t *data, uint16_t len);
bool bmi323_write_reg(uint8_t reg, const uint8_t *data, uint16_t len);
bool bmi323_set_sample_rate(bmi323_sample_rate_t sample_rate);
bmi323_sample_rate_t bmi323_get_sample_rate(void);
bool bmi323_read_accel(float *x, float *y, float *z);
bool bmi323_read_gyro(float *x, float *y, float *z);
bool bmi323_read_raw_sample(int16_t accel[3], int16_t gyro[3]);
float bmi323_accel_raw_to_mps2(int16_t raw);
float bmi323_gyro_raw_to_rads(int16_t raw);
bool bmi323_read_temperature(float *temperature);
bmi323_error_t bmi323_get_last_error(void);
bmi323_diag_status_t bmi323_get_status(void);
uint8_t bmi323_is_online(void);
uint8_t bmi323_is_ready(void);
void bmi323_refresh_data_ready_status(void);
void bmi323_get_diagnostics(bmi323_diagnostics_t *diagnostics);
void bmi323_get_diag(bmi323_diag_t *diag);
const char *bmi323_diag_status_name(bmi323_diag_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* SMARTCAR_SENSOR_BMI323_H */
