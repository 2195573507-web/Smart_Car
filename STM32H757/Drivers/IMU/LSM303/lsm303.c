#include "lsm303.h"

#include <stddef.h>
#include <stdio.h>

#include "bsp_i2c.h"
#include "bsp_uart.h"

#define LSM303_ACCEL_WHO_AM_I  UINT8_C(0x0F)
#define LSM303_ACCEL_CTRL1     UINT8_C(0x20)
#define LSM303_ACCEL_CTRL4     UINT8_C(0x23)
#define LSM303_ACCEL_OUT_X_L   UINT8_C(0x28)

#define LSM303_MAG_CRA         UINT8_C(0x00)
#define LSM303_MAG_CRB         UINT8_C(0x01)
#define LSM303_MAG_MR          UINT8_C(0x02)
#define LSM303_MAG_OUT_X_H     UINT8_C(0x03)
#define LSM303_MAG_ID_A        UINT8_C(0x0A)

#define LSM303_ACCEL_ID_VALUE  UINT8_C(0x33)
#define LSM303_MAG_ID_A_VALUE  UINT8_C(0x48)
#define LSM303_MAG_ID_B_VALUE  UINT8_C(0x34)
#define LSM303_MAG_ID_C_VALUE  UINT8_C(0x33)
#define LSM303_I2C_TIMEOUT_MS  UINT32_C(20)
#define LSM303_LOG_TIMEOUT_MS  UINT32_C(100)
#define LSM303_GRAVITY_MPS2    9.80665f

#define LSM303_DIAG_FOUND_ACC_19 UINT8_C(0x01)
#define LSM303_DIAG_FOUND_ACC_1D UINT8_C(0x02)
#define LSM303_DIAG_FOUND_MAG_1E UINT8_C(0x04)

static volatile uint8_t lsm303_ready;
static uint8_t lsm303_accel_address;
static uint8_t lsm303_mag_address;
static uint8_t lsm303_accel_id;
static uint8_t lsm303_mag_id[3];
static uint8_t lsm303_diagnostic_run;

static void lsm303_log_status(const char *label, bsp_status_t status)
{
    char line[96];

    if (label == NULL) {
        return;
    }
    (void)snprintf(line, sizeof(line), "%s status=%d\r\n", label, (int)status);
    (void)uart_log_write(line, LSM303_LOG_TIMEOUT_MS);
}

static void lsm303_log_hex(const char *label, uint8_t value)
{
    char line[96];

    if (label == NULL) {
        return;
    }
    (void)snprintf(line, sizeof(line), "%s=0x%02X\r\n", label, value);
    (void)uart_log_write(line, LSM303_LOG_TIMEOUT_MS);
}

static void lsm303_log_scan_address(uint16_t address)
{
    char line[64];

    (void)snprintf(line, sizeof(line), "I2C FOUND ADDRESS=0x%02X\r\n",
                   (unsigned int)address);
    (void)uart_log_write(line, LSM303_LOG_TIMEOUT_MS);
}

static uint8_t lsm303_scan_bus(void)
{
    uint32_t found_count = 0U;
    uint8_t found_addresses = 0U;

    (void)uart_log_write("LSM303 I2C SCAN START\r\n", LSM303_LOG_TIMEOUT_MS);
    for (uint16_t address = UINT16_C(0x08); address <= UINT16_C(0x77); ++address) {
        if (bsp_i2c_probe(address, 1U, LSM303_I2C_TIMEOUT_MS) == BSP_STATUS_OK) {
            lsm303_log_scan_address(address);
            ++found_count;
            if (address == UINT16_C(0x19)) {
                found_addresses |= LSM303_DIAG_FOUND_ACC_19;
            } else if (address == UINT16_C(0x1D)) {
                found_addresses |= LSM303_DIAG_FOUND_ACC_1D;
            } else if (address == UINT16_C(0x1E)) {
                found_addresses |= LSM303_DIAG_FOUND_MAG_1E;
            }
        }
    }
    if (found_count == 0U) {
        (void)uart_log_write("I2C SCAN NONE\r\n", LSM303_LOG_TIMEOUT_MS);
    }
    return found_addresses;
}

static bsp_status_t lsm303_read(uint8_t address, uint8_t reg, uint8_t *data,
                                size_t length, uint8_t auto_increment)
{
    uint8_t subaddress;
    if (data == NULL || length == 0U) {
        return BSP_STATUS_INVALID_ARG;
    }
    subaddress = (uint8_t)(reg | ((auto_increment != 0U && length > 1U) ? UINT8_C(0x80) : 0U));
    return bsp_i2c_write_read(address, &subaddress, 1U, data, length, LSM303_I2C_TIMEOUT_MS);
}

static bsp_status_t lsm303_write(uint8_t address, uint8_t reg, uint8_t value)
{
    const uint8_t payload[2] = {reg, value};
    return bsp_i2c_write(address, payload, sizeof(payload), LSM303_I2C_TIMEOUT_MS);
}

static void lsm303_diagnose_who_am_i(uint8_t found_addresses)
{
    uint8_t id;
    bsp_status_t status;

    if ((found_addresses & LSM303_DIAG_FOUND_ACC_19) != 0U) {
        status = lsm303_read(UINT8_C(0x19), LSM303_ACCEL_WHO_AM_I, &id, 1U, 0U);
        if (status == BSP_STATUS_OK) {
            lsm303_log_hex("LSM303 ACC WHO_AM_I", id);
        } else {
            lsm303_log_status("LSM303 ACC WHO_AM_I READ FAIL", status);
        }
    }
    if ((found_addresses & LSM303_DIAG_FOUND_ACC_1D) != 0U) {
        status = lsm303_read(UINT8_C(0x1D), LSM303_ACCEL_WHO_AM_I, &id, 1U, 0U);
        if (status == BSP_STATUS_OK) {
            lsm303_log_hex("LSM303 ACC WHO_AM_I", id);
        } else {
            lsm303_log_status("LSM303 ACC WHO_AM_I READ FAIL", status);
        }
    }
    if ((found_addresses & LSM303_DIAG_FOUND_MAG_1E) != 0U) {
        status = lsm303_read(UINT8_C(0x1E), LSM303_MAG_ID_A, &id, 1U, 0U);
        if (status == BSP_STATUS_OK) {
            lsm303_log_hex("LSM303 MAG WHO_AM_I", id);
        } else {
            lsm303_log_status("LSM303 MAG WHO_AM_I READ FAIL", status);
        }
    }
}

static int16_t lsm303_s16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static int16_t lsm303_s16_be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

static bsp_status_t lsm303_find_accel(void)
{
    static const uint8_t accel_addresses[] = {LSM303_ACCEL_ADDRESS_DEFAULT, UINT8_C(0x18)};
    uint8_t id = 0U;
    bsp_status_t status = BSP_STATUS_ERROR;
    bsp_status_t probe_status;

    for (size_t index = 0U; index < sizeof(accel_addresses); ++index) {
        probe_status = bsp_i2c_probe(accel_addresses[index], 2U, LSM303_I2C_TIMEOUT_MS);
        if (probe_status != BSP_STATUS_OK) {
            status = probe_status;
            continue;
        }
        status = lsm303_read(accel_addresses[index], LSM303_ACCEL_WHO_AM_I, &id, 1U, 0U);
        if (status == BSP_STATUS_OK && id == LSM303_ACCEL_ID_VALUE) {
            lsm303_accel_address = accel_addresses[index];
            lsm303_accel_id = id;
            return BSP_STATUS_OK;
        }
    }
    return status == BSP_STATUS_OK ? BSP_STATUS_ERROR : status;
}

static bsp_status_t lsm303_find_mag(void)
{
    static const uint8_t mag_addresses[] = {LSM303_MAG_ADDRESS_DEFAULT, UINT8_C(0x1D)};
    uint8_t ids[3] = {0U};
    bsp_status_t status = BSP_STATUS_ERROR;
    bsp_status_t probe_status;

    for (size_t index = 0U; index < sizeof(mag_addresses); ++index) {
        probe_status = bsp_i2c_probe(mag_addresses[index], 2U, LSM303_I2C_TIMEOUT_MS);
        if (probe_status != BSP_STATUS_OK) {
            status = probe_status;
            continue;
        }
        status = lsm303_read(mag_addresses[index], LSM303_MAG_ID_A, ids, sizeof(ids), 0U);
        if (status == BSP_STATUS_OK && ids[0] == LSM303_MAG_ID_A_VALUE &&
            ids[1] == LSM303_MAG_ID_B_VALUE && ids[2] == LSM303_MAG_ID_C_VALUE) {
            lsm303_mag_address = mag_addresses[index];
            lsm303_mag_id[0] = ids[0];
            lsm303_mag_id[1] = ids[1];
            lsm303_mag_id[2] = ids[2];
            return BSP_STATUS_OK;
        }
    }
    return status == BSP_STATUS_OK ? BSP_STATUS_ERROR : status;
}

static bsp_status_t lsm303_init_internal(uint8_t diagnostic)
{
    bsp_status_t status;
    bsp_status_t accel_status;
    bsp_status_t mag_status;
    uint8_t found_addresses;

    lsm303_ready = 0U;
    lsm303_accel_address = 0U;
    lsm303_mag_address = 0U;
    lsm303_accel_id = 0U;
    lsm303_mag_id[0] = 0U;
    lsm303_mag_id[1] = 0U;
    lsm303_mag_id[2] = 0U;

    (void)uart_log_write("[LSM303] POWER_ON\r\n", LSM303_LOG_TIMEOUT_MS);

    if (diagnostic != 0U) {
        diagnostic = lsm303_diagnostic_run == 0U ? 1U : 0U;
        if (diagnostic != 0U) {
            lsm303_diagnostic_run = 1U;
            (void)uart_log_write("LSM303 I2C TEST START\r\n", LSM303_LOG_TIMEOUT_MS);
        }
    }

    status = bsp_i2c_init();
    if (diagnostic != 0U) {
        lsm303_log_status("LSM303 I2C4 INIT", status);
    }
    if (status != BSP_STATUS_OK) {
        return status;
    }
    if (diagnostic != 0U) {
        found_addresses = lsm303_scan_bus();
        lsm303_diagnose_who_am_i(found_addresses);
    }

    accel_status = lsm303_find_accel();
    mag_status = lsm303_find_mag();
    if (accel_status != BSP_STATUS_OK) {
        return accel_status;
    }
    if (mag_status != BSP_STATUS_OK) {
        return mag_status;
    }

    (void)uart_log_write("[LSM303] DEVICE_CHECK OK\r\n", LSM303_LOG_TIMEOUT_MS);

    /* 100 Hz, all axes, normal mode, +/-2 g. */
    status = lsm303_write(lsm303_accel_address, LSM303_ACCEL_CTRL1, UINT8_C(0x57));
    if (status != BSP_STATUS_OK) {
        return status;
    }
    status = lsm303_write(lsm303_accel_address, LSM303_ACCEL_CTRL4, UINT8_C(0x00));
    if (status != BSP_STATUS_OK) {
        return status;
    }

    (void)uart_log_write("[LSM303] ACC_CONFIG OK\r\n", LSM303_LOG_TIMEOUT_MS);

    /* 15 Hz, +/-1.3 gauss, continuous conversion. */
    status = lsm303_write(lsm303_mag_address, LSM303_MAG_CRA, UINT8_C(0x10));
    if (status != BSP_STATUS_OK) {
        return status;
    }
    status = lsm303_write(lsm303_mag_address, LSM303_MAG_CRB, UINT8_C(0x20));
    if (status != BSP_STATUS_OK) {
        return status;
    }
    status = lsm303_write(lsm303_mag_address, LSM303_MAG_MR, UINT8_C(0x00));
    if (status != BSP_STATUS_OK) {
        return status;
    }

    (void)uart_log_write("[LSM303] MAG_CONFIG OK\r\n", LSM303_LOG_TIMEOUT_MS);
    lsm303_ready = 1U;
    (void)uart_log_write("[LSM303] READY\r\n", LSM303_LOG_TIMEOUT_MS);
    return BSP_STATUS_OK;
}

bsp_status_t lsm303_init(void)
{
    return lsm303_init_internal(0U);
}

bsp_status_t lsm303_init_diag(void)
{
    return lsm303_init_internal(1U);
}

bsp_status_t lsm303_get_accel_id(uint8_t *id)
{
    if (id == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (lsm303_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    *id = lsm303_accel_id;
    return BSP_STATUS_OK;
}

bsp_status_t lsm303_get_mag_id(uint8_t id[3])
{
    if (id == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (lsm303_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    id[0] = lsm303_mag_id[0];
    id[1] = lsm303_mag_id[1];
    id[2] = lsm303_mag_id[2];
    return BSP_STATUS_OK;
}

bsp_status_t lsm303_read_acc(Vector3f *acc)
{
    uint8_t raw[6];
    bsp_status_t status;
    if (acc == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (lsm303_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    status = lsm303_read(lsm303_accel_address, LSM303_ACCEL_OUT_X_L, raw, sizeof(raw), 1U);
    if (status == BSP_STATUS_OK) {
        /* Normal-mode +/-2 g output is 10-bit, left-aligned, at 4 mg/LSB. */
        const float scale = (0.004f * LSM303_GRAVITY_MPS2);
        acc->x = (float)(lsm303_s16_le(&raw[0]) / 64) * scale;
        acc->y = (float)(lsm303_s16_le(&raw[2]) / 64) * scale;
        acc->z = (float)(lsm303_s16_le(&raw[4]) / 64) * scale;
    }
    return status;
}

bsp_status_t lsm303_read_mag(Vector3f *mag)
{
    uint8_t raw[6];
    bsp_status_t status;
    if (mag == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (lsm303_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    status = lsm303_read(lsm303_mag_address, LSM303_MAG_OUT_X_H, raw, sizeof(raw), 0U);
    if (status == BSP_STATUS_OK) {
        /* DLHC order is X, Z, Y; 1.3-gauss gain is 1100/980 LSB/G. */
        const int16_t raw_x = lsm303_s16_be(&raw[0]);
        const int16_t raw_z = lsm303_s16_be(&raw[2]);
        const int16_t raw_y = lsm303_s16_be(&raw[4]);
        mag->x = (float)raw_x * (100.0f / 1100.0f);
        mag->y = (float)raw_y * (100.0f / 1100.0f);
        mag->z = (float)raw_z * (100.0f / 980.0f);
    }
    return status;
}

uint8_t lsm303_is_ready(void)
{
    return lsm303_ready;
}
