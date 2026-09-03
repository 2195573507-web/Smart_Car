#include "lsm303.h"

/* LSM303 I2C 驱动实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <stddef.h>
#include <stdio.h>

#include "bsp_i2c.h"
#include "bsp_uart.h"

#define LSM303_ACCEL_WHO_AM_I  UINT8_C(0x0F)
#define LSM303_ACCEL_CTRL1     UINT8_C(0x20)
#define LSM303_ACCEL_CTRL4     UINT8_C(0x23)
#define LSM303_ACCEL_STATUS    UINT8_C(0x27)
#define LSM303_ACCEL_OUT_X_L   UINT8_C(0x28)

#define LSM303_MAG_CRA         UINT8_C(0x00)
#define LSM303_MAG_CRB         UINT8_C(0x01)
#define LSM303_MAG_MR          UINT8_C(0x02)
#define LSM303_MAG_OUT_X_H     UINT8_C(0x03)
#define LSM303_MAG_STATUS      UINT8_C(0x09)
#define LSM303_MAG_ID_A        UINT8_C(0x0A)

#define LSM303_ACCEL_ID_VALUE  UINT8_C(0x33)
#define LSM303_MAG_ID_A_VALUE  UINT8_C(0x48)
#define LSM303_MAG_ID_B_VALUE  UINT8_C(0x34)
#define LSM303_MAG_ID_C_VALUE  UINT8_C(0x33)
#define LSM303_I2C_TIMEOUT_MS  UINT32_C(20)
#define LSM303_LOG_TIMEOUT_MS  UINT32_C(100)
#define LSM303_GRAVITY_MPS2    9.80665f
#define LSM303_ACCEL_STATUS_ZYXDA UINT8_C(0x08)
#define LSM303_MAG_STATUS_DRDY     UINT8_C(0x01)

#define LSM303_DIAG_FOUND_ACC_19 UINT8_C(0x01)
#define LSM303_DIAG_FOUND_ACC_1D UINT8_C(0x02)
#define LSM303_DIAG_FOUND_MAG_1E UINT8_C(0x04)

static volatile uint8_t lsm303_ready;
static uint8_t lsm303_accel_address;
static uint8_t lsm303_mag_address;
static uint8_t lsm303_accel_id;
static uint8_t lsm303_mag_id[3];
static uint8_t lsm303_diagnostic_run;

/**
 * @brief 将 LSM303 诊断标签和 BSP 状态码写入 UART 日志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] label 以 NUL 结尾的日志标签；NULL 时静默跳过。
 * @param[in] status 要记录的 BSP 状态码。
 * @return 无返回值；格式化或 UART 写入失败不向调用方传播，也不改变传感器状态。
 * 调用方式：由 I2C 初始化和 WHO_AM_I 诊断失败路径同步调用。
 * 线程约束：UART 写入最多阻塞 LSM303_LOG_TIMEOUT_MS，不使用本文件 mutex，禁止 ISR 调用；标签仅借用且不保留所有权。
 */
static void lsm303_log_status(const char *label, bsp_status_t status)
{
    char line[96];

    if (label == NULL) {
        return;
    }
    (void)snprintf(line, sizeof(line), "%s status=%d\r\n", label, (int)status);
    (void)uart_log_write(line, LSM303_LOG_TIMEOUT_MS);
}

/**
 * @brief 将单字节 LSM303 身份值按十六进制写入 UART 日志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] label 以 NUL 结尾的日志标签；NULL 时静默跳过。
 * @param[in] value 要记录的八位值。
 * @return 无返回值；格式化或 UART 写入失败被忽略，输入和驱动状态保持不变。
 * 调用方式：仅由 lsm303_diagnose_who_am_i() 在身份寄存器读取成功后调用。
 * 线程约束：UART 写入最多阻塞 LSM303_LOG_TIMEOUT_MS，不使用 mutex，禁止 ISR 调用；标签仅借用且不保留所有权。
 */
static void lsm303_log_hex(const char *label, uint8_t value)
{
    char line[96];

    if (label == NULL) {
        return;
    }
    (void)snprintf(line, sizeof(line), "%s=0x%02X\r\n", label, value);
    (void)uart_log_write(line, LSM303_LOG_TIMEOUT_MS);
}

/**
 * @brief 输出一个在 I2C 扫描中应答的七位地址。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] address 已应答的 I2C 地址；当前扫描范围为 0x08..0x77。
 * @return 无返回值；UART 写入失败被忽略，不影响扫描位图或设备状态。
 * 调用方式：由 lsm303_scan_bus() 对每个 probe 成功地址同步调用。
 * 线程约束：UART 写入最多阻塞 LSM303_LOG_TIMEOUT_MS，不使用 mutex，禁止 ISR 调用；仅按值读取参数，无所有权转移。
 */
static void lsm303_log_scan_address(uint16_t address)
{
    char line[64];

    (void)snprintf(line, sizeof(line), "I2C FOUND ADDRESS=0x%02X\r\n",
                   (unsigned int)address);
    (void)uart_log_write(line, LSM303_LOG_TIMEOUT_MS);
}

/**
 * @brief 扫描标准七位 I2C 地址并记录与 LSM303 诊断相关的应答位图。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 返回 0x19、0x1D、0x1E 三类地址的组合位图。
 *         0 既可能表示未发现设备，也可能表示只发现未纳入位图的地址；单次 probe 失败不单独上报。
 * 调用方式：仅由 lsm303_init_internal() 的首次有效 diagnostic 分支调用，随后交给 lsm303_diagnose_who_am_i()。
 * 线程约束：顺序执行 0x08..0x77 共 112 次带超时 I2C probe，并可能逐地址阻塞 UART。
 *           函数没有 mutex；禁止 ISR 或并发使用 I2C4；无对象所有权转移。
 */
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

/**
 * @brief 通过 BSP I2C write-read 读取一个或多个 LSM303 寄存器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] address 七位 I2C 设备地址。
 * @param[in] reg 起始寄存器地址。
 * @param[out] data 接收 length 字节的缓冲；必须非 NULL 且容量足够。
 * @param[in] length 读取字节数；必须大于 0。
 * @param[in] auto_increment 非零且 length>1 时在子地址上设置 0x80 自动递增位。
 * @return 参数无效返回 BSP_STATUS_INVALID_ARG，否则透传 bsp_i2c_write_read() 状态。
 *         非 OK 时 data 不得作为有效输出；BSP/HAL 可能已部分改写缓冲，不能承诺保持原值。
 * 调用方式：由身份诊断、地址探测、初始化后状态/三轴采样和公开 ID 路径内部同步调用。
 * 线程约束：包含最长 LSM303_I2C_TIMEOUT_MS 的同步阻塞 I2C，且不使用驱动 mutex。
 *           I2C4 必须由上层串行化；禁止 ISR 或并发总线事务；data 仅在调用期间借用。
 */
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

/**
 * @brief 通过 BSP I2C 写入一个 LSM303 单字节寄存器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] address 七位 I2C 设备地址。
 * @param[in] reg 目标寄存器地址。
 * @param[in] value 要写入的单字节值。
 * @return 透传 bsp_i2c_write() 状态；失败时硬件寄存器是否部分接收不可由本层确认，也不执行回滚。
 * 调用方式：仅由 lsm303_init_internal() 顺序配置 ACC 和 MAG 寄存器。
 * 线程约束：包含最长 LSM303_I2C_TIMEOUT_MS 的同步阻塞 I2C，且不使用 mutex。
 *           I2C4 必须由上层串行化；禁止 ISR 或并发事务；无外部缓冲所有权转移。
 */
static bsp_status_t lsm303_write(uint8_t address, uint8_t reg, uint8_t value)
{
    const uint8_t payload[2] = {reg, value};
    return bsp_i2c_write(address, payload, sizeof(payload), LSM303_I2C_TIMEOUT_MS);
}

/**
 * @brief 按扫描位图读取并记录候选地址的身份寄存器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] found_addresses lsm303_scan_bus() 返回的候选地址位图；当前 0x1D 位按加速度 WHO_AM_I 路径读取。
 * @return 无返回值；单次身份读取失败仅记录日志，不修改位图，也不阻止后续正常地址探测。
 * 调用方式：仅由 lsm303_init_internal() 在总线扫描后调用一次。
 * 线程约束：包含多次阻塞 I2C 读取和 UART 日志，不使用 mutex；禁止 ISR/并发 I2C4 使用；仅按值读取参数，无所有权转移。
 */
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

/**
 * @brief 将两个小端字节解码为有符号 16 位加速度原始量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] data 至少包含 2 字节的输入缓冲，必须非 NULL。
 * @return 解码后的 int16_t；当前实现不校验指针，前置条件不满足时无失败保护。
 * 调用方式：仅由 lsm303_read_acc() 在状态和三轴突发读取成功后调用。
 * 线程约束：纯计算，不阻塞、不使用 mutex；当前仅在采样任务调用而非 ISR；缓冲仅借用且不保留所有权。
 */
static int16_t lsm303_s16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

/**
 * @brief 将两个大端字节解码为有符号 16 位磁力计原始量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] data 至少包含 2 字节的输入缓冲，必须非 NULL。
 * @return 解码后的 int16_t；当前实现不校验指针，前置条件不满足时无失败保护。
 * 调用方式：仅由 lsm303_read_mag() 在磁场突发读取成功后调用。
 * 线程约束：纯计算，不阻塞、不使用 mutex；当前仅在采样任务调用而非 ISR；缓冲仅借用且不保留所有权。
 */
static int16_t lsm303_s16_be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

/**
 * @brief 依次探测加速度计候选地址并校验 WHO_AM_I。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 首个身份匹配设备返回 BSP_STATUS_OK，并提交静态地址和 ID。
 *         全部失败时返回最后候选的总线状态；若最后读取成功但 ID 不匹配，则返回 BSP_STATUS_ERROR。
 *         失败不写入地址或 ID。
 * 调用方式：仅由 lsm303_init_internal() 在 I2C 初始化及可选诊断后调用。
 * 线程约束：包含多次阻塞 I2C probe/read，且不使用 mutex。
 *           必须由单一初始化任务独占 I2C4；禁止 ISR 或并发采样。
 *           成功时会写共享静态状态，无所有权转移。
 */
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

/**
 * @brief 依次探测磁力计候选地址并校验连续三个身份字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 首个三字节身份匹配设备返回 BSP_STATUS_OK，并提交静态地址和 ID。
 *         全部失败时返回最后候选的总线状态；若最后读取成功但 ID 不匹配，则返回 BSP_STATUS_ERROR。
 *         失败不写入地址或 ID。
 * 调用方式：仅由 lsm303_init_internal() 调用；即使加速度探测失败，当前初始化流程仍会执行本探测。
 * 线程约束：包含多次阻塞 I2C probe/read，且不使用 mutex。
 *           必须由单一初始化任务独占 I2C4；禁止 ISR 或并发采样。
 *           成功时会写共享静态状态，无所有权转移。
 */
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

/**
 * @brief 初始化 LSM303 I2C、探测身份并顺序配置 ACC/MAG 运行寄存器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] diagnostic 非零时请求地址扫描/WHO_AM_I 日志；该详细诊断仅由全局锁存允许执行一次。
 * @return 全部探测和配置成功时返回 BSP_STATUS_OK，并置 ready=1；失败时返回首个被检查的错误并保持 ready=0。
 *         入口会清空地址和 ID，失败不会回滚已经写入的硬件寄存器。
 *         诊断锁存在 I2C 初始化前置位；首次失败后，后续请求也不会重跑扫描。
 * 调用方式：由 lsm303_init() 和 lsm303_init_diag() 在独立初始化 worker 中同步调用。
 * 线程约束：包含阻塞 UART 和 I2C 事务；首次诊断还会扫描 112 个地址。
 *           函数没有驱动 mutex且不可重入；禁止 ISR，也不得与 10 ms 采样或其他 I2C4 用户并发。
 *           不涉及外部缓冲所有权。
 */
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

/** 初始化 LSM303 加速度计和磁力计，使用正常生产诊断级别。 */
bsp_status_t lsm303_init(void)
{
    return lsm303_init_internal(0U);
}

/** 初始化并输出受限 WHO_AM_I/地址诊断；不改变正常采样接口。 */
bsp_status_t lsm303_init_diag(void)
{
    return lsm303_init_internal(1U);
}

/** 读取加速度计 ID 寄存器。 */
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

/** 读取磁力计 ID 三字节；id 至少有 3 字节空间。 */
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

/** 读取并换算加速度，输出单位由 Vector3f 接口约定。 */
bsp_status_t lsm303_read_acc(Vector3f *acc)
{
    uint8_t raw[7];
    bsp_status_t status;
    if (acc == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (lsm303_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    /* Read status and all three axes in one transaction. This prevents a
     * polling tick from publishing the same accelerometer sample twice. */
    status = lsm303_read(lsm303_accel_address, LSM303_ACCEL_STATUS, raw,
                         sizeof(raw), 1U);
    if (status == BSP_STATUS_OK &&
        (raw[0] & LSM303_ACCEL_STATUS_ZYXDA) == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    if (status == BSP_STATUS_OK) {
        /* Normal-mode +/-2 g output is 10-bit, left-aligned, at 4 mg/LSB. */
        const float scale = (0.004f * LSM303_GRAVITY_MPS2);
        acc->x = (float)(lsm303_s16_le(&raw[1]) / 64) * scale;
        acc->y = (float)(lsm303_s16_le(&raw[3]) / 64) * scale;
        acc->z = (float)(lsm303_s16_le(&raw[5]) / 64) * scale;
    }
    return status;
}

/** 读取并换算磁场向量；失败时输出不得作为有效样本。 */
bsp_status_t lsm303_read_mag(Vector3f *mag)
{
    uint8_t status_raw;
    uint8_t raw[6];
    bsp_status_t status;
    if (mag == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (lsm303_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    status = lsm303_read(lsm303_mag_address, LSM303_MAG_STATUS, &status_raw,
                         sizeof(status_raw), 0U);
    if (status == BSP_STATUS_OK &&
        (status_raw & LSM303_MAG_STATUS_DRDY) == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    if (status == BSP_STATUS_OK) {
        status = lsm303_read(lsm303_mag_address, LSM303_MAG_OUT_X_H, raw,
                             sizeof(raw), 0U);
    }
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

/** 查询 LSM303 初始化/配置是否完成。 */
uint8_t lsm303_is_ready(void)
{
    return lsm303_ready;
}
