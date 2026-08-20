#ifndef SCBP_CRC_H
#define SCBP_CRC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t scbp_crc8_itu(const uint8_t *data, size_t length);
uint16_t scbp_crc16_modbus(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* SCBP_CRC_H */
