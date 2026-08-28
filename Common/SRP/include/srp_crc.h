#ifndef SRP_CRC_H
#define SRP_CRC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t srp_crc16_ccitt_false(const uint8_t *data, size_t length);
uint16_t srp_crc16_modbus(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* SRP_CRC_H */
