#include "srp_crc.h"

static const uint16_t s_ccitt_nibble_table[16] = {
    UINT16_C(0x0000), UINT16_C(0x1021), UINT16_C(0x2042), UINT16_C(0x3063),
    UINT16_C(0x4084), UINT16_C(0x50A5), UINT16_C(0x60C6), UINT16_C(0x70E7),
    UINT16_C(0x8108), UINT16_C(0x9129), UINT16_C(0xA14A), UINT16_C(0xB16B),
    UINT16_C(0xC18C), UINT16_C(0xD1AD), UINT16_C(0xE1CE), UINT16_C(0xF1EF)
};

uint16_t srp_crc16_ccitt_false(const uint8_t *data, size_t length)
{
    uint16_t crc = UINT16_C(0xFFFF);

    if (data == NULL && length != 0U) {
        return 0U;
    }
    for (size_t index = 0U; index < length; ++index) {
        crc ^= (uint16_t)data[index] << 8U;
        crc = (uint16_t)((crc << 4U) ^ s_ccitt_nibble_table[crc >> 12U]);
        crc = (uint16_t)((crc << 4U) ^ s_ccitt_nibble_table[crc >> 12U]);
    }
    return crc;
}

uint16_t srp_crc16_modbus(const uint8_t *data, size_t length)
{
    uint16_t crc = UINT16_C(0xFFFF);

    if (data == NULL && length != 0U) {
        return 0U;
    }
    for (size_t index = 0U; index < length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (uint16_t)((crc & 1U) != 0U
                                 ? (crc >> 1U) ^ UINT16_C(0xA001)
                                 : crc >> 1U);
        }
    }
    return crc;
}
