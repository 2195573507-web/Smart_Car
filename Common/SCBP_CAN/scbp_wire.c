#include "scbp_wire.h"

#include <math.h>
#include <string.h>

_Static_assert(sizeof(float) == sizeof(uint32_t), "SCBP requires binary32 float");

void scbp_wire_write_u32_le(uint8_t out[4], uint32_t value)
{
    if (out == NULL) {
        return;
    }
    out[0] = (uint8_t)(value & UINT32_C(0xFF));
    out[1] = (uint8_t)((value >> 8U) & UINT32_C(0xFF));
    out[2] = (uint8_t)((value >> 16U) & UINT32_C(0xFF));
    out[3] = (uint8_t)((value >> 24U) & UINT32_C(0xFF));
}

uint32_t scbp_wire_read_u32_le(const uint8_t in[4])
{
    if (in == NULL) {
        return 0U;
    }
    return (uint32_t)in[0] |
           ((uint32_t)in[1] << 8U) |
           ((uint32_t)in[2] << 16U) |
           ((uint32_t)in[3] << 24U);
}

void scbp_wire_write_f32_le(uint8_t out[4], float value)
{
    uint32_t bits = 0U;

    if (out == NULL) {
        return;
    }
    (void)memcpy(&bits, &value, sizeof(bits));
    scbp_wire_write_u32_le(out, bits);
}

float scbp_wire_read_f32_le(const uint8_t in[4])
{
    const uint32_t bits = scbp_wire_read_u32_le(in);
    float value = 0.0f;

    (void)memcpy(&value, &bits, sizeof(value));
    return value;
}

void scbp_wire_write_f32_array_le(uint8_t *out, const float *values,
                                  size_t count)
{
    if (out == NULL || values == NULL) {
        return;
    }
    for (size_t index = 0U; index < count; ++index) {
        scbp_wire_write_f32_le(&out[index * sizeof(float)], values[index]);
    }
}

bool scbp_wire_read_f32_array_le(const uint8_t *in, size_t length,
                                 float *values, size_t count)
{
    if (in == NULL || values == NULL || length != count * sizeof(float)) {
        return false;
    }
    for (size_t index = 0U; index < count; ++index) {
        values[index] = scbp_wire_read_f32_le(&in[index * sizeof(float)]);
        if (!isfinite(values[index])) {
            return false;
        }
    }
    return true;
}
