#include "srp_wire.h"

#include <math.h>
#include <string.h>

_Static_assert(sizeof(float) == sizeof(uint32_t), "SRP requires binary32 float");

void srp_wire_write_u32_le(uint8_t out[4], uint32_t value)
{
    if (out == NULL) {
        return;
    }
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8U);
    out[2] = (uint8_t)(value >> 16U);
    out[3] = (uint8_t)(value >> 24U);
}

uint32_t srp_wire_read_u32_le(const uint8_t in[4])
{
    return in == NULL ? 0U : (uint32_t)in[0] | ((uint32_t)in[1] << 8U) |
                                ((uint32_t)in[2] << 16U) |
                                ((uint32_t)in[3] << 24U);
}

void srp_wire_write_f32_le(uint8_t out[4], float value)
{
    uint32_t bits = 0U;

    if (out == NULL) {
        return;
    }
    (void)memcpy(&bits, &value, sizeof(bits));
    srp_wire_write_u32_le(out, bits);
}

float srp_wire_read_f32_le(const uint8_t in[4])
{
    float value = 0.0f;
    const uint32_t bits = srp_wire_read_u32_le(in);

    (void)memcpy(&value, &bits, sizeof(value));
    return value;
}

void srp_wire_write_f32_array_le(uint8_t *out, const float *values, size_t count)
{
    if (out == NULL || values == NULL) {
        return;
    }
    for (size_t index = 0U; index < count; ++index) {
        srp_wire_write_f32_le(&out[index * sizeof(float)], values[index]);
    }
}

bool srp_wire_read_f32_array_le(const uint8_t *in, size_t length, float *values,
                                size_t count)
{
    if (in == NULL || values == NULL || length != count * sizeof(float)) {
        return false;
    }
    for (size_t index = 0U; index < count; ++index) {
        values[index] = srp_wire_read_f32_le(&in[index * sizeof(float)]);
        if (!isfinite(values[index])) {
            return false;
        }
    }
    return true;
}
