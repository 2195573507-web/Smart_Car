#ifndef SCBP_WIRE_H
#define SCBP_WIRE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Wire helpers deliberately avoid packed-struct access and host endianness. */
void scbp_wire_write_u32_le(uint8_t out[4], uint32_t value);
uint32_t scbp_wire_read_u32_le(const uint8_t in[4]);
void scbp_wire_write_f32_le(uint8_t out[4], float value);
float scbp_wire_read_f32_le(const uint8_t in[4]);
void scbp_wire_write_f32_array_le(uint8_t *out, const float *values,
                                  size_t count);
bool scbp_wire_read_f32_array_le(const uint8_t *in, size_t length,
                                 float *values, size_t count);

#ifdef __cplusplus
}
#endif

#endif /* SCBP_WIRE_H */
