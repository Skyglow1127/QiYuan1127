// One-pass RGB888 -> RGBX8888 directly into the coherent TX mapping.
// The VPR3 PL packs RGB888 at its output; no pixel-wise unpack is needed.
#include <stdint.h>
#include <stddef.h>
void vpr3_pack(const uint8_t *restrict src, uint32_t *restrict dst, size_t count)
{
    for (size_t i = 0; i < count; ++i)
        dst[i] = (uint32_t)src[3*i] | ((uint32_t)src[3*i+1] << 8) |
                 ((uint32_t)src[3*i+2] << 16);
}
