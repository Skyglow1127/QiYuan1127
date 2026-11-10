#pragma once
#include <stdint.h>
// Unsigned depth units; thresholds use raw device units, not assumed mm.
// stats: [0]=valid count [1]=sum [2]=min [3]=max [4]=status (0 OK,1 invalid args)
void depth_gate(const uint16_t *input, uint16_t *output, uint64_t *stats,
                uint32_t count, uint16_t lower, uint16_t upper);
