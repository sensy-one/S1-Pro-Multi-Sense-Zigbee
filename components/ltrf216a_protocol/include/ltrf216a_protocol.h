#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LTRF216A_DATA_SIZE 3U

uint32_t ltrf216a_decode_20bit(
    const uint8_t data[LTRF216A_DATA_SIZE]);

uint32_t ltrf216a_counts_to_centilux(uint32_t counts, uint8_t gain,
                                     uint16_t integration_time_ms);

#ifdef __cplusplus
}
#endif
