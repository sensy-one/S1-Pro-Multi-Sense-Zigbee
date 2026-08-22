#include "ltrf216a_protocol.h"

#include <limits.h>
#include <stddef.h>

#define LTRF216A_20BIT_MASK 0x000FFFFFUL
#define LTRF216A_LUX_COEFFICIENT_CENTI 45ULL
#define LTRF216A_REFERENCE_INTEGRATION_TIME_MS 100ULL

uint32_t ltrf216a_decode_20bit(
    const uint8_t data[LTRF216A_DATA_SIZE])
{
    if (data == NULL) {
        return 0U;
    }
    return (((uint32_t)data[2] << 16U) | ((uint32_t)data[1] << 8U) |
            data[0]) &
           LTRF216A_20BIT_MASK;
}

uint32_t ltrf216a_counts_to_centilux(uint32_t counts, uint8_t gain,
                                     uint16_t integration_time_ms)
{
    if (gain == 0U || integration_time_ms == 0U) {
        return 0U;
    }

    counts &= LTRF216A_20BIT_MASK;
    const uint64_t numerator =
        (uint64_t)counts * LTRF216A_LUX_COEFFICIENT_CENTI *
        LTRF216A_REFERENCE_INTEGRATION_TIME_MS;
    const uint64_t denominator = (uint64_t)gain * integration_time_ms;
    const uint64_t centilux = (numerator + denominator / 2ULL) / denominator;
    return centilux > UINT32_MAX ? UINT32_MAX : (uint32_t)centilux;
}
