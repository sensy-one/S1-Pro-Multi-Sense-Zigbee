#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int64_t received_at_us;
    uint32_t measurement_count;
    uint32_t errors;
    uint32_t illuminance_centilux;
    uint8_t part_id;
    bool valid;
} ltrf216a_snapshot_t;

void ltrf216a_sensor_start(void);
void ltrf216a_sensor_get_snapshot(ltrf216a_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif
