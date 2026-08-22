#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ld2450_parser.h"

typedef enum {
    RADAR_TARGET_STATE_NONE = 0,
    RADAR_TARGET_STATE_MOVING = 1,
    RADAR_TARGET_STATE_STATIONARY = 2,
    RADAR_TARGET_STATE_HOLDING = 3,
} radar_target_state_t;

typedef struct {
    bool enabled;
    uint16_t gate_radius_cm;
    uint16_t stationary_speed_threshold_cms;
    uint16_t stationary_time_s;
    uint16_t dropout_hold_minutes;
    uint16_t detection_range_cm;
} radar_holding_config_t;

typedef struct {
    ld2450_target_t target;
    radar_target_state_t state;
} radar_holding_result_t;

typedef struct {
    bool has_fix;
    bool stationary;
    bool held;
    ld2450_target_t target;
    uint64_t last_update_ms;
    uint64_t stationary_since_ms;
} radar_holding_track_t;

typedef struct {
    radar_holding_track_t tracks[LD2450_TARGET_COUNT];
} radar_holding_engine_t;

void radar_holding_init(radar_holding_engine_t *engine);
void radar_holding_update(
    radar_holding_engine_t *engine,
    const ld2450_target_t measurements[LD2450_TARGET_COUNT], bool radar_fresh,
    const bool force_drop[LD2450_TARGET_COUNT],
    const radar_holding_config_t *config, uint64_t now_ms,
    radar_holding_result_t results[LD2450_TARGET_COUNT]);
