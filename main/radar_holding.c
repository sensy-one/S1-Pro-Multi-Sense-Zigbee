#include "radar_holding.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define RADAR_HOLDING_FOV_DEGREES 120.0F
#define RADAR_HOLDING_EDGE_MARGIN_CM 30.0F
#define RADAR_HOLDING_PI 3.14159265358979323846F

static uint32_t absolute_speed_cms(int16_t speed_cms)
{
    const int32_t speed = speed_cms;
    return (uint32_t)(speed < 0 ? -speed : speed);
}

static uint64_t elapsed_ms(uint64_t now_ms, uint64_t since_ms)
{
    return now_ms >= since_ms ? now_ms - since_ms : UINT64_MAX;
}

static bool target_near_radar_edge(const ld2450_target_t *target,
                                   uint16_t detection_range_cm)
{
    const float distance_cm = target->distance_mm / 10.0F;
    const float range_cm = detection_range_cm;
    const bool near_max_range =
        range_cm > 0.0F &&
        distance_cm >= fmaxf(0.0F, range_cm - RADAR_HOLDING_EDGE_MARGIN_CM);

    const float angle_degrees =
        atan2f(target->x_mm, target->y_mm) * (180.0F / RADAR_HOLDING_PI);
    const float half_fov_degrees = RADAR_HOLDING_FOV_DEGREES * 0.5F;
    const float absolute_angle_degrees = fabsf(angle_degrees);
    if (near_max_range || absolute_angle_degrees > half_fov_degrees) {
        return true;
    }

    const float angle_to_edge_radians =
        (half_fov_degrees - absolute_angle_degrees) *
        (RADAR_HOLDING_PI / 180.0F);
    return distance_cm * angle_to_edge_radians <=
           RADAR_HOLDING_EDGE_MARGIN_CM;
}

static void clear_track(radar_holding_track_t *track,
                        radar_holding_result_t *result)
{
    memset(track, 0, sizeof(*track));
    memset(result, 0, sizeof(*result));
    result->state = RADAR_TARGET_STATE_NONE;
}

static void publish_track(const radar_holding_track_t *track,
                          radar_holding_result_t *result)
{
    result->target = track->target;
    result->target.present = true;
    result->state = track->held          ? RADAR_TARGET_STATE_HOLDING
                    : track->stationary ? RADAR_TARGET_STATE_STATIONARY
                                        : RADAR_TARGET_STATE_MOVING;
}

static void update_track(radar_holding_track_t *track,
                         const ld2450_target_t *measurement, bool radar_fresh,
                         const radar_holding_config_t *config, uint64_t now_ms,
                         radar_holding_result_t *result)
{
    const bool measurement_valid = radar_fresh && measurement->present;
    if (measurement_valid) {
        const bool stationary_measurement =
            absolute_speed_cms(measurement->speed_cms) <=
            config->stationary_speed_threshold_cms;

        if (!config->enabled && track->held) {
            track->held = false;
            track->stationary = false;
            track->stationary_since_ms = 0U;
        }

        if (track->held && stationary_measurement) {
            track->last_update_ms = now_ms;
            publish_track(track, result);
            return;
        }

        if (!stationary_measurement) {
            track->held = false;
            track->stationary = false;
            track->stationary_since_ms = 0U;
        }

        if (!track->held && track->has_fix) {
            const float delta_x_mm =
                (float)measurement->x_mm - track->target.x_mm;
            const float delta_y_mm =
                (float)measurement->y_mm - track->target.y_mm;
            const float displacement_mm =
                sqrtf(delta_x_mm * delta_x_mm + delta_y_mm * delta_y_mm);
            const float gate_mm = config->gate_radius_cm * 10.0F;
            const float allowed_gate_mm =
                track->stationary ? gate_mm * 1.5F : gate_mm;
            if (displacement_mm > allowed_gate_mm) {
                memset(track, 0, sizeof(*track));
            }
        }

        track->target = *measurement;
        track->target.present = true;
        track->has_fix = true;
        track->last_update_ms = now_ms;

        if (stationary_measurement) {
            if (!track->stationary) {
                track->stationary = true;
                track->stationary_since_ms = now_ms;
            }
            const uint64_t stationary_time_ms =
                (uint64_t)config->stationary_time_s * 1000U;
            if (config->enabled && !track->held &&
                elapsed_ms(now_ms, track->stationary_since_ms) >=
                    stationary_time_ms) {
                track->held = true;
            }
        }

        publish_track(track, result);
        return;
    }

    if (!radar_fresh || !config->enabled || !track->has_fix ||
        target_near_radar_edge(&track->target,
                               config->detection_range_cm)) {
        clear_track(track, result);
        return;
    }

    const uint64_t hold_time_ms =
        (uint64_t)config->dropout_hold_minutes * 60U * 1000U;
    if (track->held && elapsed_ms(now_ms, track->last_update_ms) <=
                           hold_time_ms) {
        publish_track(track, result);
        return;
    }

    clear_track(track, result);
}

void radar_holding_init(radar_holding_engine_t *engine)
{
    if (engine != NULL) {
        memset(engine, 0, sizeof(*engine));
    }
}

void radar_holding_update(
    radar_holding_engine_t *engine,
    const ld2450_target_t measurements[LD2450_TARGET_COUNT], bool radar_fresh,
    const bool force_drop[LD2450_TARGET_COUNT],
    const radar_holding_config_t *config, uint64_t now_ms,
    radar_holding_result_t results[LD2450_TARGET_COUNT])
{
    if (engine == NULL || measurements == NULL || config == NULL ||
        results == NULL) {
        return;
    }

    for (size_t target = 0U; target < LD2450_TARGET_COUNT; ++target) {
        if (force_drop != NULL && force_drop[target]) {
            clear_track(&engine->tracks[target], &results[target]);
            continue;
        }
        update_track(&engine->tracks[target], &measurements[target],
                     radar_fresh, config, now_ms, &results[target]);
    }
}
