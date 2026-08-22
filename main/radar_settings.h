#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "radar_zones.h"

void radar_settings_init(void);
uint16_t radar_settings_get_detection_range_cm(void);
bool radar_settings_set_detection_range_cm(uint16_t range_cm);
uint16_t radar_settings_get_movement_threshold_cms(void);
bool radar_settings_set_movement_threshold_cms(uint16_t threshold_cms);
uint16_t radar_settings_get_presence_delay_s(void);
bool radar_settings_set_presence_delay_s(uint16_t delay_s);
bool radar_settings_get_flip_y_axis(void);
bool radar_settings_set_flip_y_axis(bool flip_y_axis);
bool radar_settings_get_bluetooth_enabled(void);
bool radar_settings_set_bluetooth_enabled(bool enabled);
bool radar_settings_get_single_target(void);
bool radar_settings_set_single_target(bool enabled);
bool radar_settings_get_holding_enabled(void);
bool radar_settings_set_holding_enabled(bool enabled);
uint16_t radar_settings_get_holding_gate_radius_cm(void);
bool radar_settings_set_holding_gate_radius_cm(uint16_t radius_cm);
uint16_t radar_settings_get_stationary_speed_threshold_cms(void);
bool radar_settings_set_stationary_speed_threshold_cms(uint16_t threshold_cms);
uint16_t radar_settings_get_stationary_time_s(void);
bool radar_settings_set_stationary_time_s(uint16_t time_s);
uint16_t radar_settings_get_dropout_hold_minutes(void);
bool radar_settings_set_dropout_hold_minutes(uint16_t minutes);
void radar_settings_get_zones(radar_zones_config_t *config);
bool radar_settings_set_exclusion_point(uint8_t point_index, bool y_axis,
                                        int16_t coordinate_cm);
bool radar_settings_set_exclusion_point_count(uint8_t point_count);
bool radar_settings_set_zone_point(uint8_t zone_index, uint8_t point_index,
                                   bool y_axis, int16_t coordinate_cm);
bool radar_settings_set_zone_point_count(uint8_t zone_index,
                                         uint8_t point_count);
bool radar_settings_set_zone_movement_threshold_cms(uint8_t zone_index,
                                                     uint16_t threshold_cms);
bool radar_settings_set_zone_presence_delay_s(uint8_t zone_index,
                                               uint16_t delay_s);
