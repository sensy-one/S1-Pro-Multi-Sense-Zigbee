#include "radar_settings.h"

#include <string.h>

#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "s1_pro_config.h"

static const char *NVS_NAMESPACE = "radar_cfg";
static const char *NVS_RANGE_KEY = "range_cm";
static const char *NVS_MOVEMENT_KEY = "move_cms";
static const char *NVS_PRESENCE_DELAY_KEY = "presence_s";
static const char *NVS_FLIP_Y_KEY = "flip_y";
static const char *NVS_BLUETOOTH_KEY = "bt_on";
static const char *NVS_SINGLE_TARGET_KEY = "single";
static const char *NVS_HOLDING_ENABLED_KEY = "hold_on";
static const char *NVS_HOLDING_GATE_RADIUS_KEY = "hold_gate";
static const char *NVS_STATIONARY_SPEED_KEY = "stat_speed";
static const char *NVS_STATIONARY_TIME_KEY = "stat_time";
static const char *NVS_DROPOUT_HOLD_TIME_KEY = "drop_min";
static const char *NVS_ZONES_KEY = "zones_v1";
static const char *NVS_EXCLUSION_POINT_COUNT_KEY = "excl_cnt";

#define RADAR_ZONES_STORAGE_VERSION 1U

typedef struct {
    radar_zone_point_t exclusion_points[RADAR_ZONE_MAX_POINTS];
    radar_detection_zone_config_t zones[RADAR_ZONE_COUNT];
} stored_radar_zones_config_t;

typedef struct {
    uint8_t version;
    stored_radar_zones_config_t config;
} stored_radar_zones_t;

static SemaphoreHandle_t s_mutex;
static uint16_t s_detection_range_cm = S1_PRO_RADAR_DEFAULT_RANGE_CM;
static uint16_t s_movement_threshold_cms =
    S1_PRO_RADAR_DEFAULT_MOVEMENT_THRESHOLD_CMS;
static uint16_t s_presence_delay_s = S1_PRO_RADAR_DEFAULT_PRESENCE_DELAY_S;
static bool s_flip_y_axis = S1_PRO_RADAR_DEFAULT_FLIP_Y_AXIS;
static bool s_bluetooth_enabled = S1_PRO_RADAR_DEFAULT_BLUETOOTH_ENABLED;
static bool s_single_target = S1_PRO_RADAR_DEFAULT_SINGLE_TARGET;
static bool s_holding_enabled = S1_PRO_RADAR_DEFAULT_HOLDING_ENABLED;
static uint16_t s_holding_gate_radius_cm =
    S1_PRO_RADAR_DEFAULT_HOLDING_GATE_RADIUS_CM;
static uint16_t s_stationary_speed_threshold_cms =
    S1_PRO_RADAR_DEFAULT_STATIONARY_SPEED_THRESHOLD_CMS;
static uint16_t s_stationary_time_s = S1_PRO_RADAR_DEFAULT_STATIONARY_TIME_S;
static uint16_t s_dropout_hold_minutes =
    S1_PRO_RADAR_DEFAULT_DROPOUT_HOLD_MINUTES;
static radar_zones_config_t s_zones = {
    .zones = {
        [0] = {
            .movement_threshold_cms =
                S1_PRO_RADAR_DEFAULT_MOVEMENT_THRESHOLD_CMS,
            .presence_delay_s = S1_PRO_RADAR_DEFAULT_PRESENCE_DELAY_S,
        },
        [1] = {
            .movement_threshold_cms =
                S1_PRO_RADAR_DEFAULT_MOVEMENT_THRESHOLD_CMS,
            .presence_delay_s = S1_PRO_RADAR_DEFAULT_PRESENCE_DELAY_S,
        },
        [2] = {
            .movement_threshold_cms =
                S1_PRO_RADAR_DEFAULT_MOVEMENT_THRESHOLD_CMS,
            .presence_delay_s = S1_PRO_RADAR_DEFAULT_PRESENCE_DELAY_S,
        },
    },
};
static bool s_dirty;
static int64_t s_changed_at_us;

static bool range_is_valid(uint16_t range_cm)
{
    return range_cm >= S1_PRO_RADAR_MIN_RANGE_CM &&
           range_cm <= S1_PRO_RADAR_MAX_RANGE_CM;
}

static bool movement_threshold_is_valid(uint16_t threshold_cms)
{
    return threshold_cms <= S1_PRO_RADAR_MAX_MOVEMENT_THRESHOLD_CMS;
}

static bool presence_delay_is_valid(uint16_t delay_s)
{
    return delay_s <= S1_PRO_RADAR_MAX_PRESENCE_DELAY_S;
}

static bool holding_gate_radius_is_valid(uint16_t radius_cm)
{
    return radius_cm >= S1_PRO_RADAR_MIN_HOLDING_GATE_RADIUS_CM &&
           radius_cm <= S1_PRO_RADAR_MAX_HOLDING_GATE_RADIUS_CM;
}

static bool stationary_speed_threshold_is_valid(uint16_t threshold_cms)
{
    return threshold_cms <=
               S1_PRO_RADAR_MAX_STATIONARY_SPEED_THRESHOLD_CMS;
}

static bool stationary_time_is_valid(uint16_t time_s)
{
    return time_s <= S1_PRO_RADAR_MAX_STATIONARY_TIME_S;
}

static bool dropout_hold_time_is_valid(uint16_t minutes)
{
    return minutes <= S1_PRO_RADAR_MAX_DROPOUT_HOLD_MINUTES;
}

static bool coordinate_is_valid(int16_t coordinate_cm)
{
    return coordinate_cm >= RADAR_ZONE_MIN_COORDINATE_CM &&
           coordinate_cm <= RADAR_ZONE_MAX_COORDINATE_CM;
}

static uint8_t infer_exclusion_point_count(
    const radar_zone_point_t points[RADAR_ZONE_MAX_POINTS])
{
    uint8_t count = RADAR_ZONE_MAX_POINTS;
    while (count > 0U && points[count - 1U].x_cm == 0 &&
           points[count - 1U].y_cm == 0) {
        --count;
    }
    return count >= 3U ? count : 0U;
}

static bool zones_config_is_valid(const radar_zones_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    if (config->exclusion_point_count > RADAR_ZONE_MAX_POINTS) {
        return false;
    }
    for (size_t point = 0U; point < RADAR_ZONE_MAX_POINTS; ++point) {
        if (!coordinate_is_valid(config->exclusion_points[point].x_cm) ||
            !coordinate_is_valid(config->exclusion_points[point].y_cm)) {
            return false;
        }
    }
    for (size_t zone = 0U; zone < RADAR_ZONE_COUNT; ++zone) {
        if (config->zones[zone].point_count > RADAR_ZONE_MAX_POINTS ||
            !movement_threshold_is_valid(
                config->zones[zone].movement_threshold_cms) ||
            !presence_delay_is_valid(config->zones[zone].presence_delay_s)) {
            return false;
        }
        for (size_t point = 0U; point < RADAR_ZONE_MAX_POINTS; ++point) {
            if (!coordinate_is_valid(config->zones[zone].points[point].x_cm) ||
                !coordinate_is_valid(config->zones[zone].points[point].y_cm)) {
                return false;
            }
        }
    }
    return true;
}

static esp_err_t save_radar_settings(uint16_t range_cm,
                                     uint16_t movement_threshold_cms,
                                     uint16_t presence_delay_s,
                                     bool flip_y_axis,
                                     bool bluetooth_enabled,
                                     bool single_target,
                                     bool holding_enabled,
                                     uint16_t holding_gate_radius_cm,
                                     uint16_t stationary_speed_threshold_cms,
                                     uint16_t stationary_time_s,
                                     uint16_t dropout_hold_minutes,
                                     const radar_zones_config_t *zones)
{
    nvs_handle_t handle = 0;
    esp_err_t status = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (status == ESP_OK) {
        status = nvs_set_u16(handle, NVS_RANGE_KEY, range_cm);
    }
    if (status == ESP_OK) {
        status = nvs_set_u16(handle, NVS_MOVEMENT_KEY,
                             movement_threshold_cms);
    }
    if (status == ESP_OK) {
        status = nvs_set_u16(handle, NVS_PRESENCE_DELAY_KEY,
                             presence_delay_s);
    }
    if (status == ESP_OK) {
        status = nvs_set_u8(handle, NVS_FLIP_Y_KEY, flip_y_axis ? 1U : 0U);
    }
    if (status == ESP_OK) {
        status = nvs_set_u8(handle, NVS_BLUETOOTH_KEY,
                            bluetooth_enabled ? 1U : 0U);
    }
    if (status == ESP_OK) {
        status = nvs_set_u8(handle, NVS_SINGLE_TARGET_KEY,
                            single_target ? 1U : 0U);
    }
    if (status == ESP_OK) {
        status = nvs_set_u8(handle, NVS_HOLDING_ENABLED_KEY,
                            holding_enabled ? 1U : 0U);
    }
    if (status == ESP_OK) {
        status = nvs_set_u16(handle, NVS_HOLDING_GATE_RADIUS_KEY,
                             holding_gate_radius_cm);
    }
    if (status == ESP_OK) {
        status = nvs_set_u16(handle, NVS_STATIONARY_SPEED_KEY,
                             stationary_speed_threshold_cms);
    }
    if (status == ESP_OK) {
        status = nvs_set_u16(handle, NVS_STATIONARY_TIME_KEY,
                             stationary_time_s);
    }
    if (status == ESP_OK) {
        status = nvs_set_u16(handle, NVS_DROPOUT_HOLD_TIME_KEY,
                             dropout_hold_minutes);
    }
    if (status == ESP_OK) {
        stored_radar_zones_t stored = {
            .version = RADAR_ZONES_STORAGE_VERSION,
        };
        memcpy(stored.config.exclusion_points, zones->exclusion_points,
               sizeof(stored.config.exclusion_points));
        memcpy(stored.config.zones, zones->zones,
               sizeof(stored.config.zones));
        status = nvs_set_blob(handle, NVS_ZONES_KEY, &stored, sizeof(stored));
    }
    if (status == ESP_OK) {
        status = nvs_set_u8(handle, NVS_EXCLUSION_POINT_COUNT_KEY,
                            zones->exclusion_point_count);
    }
    if (status == ESP_OK) {
        status = nvs_commit(handle);
    }
    if (handle != 0) {
        nvs_close(handle);
    }
    return status;
}

static void persistence_task(void *argument)
{
    (void)argument;
    const int64_t delay_us =
        (int64_t)S1_PRO_RADAR_SETTINGS_SAVE_DELAY_MS * 1000LL;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
        uint16_t range_cm = 0U;
        uint16_t movement_threshold_cms = 0U;
        uint16_t presence_delay_s = 0U;
        bool flip_y_axis = false;
        bool bluetooth_enabled = false;
        bool single_target = false;
        bool holding_enabled = false;
        uint16_t holding_gate_radius_cm = 0U;
        uint16_t stationary_speed_threshold_cms = 0U;
        uint16_t stationary_time_s = 0U;
        uint16_t dropout_hold_minutes = 0U;
        radar_zones_config_t zones = {0};
        bool save_due = false;

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        if (s_dirty && esp_timer_get_time() - s_changed_at_us >= delay_us) {
            range_cm = s_detection_range_cm;
            movement_threshold_cms = s_movement_threshold_cms;
            presence_delay_s = s_presence_delay_s;
            flip_y_axis = s_flip_y_axis;
            bluetooth_enabled = s_bluetooth_enabled;
            single_target = s_single_target;
            holding_enabled = s_holding_enabled;
            holding_gate_radius_cm = s_holding_gate_radius_cm;
            stationary_speed_threshold_cms =
                s_stationary_speed_threshold_cms;
            stationary_time_s = s_stationary_time_s;
            dropout_hold_minutes = s_dropout_hold_minutes;
            zones = s_zones;
            s_dirty = false;
            save_due = true;
        }
        xSemaphoreGive(s_mutex);

        if (!save_due) {
            continue;
        }

        const esp_err_t status =
            save_radar_settings(range_cm, movement_threshold_cms,
                                presence_delay_s, flip_y_axis,
                                bluetooth_enabled, single_target,
                                holding_enabled, holding_gate_radius_cm,
                                stationary_speed_threshold_cms,
                                stationary_time_s, dropout_hold_minutes,
                                &zones);
        if (status != ESP_OK) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            s_dirty = true;
            s_changed_at_us = esp_timer_get_time();
            xSemaphoreGive(s_mutex);
        }
    }
}

void radar_settings_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    nvs_handle_t handle = 0;
    esp_err_t status = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (status == ESP_OK) {
        uint16_t restored_range_cm = 0U;
        esp_err_t range_status =
            nvs_get_u16(handle, NVS_RANGE_KEY, &restored_range_cm);
        if (range_status == ESP_OK && range_is_valid(restored_range_cm)) {
            s_detection_range_cm = restored_range_cm;
        }

        uint16_t restored_movement_threshold_cms = 0U;
        esp_err_t movement_status = nvs_get_u16(
            handle, NVS_MOVEMENT_KEY, &restored_movement_threshold_cms);
        if (movement_status == ESP_OK &&
            movement_threshold_is_valid(restored_movement_threshold_cms)) {
            s_movement_threshold_cms = restored_movement_threshold_cms;
        }

        uint16_t restored_presence_delay_s = 0U;
        esp_err_t presence_delay_status = nvs_get_u16(
            handle, NVS_PRESENCE_DELAY_KEY, &restored_presence_delay_s);
        if (presence_delay_status == ESP_OK &&
            presence_delay_is_valid(restored_presence_delay_s)) {
            s_presence_delay_s = restored_presence_delay_s;
        }

        uint8_t restored_bool = 0U;
        esp_err_t bool_status =
            nvs_get_u8(handle, NVS_FLIP_Y_KEY, &restored_bool);
        if (bool_status == ESP_OK && restored_bool <= 1U) {
            s_flip_y_axis = restored_bool != 0U;
        }

        restored_bool = 0U;
        bool_status = nvs_get_u8(handle, NVS_BLUETOOTH_KEY, &restored_bool);
        if (bool_status == ESP_OK && restored_bool <= 1U) {
            s_bluetooth_enabled = restored_bool != 0U;
        }

        restored_bool = 0U;
        bool_status =
            nvs_get_u8(handle, NVS_SINGLE_TARGET_KEY, &restored_bool);
        if (bool_status == ESP_OK && restored_bool <= 1U) {
            s_single_target = restored_bool != 0U;
        }

        restored_bool = 0U;
        bool_status =
            nvs_get_u8(handle, NVS_HOLDING_ENABLED_KEY, &restored_bool);
        if (bool_status == ESP_OK && restored_bool <= 1U) {
            s_holding_enabled = restored_bool != 0U;
        }

        uint16_t restored_holding_value = 0U;
        esp_err_t holding_status = nvs_get_u16(
            handle, NVS_HOLDING_GATE_RADIUS_KEY, &restored_holding_value);
        if (holding_status == ESP_OK &&
            holding_gate_radius_is_valid(restored_holding_value)) {
            s_holding_gate_radius_cm = restored_holding_value;
        }

        restored_holding_value = 0U;
        holding_status = nvs_get_u16(handle, NVS_STATIONARY_SPEED_KEY,
                                     &restored_holding_value);
        if (holding_status == ESP_OK &&
            stationary_speed_threshold_is_valid(restored_holding_value)) {
            s_stationary_speed_threshold_cms = restored_holding_value;
        }

        restored_holding_value = 0U;
        holding_status = nvs_get_u16(handle, NVS_STATIONARY_TIME_KEY,
                                     &restored_holding_value);
        if (holding_status == ESP_OK &&
            stationary_time_is_valid(restored_holding_value)) {
            s_stationary_time_s = restored_holding_value;
        }

        restored_holding_value = 0U;
        holding_status = nvs_get_u16(handle, NVS_DROPOUT_HOLD_TIME_KEY,
                                     &restored_holding_value);
        if (holding_status == ESP_OK &&
            dropout_hold_time_is_valid(restored_holding_value)) {
            s_dropout_hold_minutes = restored_holding_value;
        }

        stored_radar_zones_t restored_zones = {0};
        size_t restored_zones_size = sizeof(restored_zones);
        const esp_err_t zones_status = nvs_get_blob(
            handle, NVS_ZONES_KEY, &restored_zones, &restored_zones_size);
        if (zones_status == ESP_OK &&
            restored_zones_size == sizeof(restored_zones) &&
            restored_zones.version == RADAR_ZONES_STORAGE_VERSION) {
            radar_zones_config_t restored_config = {0};
            memcpy(restored_config.exclusion_points,
                   restored_zones.config.exclusion_points,
                   sizeof(restored_config.exclusion_points));
            memcpy(restored_config.zones, restored_zones.config.zones,
                   sizeof(restored_config.zones));
            uint8_t restored_exclusion_point_count = 0U;
            const esp_err_t exclusion_count_status = nvs_get_u8(
                handle, NVS_EXCLUSION_POINT_COUNT_KEY,
                &restored_exclusion_point_count);
            restored_config.exclusion_point_count =
                exclusion_count_status == ESP_OK &&
                        restored_exclusion_point_count <=
                            RADAR_ZONE_MAX_POINTS
                    ? restored_exclusion_point_count
                    : infer_exclusion_point_count(
                          restored_config.exclusion_points);
            if (zones_config_is_valid(&restored_config)) {
                s_zones = restored_config;
            }
        }
        nvs_close(handle);
    }

    BaseType_t created =
        xTaskCreate(persistence_task, "radar_settings", 3072, NULL, 2, NULL);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

uint16_t radar_settings_get_detection_range_cm(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const uint16_t range_cm = s_detection_range_cm;
    xSemaphoreGive(s_mutex);
    return range_cm;
}

bool radar_settings_set_detection_range_cm(uint16_t range_cm)
{
    if (!range_is_valid(range_cm)) {
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_detection_range_cm != range_cm) {
        s_detection_range_cm = range_cm;
        s_dirty = true;
        s_changed_at_us = esp_timer_get_time();
    }
    xSemaphoreGive(s_mutex);
    return true;
}

uint16_t radar_settings_get_movement_threshold_cms(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const uint16_t threshold_cms = s_movement_threshold_cms;
    xSemaphoreGive(s_mutex);
    return threshold_cms;
}

bool radar_settings_set_movement_threshold_cms(uint16_t threshold_cms)
{
    if (!movement_threshold_is_valid(threshold_cms)) {
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_movement_threshold_cms != threshold_cms) {
        s_movement_threshold_cms = threshold_cms;
        s_dirty = true;
        s_changed_at_us = esp_timer_get_time();
    }
    xSemaphoreGive(s_mutex);
    return true;
}

uint16_t radar_settings_get_presence_delay_s(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const uint16_t delay_s = s_presence_delay_s;
    xSemaphoreGive(s_mutex);
    return delay_s;
}

bool radar_settings_set_presence_delay_s(uint16_t delay_s)
{
    if (!presence_delay_is_valid(delay_s)) {
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_presence_delay_s != delay_s) {
        s_presence_delay_s = delay_s;
        s_dirty = true;
        s_changed_at_us = esp_timer_get_time();
    }
    xSemaphoreGive(s_mutex);
    return true;
}

bool radar_settings_get_flip_y_axis(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const bool enabled = s_flip_y_axis;
    xSemaphoreGive(s_mutex);
    return enabled;
}

bool radar_settings_set_flip_y_axis(bool flip_y_axis)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_flip_y_axis != flip_y_axis) {
        s_flip_y_axis = flip_y_axis;
        s_dirty = true;
        s_changed_at_us = esp_timer_get_time();
    }
    xSemaphoreGive(s_mutex);
    return true;
}

bool radar_settings_get_bluetooth_enabled(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const bool enabled = s_bluetooth_enabled;
    xSemaphoreGive(s_mutex);
    return enabled;
}

bool radar_settings_set_bluetooth_enabled(bool enabled)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_bluetooth_enabled != enabled) {
        s_bluetooth_enabled = enabled;
        s_dirty = true;
        s_changed_at_us = esp_timer_get_time();
    }
    xSemaphoreGive(s_mutex);
    return true;
}

bool radar_settings_get_single_target(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const bool enabled = s_single_target;
    xSemaphoreGive(s_mutex);
    return enabled;
}

bool radar_settings_set_single_target(bool enabled)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_single_target != enabled) {
        s_single_target = enabled;
        s_dirty = true;
        s_changed_at_us = esp_timer_get_time();
    }
    xSemaphoreGive(s_mutex);
    return true;
}

bool radar_settings_get_holding_enabled(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const bool enabled = s_holding_enabled;
    xSemaphoreGive(s_mutex);
    return enabled;
}

bool radar_settings_set_holding_enabled(bool enabled)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_holding_enabled != enabled) {
        s_holding_enabled = enabled;
        s_dirty = true;
        s_changed_at_us = esp_timer_get_time();
    }
    xSemaphoreGive(s_mutex);
    return true;
}

uint16_t radar_settings_get_holding_gate_radius_cm(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const uint16_t radius_cm = s_holding_gate_radius_cm;
    xSemaphoreGive(s_mutex);
    return radius_cm;
}

bool radar_settings_set_holding_gate_radius_cm(uint16_t radius_cm)
{
    if (!holding_gate_radius_is_valid(radius_cm)) {
        return false;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_holding_gate_radius_cm != radius_cm) {
        s_holding_gate_radius_cm = radius_cm;
        s_dirty = true;
        s_changed_at_us = esp_timer_get_time();
    }
    xSemaphoreGive(s_mutex);
    return true;
}

uint16_t radar_settings_get_stationary_speed_threshold_cms(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const uint16_t threshold_cms = s_stationary_speed_threshold_cms;
    xSemaphoreGive(s_mutex);
    return threshold_cms;
}

bool radar_settings_set_stationary_speed_threshold_cms(uint16_t threshold_cms)
{
    if (!stationary_speed_threshold_is_valid(threshold_cms)) {
        return false;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_stationary_speed_threshold_cms != threshold_cms) {
        s_stationary_speed_threshold_cms = threshold_cms;
        s_dirty = true;
        s_changed_at_us = esp_timer_get_time();
    }
    xSemaphoreGive(s_mutex);
    return true;
}

uint16_t radar_settings_get_stationary_time_s(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const uint16_t time_s = s_stationary_time_s;
    xSemaphoreGive(s_mutex);
    return time_s;
}

bool radar_settings_set_stationary_time_s(uint16_t time_s)
{
    if (!stationary_time_is_valid(time_s)) {
        return false;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_stationary_time_s != time_s) {
        s_stationary_time_s = time_s;
        s_dirty = true;
        s_changed_at_us = esp_timer_get_time();
    }
    xSemaphoreGive(s_mutex);
    return true;
}

uint16_t radar_settings_get_dropout_hold_minutes(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const uint16_t minutes = s_dropout_hold_minutes;
    xSemaphoreGive(s_mutex);
    return minutes;
}

bool radar_settings_set_dropout_hold_minutes(uint16_t minutes)
{
    if (!dropout_hold_time_is_valid(minutes)) {
        return false;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_dropout_hold_minutes != minutes) {
        s_dropout_hold_minutes = minutes;
        s_dirty = true;
        s_changed_at_us = esp_timer_get_time();
    }
    xSemaphoreGive(s_mutex);
    return true;
}

void radar_settings_get_zones(radar_zones_config_t *config)
{
    if (config == NULL) {
        return;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *config = s_zones;
    xSemaphoreGive(s_mutex);
}

static void mark_zones_changed(void)
{
    s_dirty = true;
    s_changed_at_us = esp_timer_get_time();
}

bool radar_settings_set_exclusion_point(uint8_t point_index, bool y_axis,
                                        int16_t coordinate_cm)
{
    if (point_index >= RADAR_ZONE_MAX_POINTS ||
        !coordinate_is_valid(coordinate_cm)) {
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int16_t *coordinate = y_axis
                              ? &s_zones.exclusion_points[point_index].y_cm
                              : &s_zones.exclusion_points[point_index].x_cm;
    if (*coordinate != coordinate_cm) {
        *coordinate = coordinate_cm;
        mark_zones_changed();
    }
    xSemaphoreGive(s_mutex);
    return true;
}

bool radar_settings_set_exclusion_point_count(uint8_t point_count)
{
    if (point_count > RADAR_ZONE_MAX_POINTS) {
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_zones.exclusion_point_count != point_count) {
        s_zones.exclusion_point_count = point_count;
        mark_zones_changed();
    }
    xSemaphoreGive(s_mutex);
    return true;
}

bool radar_settings_set_zone_point(uint8_t zone_index, uint8_t point_index,
                                   bool y_axis, int16_t coordinate_cm)
{
    if (zone_index >= RADAR_ZONE_COUNT ||
        point_index >= RADAR_ZONE_MAX_POINTS ||
        !coordinate_is_valid(coordinate_cm)) {
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int16_t *coordinate =
        y_axis ? &s_zones.zones[zone_index].points[point_index].y_cm
               : &s_zones.zones[zone_index].points[point_index].x_cm;
    if (*coordinate != coordinate_cm) {
        *coordinate = coordinate_cm;
        mark_zones_changed();
    }
    xSemaphoreGive(s_mutex);
    return true;
}

bool radar_settings_set_zone_point_count(uint8_t zone_index,
                                         uint8_t point_count)
{
    if (zone_index >= RADAR_ZONE_COUNT ||
        point_count > RADAR_ZONE_MAX_POINTS) {
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_zones.zones[zone_index].point_count != point_count) {
        s_zones.zones[zone_index].point_count = point_count;
        mark_zones_changed();
    }
    xSemaphoreGive(s_mutex);
    return true;
}

bool radar_settings_set_zone_movement_threshold_cms(uint8_t zone_index,
                                                     uint16_t threshold_cms)
{
    if (zone_index >= RADAR_ZONE_COUNT ||
        !movement_threshold_is_valid(threshold_cms)) {
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_zones.zones[zone_index].movement_threshold_cms != threshold_cms) {
        s_zones.zones[zone_index].movement_threshold_cms = threshold_cms;
        mark_zones_changed();
    }
    xSemaphoreGive(s_mutex);
    return true;
}

bool radar_settings_set_zone_presence_delay_s(uint8_t zone_index,
                                               uint16_t delay_s)
{
    if (zone_index >= RADAR_ZONE_COUNT || !presence_delay_is_valid(delay_s)) {
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_zones.zones[zone_index].presence_delay_s != delay_s) {
        s_zones.zones[zone_index].presence_delay_s = delay_s;
        mark_zones_changed();
    }
    xSemaphoreGive(s_mutex);
    return true;
}
