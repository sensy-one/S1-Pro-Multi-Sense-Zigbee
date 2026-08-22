#include "ltrf216a_sensor.h"

#include <stddef.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "ltrf216a_protocol.h"
#include "s1_pro_config.h"

static const char *TAG = "ltrf216a";

#define LTRF216A_REG_MAIN_CTRL 0x00U
#define LTRF216A_REG_MEAS_RES 0x04U
#define LTRF216A_REG_GAIN 0x05U
#define LTRF216A_REG_PART_ID 0x06U
#define LTRF216A_REG_ALS_STATUS 0x07U
#define LTRF216A_REG_ALS_DATA 0x0DU

#define LTRF216A_CTRL_STANDBY 0x00U
#define LTRF216A_CTRL_ALS_ACTIVE 0x02U
#define LTRF216A_STATUS_NEW_DATA 0x08U
#define LTRF216A_EXPECTED_PART_NUMBER 0xB0U

/* Datasheet defaults: 18-bit resolution, 100 ms, 3x gain. */
#define LTRF216A_ALS_MEAS_RES 0x22U
#define LTRF216A_ALS_GAIN_REGISTER 0x01U
#define LTRF216A_ALS_GAIN 3U
#define LTRF216A_ALS_INTEGRATION_TIME_MS 100U
#define LTRF216A_ALS_READY_TIMEOUT_MS 250U
#define LTRF216A_STATUS_POLL_MS 20U
#define LTRF216A_MAX_CONSECUTIVE_ERRORS 3U

static portMUX_TYPE s_snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static ltrf216a_snapshot_t s_snapshot;
static i2c_master_dev_handle_t s_device;

static void record_error(void)
{
    portENTER_CRITICAL(&s_snapshot_lock);
    ++s_snapshot.errors;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

static esp_err_t write_register(uint8_t register_address, uint8_t value)
{
    const uint8_t transaction[] = {register_address, value};
    return i2c_master_transmit(s_device, transaction, sizeof(transaction),
                               S1_PRO_I2C_TIMEOUT_MS);
}

static esp_err_t read_registers(uint8_t register_address, uint8_t *data,
                                size_t data_size)
{
    ESP_RETURN_ON_FALSE(data != NULL && data_size > 0U, ESP_ERR_INVALID_ARG,
                        TAG, "Invalid register read buffer");
    return i2c_master_transmit_receive(s_device, &register_address, 1U, data,
                                       data_size, S1_PRO_I2C_TIMEOUT_MS);
}

static esp_err_t read_register(uint8_t register_address, uint8_t *value)
{
    return read_registers(register_address, value, 1U);
}

static esp_err_t configure_sensor(void)
{
    ESP_RETURN_ON_ERROR(
        write_register(LTRF216A_REG_MAIN_CTRL, LTRF216A_CTRL_STANDBY), TAG,
        "Unable to stop LTR-F216A conversion");
    ESP_RETURN_ON_ERROR(
        write_register(LTRF216A_REG_MEAS_RES, LTRF216A_ALS_MEAS_RES), TAG,
        "Unable to set LTR-F216A measurement resolution");
    ESP_RETURN_ON_ERROR(
        write_register(LTRF216A_REG_GAIN, LTRF216A_ALS_GAIN_REGISTER), TAG,
        "Unable to set LTR-F216A gain");

    uint8_t ignored_status = 0;
    ESP_RETURN_ON_ERROR(
        read_register(LTRF216A_REG_ALS_STATUS, &ignored_status), TAG,
        "Unable to clear LTR-F216A status");
    ESP_RETURN_ON_ERROR(
        write_register(LTRF216A_REG_MAIN_CTRL, LTRF216A_CTRL_ALS_ACTIVE), TAG,
        "Unable to start LTR-F216A conversion");
    return ESP_OK;
}

static esp_err_t wait_for_new_data(void)
{
    uint32_t elapsed_ms = 0;
    while (elapsed_ms < LTRF216A_ALS_READY_TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(LTRF216A_STATUS_POLL_MS));
        elapsed_ms += LTRF216A_STATUS_POLL_MS;
        uint8_t status = 0;
        ESP_RETURN_ON_ERROR(
            read_register(LTRF216A_REG_ALS_STATUS, &status), TAG,
            "LTR-F216A status read failed");
        if ((status & LTRF216A_STATUS_NEW_DATA) != 0U) {
            return ESP_OK;
        }
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t read_illuminance(uint32_t *counts)
{
    ESP_RETURN_ON_FALSE(counts != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "Invalid illuminance output");
    ESP_RETURN_ON_ERROR(wait_for_new_data(), TAG,
                        "LTR-F216A ALS conversion timed out");

    uint8_t data[LTRF216A_DATA_SIZE];
    ESP_RETURN_ON_ERROR(
        read_registers(LTRF216A_REG_ALS_DATA, data, sizeof(data)), TAG,
        "LTR-F216A ALS data read failed");
    *counts = ltrf216a_decode_20bit(data);
    return ESP_OK;
}

static esp_err_t initialize_sensor(void)
{
    uint8_t part_id = 0;
    ESP_RETURN_ON_ERROR(read_register(LTRF216A_REG_PART_ID, &part_id), TAG,
                        "LTR-F216A identity read failed");
    ESP_RETURN_ON_FALSE(
        (part_id & 0xF0U) == LTRF216A_EXPECTED_PART_NUMBER,
        ESP_ERR_INVALID_RESPONSE, TAG,
        "Unexpected LTR-F216A part ID 0x%02x", part_id);
    ESP_RETURN_ON_ERROR(configure_sensor(), TAG,
                        "LTR-F216A configuration failed");

    portENTER_CRITICAL(&s_snapshot_lock);
    s_snapshot.part_id = part_id;
    portEXIT_CRITICAL(&s_snapshot_lock);
    return ESP_OK;
}

static void store_measurement(uint32_t counts)
{
    const uint32_t illuminance_centilux = ltrf216a_counts_to_centilux(
        counts, LTRF216A_ALS_GAIN, LTRF216A_ALS_INTEGRATION_TIME_MS);

    portENTER_CRITICAL(&s_snapshot_lock);
    s_snapshot.received_at_us = esp_timer_get_time();
    s_snapshot.illuminance_centilux = illuminance_centilux;
    s_snapshot.valid = true;
    ++s_snapshot.measurement_count;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

static bool detect_sensor(void)
{
    return i2c_master_probe(s1_pro_i2c_bus_get(),
                            S1_PRO_LTRF216A_I2C_ADDRESS,
                            S1_PRO_I2C_TIMEOUT_MS) == ESP_OK;
}

static void ltrf216a_task(void *argument)
{
    (void)argument;
    bool device_added = false;

    while (true) {
        if (!detect_sensor()) {
            vTaskDelay(pdMS_TO_TICKS(S1_PRO_LTRF216A_DETECT_RETRY_MS));
            continue;
        }
        if (!device_added) {
            const esp_err_t status = s1_pro_i2c_add_device(
                S1_PRO_LTRF216A_I2C_ADDRESS, &s_device);
            if (status != ESP_OK) {
                record_error();
                vTaskDelay(pdMS_TO_TICKS(S1_PRO_LTRF216A_DETECT_RETRY_MS));
                continue;
            }
            device_added = true;
        }
        if (initialize_sensor() != ESP_OK) {
            record_error();
            vTaskDelay(pdMS_TO_TICKS(S1_PRO_LTRF216A_DETECT_RETRY_MS));
            continue;
        }

        uint8_t consecutive_errors = 0;
        TickType_t last_wake = xTaskGetTickCount();
        while (consecutive_errors < LTRF216A_MAX_CONSECUTIVE_ERRORS) {
            uint32_t counts = 0;
            const esp_err_t status = read_illuminance(&counts);
            if (status == ESP_OK) {
                consecutive_errors = 0;
                store_measurement(counts);
            } else {
                ++consecutive_errors;
                record_error();
            }
            vTaskDelayUntil(
                &last_wake,
                pdMS_TO_TICKS(S1_PRO_LTRF216A_MEASUREMENT_PERIOD_MS));
        }
    }
}

void ltrf216a_sensor_get_snapshot(ltrf216a_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_snapshot_lock);
    *snapshot = s_snapshot;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

void ltrf216a_sensor_start(void)
{
    const BaseType_t created =
        xTaskCreate(ltrf216a_task, "ltrf216a", 4096, NULL, 4, NULL);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
