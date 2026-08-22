#include "platform/esp_zigbee_platform.h"
#include "actuator_state.h"
#include "bme688_sensor.h"
#include "buzzer.h"
#include "factory_reset.h"
#include "i2c_bus.h"
#include "ltrf216a_sensor.h"
#include "nvs_flash.h"
#include "radar_uart.h"
#include "radar_settings.h"
#include "rgb_led.h"
#include "scd40_sensor.h"
#include "scd40_settings.h"
#include "system_control.h"
#include "zigbee_device.h"

void app_main(void)
{
    esp_err_t nvs_status = nvs_flash_init();
    if (nvs_status == ESP_ERR_NVS_NO_FREE_PAGES || nvs_status == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(nvs_status);
    }

    esp_zb_platform_config_t platform_config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&platform_config));

    actuator_state_init();
    radar_settings_init();
    scd40_settings_init();
    ESP_ERROR_CHECK(s1_pro_i2c_bus_init());
    rgb_led_init();
    buzzer_init();
    radar_uart_start();
    bme688_sensor_start();
    scd40_sensor_start();
    ltrf216a_sensor_start();
    system_control_start();
    zigbee_device_start();
    factory_reset_button_start();
}
