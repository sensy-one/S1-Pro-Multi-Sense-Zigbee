idf_component_register(
    SRCS
        "actuator_state.c"
        "app_main.c"
        "bme688_sensor.cpp"
        "buzzer.c"
        "factory_reset.c"
        "i2c_bus.c"
        "ltrf216a_sensor.c"
        "ld2450_command_protocol.c"
        "presence_delay.c"
        "radar_holding.c"
        "radar_settings.c"
        "radar_transform.c"
        "radar_uart.c"
        "radar_zones.c"
        "rgb_led.c"
        "scd40_sensor.c"
        "scd40_settings.c"
        "system_control.c"
        "zigbee_device.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        bsec2
        esp_app_format
        esp_timer
        ld2450_parser
        led_strip
        ltrf216a_protocol
        nvs_flash
        scd4x_protocol
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
