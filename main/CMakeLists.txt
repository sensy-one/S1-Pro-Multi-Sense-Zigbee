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
        "zigbee_ota.c"
    INCLUDE_DIRS "." "${CMAKE_CURRENT_BINARY_DIR}"
    REQUIRES
        driver
        bsec2
        esp_app_format
        app_update
        esp_timer
        ld2450_parser
        led_strip
        ltrf216a_protocol
        nvs_flash
        scd4x_protocol
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)

idf_build_get_property(s1_pro_version PROJECT_VER)
if(NOT s1_pro_version MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)$")
    message(FATAL_ERROR "Zigbee OTA requires a major.minor.patch firmware version")
endif()
foreach(part RANGE 1 3)
    if(CMAKE_MATCH_${part} GREATER 255)
        message(FATAL_ERROR "Zigbee OTA version components must fit in one byte")
    endif()
endforeach()
math(EXPR S1_PRO_OTA_FILE_VERSION
    "(${CMAKE_MATCH_1} << 24) | (${CMAKE_MATCH_2} << 16) | (${CMAKE_MATCH_3} << 8)"
)
configure_file(ota_version.h.in "${CMAKE_CURRENT_BINARY_DIR}/ota_version.h" @ONLY)
