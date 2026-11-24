cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET "esp32c6" CACHE STRING "Target chip")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(s1_pro_multi_sense_zigbee VERSION 1.2.1)

if(NOT CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE)
    message(FATAL_ERROR "Enable CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE in sdkconfig for Zigbee OTA")
endif()

idf_build_get_property(s1_pro_python PYTHON)
add_custom_target(zigbee_ota_image ALL
    COMMAND "${s1_pro_python}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/package_firmware.py"
        --build-dir "${CMAKE_BINARY_DIR}"
    DEPENDS app bootloader partition_table_bin blank_ota_data
    VERBATIM
)
