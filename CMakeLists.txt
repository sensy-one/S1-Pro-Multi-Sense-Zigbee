cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET "esp32c6" CACHE STRING "Target chip")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(s1_pro_multi_sense_zigbee VERSION 1.1.1)
