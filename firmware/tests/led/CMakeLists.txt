cmake_minimum_required(VERSION 3.20.0)

# Tell Zephyr where to find device tree bindings
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Use the device tree overlay from the main app directory
set(DTC_OVERLAY_FILE "../../app/boards/esp32s3_devkitc_procpu.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_led)

# Include LED driver source directly
target_sources(app PRIVATE led_control.cpp)
target_sources(app PRIVATE ../../drivers/led/simple_led.c)
target_include_directories(app PRIVATE ../../include)
target_include_directories(app PRIVATE ../../drivers/led)