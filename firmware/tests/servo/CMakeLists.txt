cmake_minimum_required(VERSION 3.20.0)

# Add devicetree bindings directory - critical for driver setup
list(APPEND DTS_ROOT ${CMAKE_SOURCE_DIR}/../..)

# Use the device tree overlay from the main app directory
set(DTC_OVERLAY_FILE "../../app/boards/esp32s3_devkitc_procpu.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_servo)

# Include servo test and driver sources
target_sources(app PRIVATE 
    servo_control.cpp
    ../../drivers/servo/servo.c
)
target_include_directories(app PRIVATE ../../include)