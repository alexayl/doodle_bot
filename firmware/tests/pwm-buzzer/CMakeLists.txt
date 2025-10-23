cmake_minimum_required(VERSION 3.20.0)

# Tell Zephyr where to find device tree bindings
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Use the device tree overlay from the main app directory
set(DTC_OVERLAY_FILE "../../app/boards/esp32s3_devkitc_procpu.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_pwm_buzzer)

# Include PWM buzzer driver source directly
target_sources(app PRIVATE pwm_buzzer_driver_test.cpp)
target_sources(app PRIVATE ../../drivers/buzzer/pwm_buzzer.c)
target_include_directories(app PRIVATE ../../include)
target_include_directories(app PRIVATE ../../drivers/buzzer)