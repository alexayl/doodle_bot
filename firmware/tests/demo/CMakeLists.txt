cmake_minimum_required(VERSION 3.20.0)

# Tell Zephyr where to find device tree bindings
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Use the device tree overlay from the main app directory
set(DTC_OVERLAY_FILE "../../app/boards/esp32s3_devkitc_procpu.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(peripheral_demo)

# Include demo source and all driver sources
target_sources(app PRIVATE peripheral_demo.cpp)
target_sources(app PRIVATE ../../drivers/stepper/stepper.c)
target_sources(app PRIVATE ../../drivers/buzzer/pwm_buzzer.c)
target_sources(app PRIVATE ../../drivers/servo/servo.c)
target_sources(app PRIVATE ../../drivers/led/simple_led.c)

# Include directories
target_include_directories(app PRIVATE ../../include)
target_include_directories(app PRIVATE ../../drivers/stepper)
target_include_directories(app PRIVATE ../../drivers/buzzer)
target_include_directories(app PRIVATE ../../drivers/servo)
target_include_directories(app PRIVATE ../../drivers/led)