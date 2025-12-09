
cmake_minimum_required(VERSION 3.20.0)

# Add devicetree bindings directory
list(APPEND DTS_ROOT ${CMAKE_SOURCE_DIR}/..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(doodle_bot)

# Main application sources
file(GLOB DOODLE_BOT_SOURCES "src/*.c" "src/*.cpp")
target_sources(app PRIVATE ${DOODLE_BOT_SOURCES})

# Include driver sources for main application
target_sources(app PRIVATE 
    ${CMAKE_SOURCE_DIR}/../drivers/stepper/a4988.c
    ${CMAKE_SOURCE_DIR}/../drivers/buzzer/pwm_buzzer.c
    ${CMAKE_SOURCE_DIR}/../drivers/led/simple_led.c
    ${CMAKE_SOURCE_DIR}/../drivers/servo/servo.c
)

# Include directories
target_include_directories(app PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${CMAKE_SOURCE_DIR}/../include
    ${CMAKE_SOURCE_DIR}/../drivers/servo
    ${CMAKE_SOURCE_DIR}/../drivers/stepper
    ${CMAKE_SOURCE_DIR}/../drivers/buzzer
    ${CMAKE_SOURCE_DIR}/../drivers/led
    ${ZEPHYR_BASE}/drivers/stepper  # For step_dir_stepper_common.h
)

# Add specific flags for x86 platform to handle floating point
if(CONFIG_X86)
    target_compile_options(app PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-m32 -mno-sse -mno-mmx>
    )
endif()
