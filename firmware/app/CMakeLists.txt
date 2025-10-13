
cmake_minimum_required(VERSION 3.20.0)

# Add devicetree bindings directory
list(APPEND DTS_ROOT ${CMAKE_SOURCE_DIR}/..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(doodle_bot)

project(app LANGUAGES C CXX)

# Choose between main app sources or test applications
if(CONFIG_LED_TEST)
    target_sources(app PRIVATE
        ${CMAKE_SOURCE_DIR}/../tests/led/led_control.cpp
        ${CMAKE_SOURCE_DIR}/../drivers/led/led_gpio.c
    )
elseif(CONFIG_BUZZER_TEST)
    target_sources(app PRIVATE
        ${CMAKE_SOURCE_DIR}/../tests/buzzer/buzzer_test.cpp
        ${CMAKE_SOURCE_DIR}/../drivers/buzzer/buzzer.c
    )
elseif(CONFIG_PWM_BUZZER_TEST)
    target_sources(app PRIVATE
        ${CMAKE_SOURCE_DIR}/../tests/buzzer/pwm_buzzer_test.cpp
    )
elseif(CONFIG_PWM_BUZZER_DRIVER_TEST)
    target_sources(app PRIVATE
        ${CMAKE_SOURCE_DIR}/../tests/buzzer/pwm_buzzer_driver_test.cpp
        ${CMAKE_SOURCE_DIR}/../drivers/buzzer/pwm_buzzer.c
    )
elseif(CONFIG_SERVO_TEST)
    target_sources(app PRIVATE
        ${CMAKE_SOURCE_DIR}/../tests/servo/servo_control.cpp
        ${CMAKE_SOURCE_DIR}/../drivers/servo/servo.c
    )
elseif(CONFIG_SERVO_PWM_TEST)
    target_sources(app PRIVATE
        ${CMAKE_SOURCE_DIR}/../pwm_test.cpp
    )
elseif(CONFIG_STEPPER_TEST)
    target_sources(app PRIVATE
        ${CMAKE_SOURCE_DIR}/../tests/stepper/stepper_control.cpp
        ${CMAKE_SOURCE_DIR}/../drivers/stepper/stepper.c
    )
else()
    file(GLOB DOODLE_BOT_SOURCES "src/*.c" "src/*.cpp")
    target_sources(app PRIVATE ${DOODLE_BOT_SOURCES})
endif()

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/../include)

# Add DT bindings for custom drivers
list(APPEND DTS_ROOT ${CMAKE_SOURCE_DIR}/..)

# Add custom servo driver directly to the application
if(CONFIG_SERVO)
    target_sources(app PRIVATE ${CMAKE_SOURCE_DIR}/../drivers/servo/servo.c)
    target_include_directories(app PRIVATE ${CMAKE_SOURCE_DIR}/../drivers/servo)
endif()

# Add specific flags for x86 platform to handle floating point
if(CONFIG_X86)
    target_compile_options(app PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-m32 -mno-sse -mno-mmx>
    )
endif()
