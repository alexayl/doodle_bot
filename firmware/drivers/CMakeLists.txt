# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_SERVO servo)
add_subdirectory_ifdef(CONFIG_LOCAL_LED_GPIO led)
add_subdirectory_ifdef(CONFIG_BUZZER buzzer)
add_subdirectory_ifdef(CONFIG_DOODLE_A4988_STEPPER stepper)
