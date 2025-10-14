# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

# Out-of-tree drivers for existing driver classes
add_subdirectory_ifdef(CONFIG_SERVO servo)
add_subdirectory_ifdef(CONFIG_LOCAL_LED_GPIO led)
add_subdirectory_ifdef(CONFIG_BUZZER buzzer)
add_subdirectory_ifdef(CONFIG_DOODLE_STEPPER stepper)
