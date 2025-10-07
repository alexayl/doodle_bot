# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

# Out-of-tree drivers for existing driver classes
add_subdirectory_ifdef(CONFIG_SENSOR sensor)
add_subdirectory_ifdef(CONFIG_SERVO servo)
add_subdirectory_ifdef(CONFIG_LED_GPIO led)
add_subdirectory_ifdef(CONFIG_BUZZER buzzer)
