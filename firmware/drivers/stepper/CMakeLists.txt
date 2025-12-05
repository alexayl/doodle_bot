# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_DOODLE_A4988_STEPPER a4988.c)
zephyr_include_directories(${ZEPHYR_BASE}/drivers/stepper)
