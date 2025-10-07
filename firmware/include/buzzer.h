/*
 * Copyright (c) 2025 Doodle Bot Project
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BUZZER_H
#define BUZZER_H

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Simple GPIO Buzzer Driver
 * 
 * Provides basic on/off control for active buzzers
 * connected to GPIO pins.
 */

/**
 * @brief Initialize the buzzer driver
 * @return 0 on success, negative error code on failure
 */
int buzzer_init(void);

/**
 * @brief Turn buzzer on
 * @return 0 on success, negative error code on failure
 */
int buzzer_on(void);

/**
 * @brief Turn buzzer off
 * @return 0 on success, negative error code on failure
 */
int buzzer_off(void);

/**
 * @brief Toggle buzzer state
 * @return 0 on success, negative error code on failure
 */
int buzzer_toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_H */