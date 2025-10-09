/*
 * Copyright (c) 2025 Doodle Bot Project
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BUZZER_H
#define BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Simple GPIO Buzzer Driver
 * 
 * Minimal buzzer control for GPIO-connected active buzzers.
 */

/**
 * @brief Initialize the buzzer GPIO pin
 */
void buzzer_init(void);

/**
 * @brief Turn buzzer on
 */
void buzzer_on(void);

/**
 * @brief Turn buzzer off
 */
void buzzer_off(void);

/**
 * @brief Toggle buzzer state
 */
void buzzer_toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_H */