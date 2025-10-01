/*
 * Copyright (c) 2025 Doodle Bot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Set servo to specific angle with gradual movement (power-safe)
 * @param angle Desired angle in degrees (0-180)
 * @return 0 on success, negative on error
 */
int servo_set_angle_gradual(uint16_t angle);

/**
 * Demo function showing continuous servo sweep
 * This function runs indefinitely
 */
void servo_demo_sweep(void);

/**
 * Power-safe servo test function - used in main.cpp
 * Performs a test sequence: 45° -> 135° -> 90°
 */
void servo_power_safe_test(void);
void servo_continuous_sweep(void);
void servo_request_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_CONTROL_H */