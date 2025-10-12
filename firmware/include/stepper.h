/*
 * Copyright (c) 2025 Doodle Bot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Stepper motor driver for A4988/DRV8825 controllers
 * @brief Simple velocity control with deg/s input for 1.8° stepper motors
 * 
 * This driver is optimized for:
 * - 1.8° stepper motors (200 steps/rev) 
 * - 16x microstepping = 3200 effective steps/rev
 * - 50 Hz velocity updates from motor control thread
 * 
 * Conversion: velocity_deg_s * 8.89 = steps/s
 */

#ifndef STEPPER_H
#define STEPPER_H

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Stepper motor selection
 */
enum stepper_motor {
    STEPPER_LEFT,   /**< Left stepper motor */
    STEPPER_RIGHT,  /**< Right stepper motor */
    STEPPER_BOTH    /**< Both stepper motors */
};

/**
 * @brief Initialize stepper motor driver
 * 
 * Configures GPIO pins and timers for stepper control.
 * Safe to call multiple times - will only initialize once.
 * 
 * @return 0 on success, negative errno on error
 */
int stepper_init(void);

/**
 * @brief Enable stepper motor(s)
 * 
 * Enables power to the specified stepper motor(s).
 * Motors are disabled by default to save power.
 * 
 * @param motor Which motor(s) to enable
 * @return 0 on success, negative errno on error
 */
int stepper_enable(enum stepper_motor motor);

/**
 * @brief Disable stepper motor(s)
 * 
 * Disables power to the specified stepper motor(s) and stops any movement.
 * 
 * @param motor Which motor(s) to disable
 * @return 0 on success, negative errno on error
 */
int stepper_disable(enum stepper_motor motor);

/**
 * @brief Set stepper motor velocity
 * 
 * Sets the velocity in degrees per second. Designed for 50 Hz updates.
 * 
 * Conversion formula: f_step = velocity_deg_s * 8.89 steps/s
 * 
 * @param motor Which motor(s) to control
 * @param velocity_deg_s Velocity in degrees per second
 *                       - Positive = clockwise
 *                       - Negative = counter-clockwise  
 *                       - Zero = stop
 * @return 0 on success, negative errno on error
 */
int stepper_set_velocity(enum stepper_motor motor, float velocity_deg_s);

/**
 * @brief Stop stepper motor(s)
 * 
 * Convenience function to set velocity to zero.
 * 
 * @param motor Which motor(s) to stop
 * @return 0 on success, negative errno on error
 */
int stepper_stop(enum stepper_motor motor);

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_H */