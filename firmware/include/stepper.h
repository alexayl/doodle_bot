/*
 * Copyright (c) 2025 Doodle Bot Project
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef STEPPER_H
#define STEPPER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Stepper Motor Driver
 * 
 * Simple stepper motor control that converts deg/s input to microsteps.
 * Optimized for 50 Hz velocity updates from motor control thread.
 */

/**
 * @brief Stepper motor selection
 */
enum stepper_motor {
    STEPPER_LEFT = 0,
    STEPPER_RIGHT = 1,
    STEPPER_BOTH = 2
};

/**
 * @brief Initialize stepper motor drivers
 * @return 0 on success, negative error code on failure
 */
int stepper_init(void);

/**
 * @brief Enable stepper motor(s)
 * @param motor Which motor(s) to enable
 * @return 0 on success, negative error code on failure
 */
int stepper_enable(enum stepper_motor motor);

/**
 * @brief Disable stepper motor(s)
 * @param motor Which motor(s) to disable
 * @return 0 on success, negative error code on failure
 */
int stepper_disable(enum stepper_motor motor);

/**
 * @brief Set stepper motor velocity
 * 
 * Updates motor velocity. Designed to be called at 50 Hz from motor control thread.
 * Optimized for frequent updates - only changes direction pin when needed.
 * 
 * @param motor Which motor to control
 * @param velocity_deg_s Target velocity in degrees/second
 * @return 0 on success, negative error code on failure
 */
int stepper_set_velocity(enum stepper_motor motor, float velocity_deg_s);

/**
 * @brief Stop stepper motor
 * @param motor Which motor(s) to stop
 * @return 0 on success, negative error code on failure
 */
int stepper_stop(enum stepper_motor motor);

/**
 * @brief Get current velocity
 * @param motor Which motor to query
 * @return Current velocity in degrees/second
 */
float stepper_get_velocity(enum stepper_motor motor);

/**
 * @brief Check if motor is enabled
 * @param motor Which motor to check
 * @return true if enabled, false if disabled
 */
bool stepper_is_enabled(enum stepper_motor motor);

/* Convenience macros for common operations */
#define stepper_enable_all()    stepper_enable(STEPPER_BOTH)
#define stepper_disable_all()   stepper_disable(STEPPER_BOTH)
#define stepper_stop_all()      stepper_stop(STEPPER_BOTH)

/* Common velocity presets (deg/s) for 1.8° motors */
#define STEPPER_VELOCITY_SLOW   (90.0f)    /* 0.25 rev/s */
#define STEPPER_VELOCITY_MEDIUM (180.0f)   /* 0.5 rev/s */
#define STEPPER_VELOCITY_FAST   (360.0f)   /* 1.0 rev/s */

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_H */