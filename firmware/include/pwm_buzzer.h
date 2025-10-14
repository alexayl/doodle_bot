/**
 * PWM Buzzer Driver Header
 * - Volume controlled passive buzzer driver using PWM
 * - Supports frequency and volume control
 * - Built on Zephyr PWM API
 */

#pragma once

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize PWM buzzer driver
 * @return 0 on success, negative error code on failure
 */
int pwm_buzzer_init(void);

/**
 * Check if PWM buzzer is ready for use
 * @return true if ready, false otherwise
 */
bool pwm_buzzer_is_ready(void);

/**
 * Set buzzer frequency
 * @param frequency_hz Frequency in Hz (0 = off)
 * @return 0 on success, negative error code on failure
 */
int pwm_buzzer_set_frequency(uint16_t frequency_hz);

/**
 * Set buzzer volume
 * @param volume_percent Volume percentage (0-100%)
 * @return 0 on success, negative error code on failure
 */
int pwm_buzzer_set_volume(uint8_t volume_percent);

/**
 * Update buzzer with current frequency and volume settings
 * @return 0 on success, negative error code on failure
 */
int pwm_buzzer_update(void);

/**
 * Turn buzzer on with current settings
 * @return 0 on success, negative error code on failure
 */
int pwm_buzzer_on(void);

/**
 * Turn buzzer off
 * @return 0 on success, negative error code on failure
 */
int pwm_buzzer_off(void);

/**
 * Toggle buzzer state
 * @return 0 on success, negative error code on failure
 */
int pwm_buzzer_toggle(void);

/**
 * Play a beep with specified parameters (blocking)
 * @param frequency_hz Frequency in Hz
 * @param volume_percent Volume percentage (0-100%)
 * @param duration_ms Duration in milliseconds
 * @return 0 on success, negative error code on failure
 */
int pwm_buzzer_beep(uint16_t frequency_hz, uint8_t volume_percent, uint32_t duration_ms);

/**
 * Get current frequency setting
 * @return Current frequency in Hz
 */
uint16_t pwm_buzzer_get_frequency(void);

/**
 * Get current volume setting
 * @return Current volume percentage (0-100%)
 */
uint8_t pwm_buzzer_get_volume(void);

/**
 * Check if buzzer is currently active
 * @return true if active, false if off
 */
bool pwm_buzzer_is_active(void);

#ifdef __cplusplus
}
#endif