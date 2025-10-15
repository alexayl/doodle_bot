/**
 * Simple GPIO LED Driver
 * 
 * Direct GPIO control for LED on/off functionality
 * Much simpler than Zephyr's LED subsystem
 * Compatible with led_control test interface
 */

#ifndef SIMPLE_LED_H
#define SIMPLE_LED_H

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * LED Command enumeration for compatibility with led_control test
 */
typedef enum {
    LED_OFF = 0,
    LED_ON = 1
} led_command_t;

/**
 * Initialize the LED driver
 * @return 0 on success, negative error code on failure
 */
int simple_led_init(void);

/**
 * LED Driver Function - receives ON/OFF commands from core firmware
 * Compatible with led_control test expectations
 * @param command LED_ON or LED_OFF
 * @return 0 on success, negative error code on failure
 */
int led_driver_set(led_command_t command);

/**
 * LED Driver Function - receives ON/OFF commands from core firmware
 * Compatible with led_control test expectations
 * @param command LED_ON or LED_OFF
 * @return 0 on success, negative error code on failure
 */
int led_driver_set(led_command_t command);

/**
 * Turn LED on
 * @return 0 on success, negative error code on failure
 */
int simple_led_on(void);

/**
 * Turn LED off
 * @return 0 on success, negative error code on failure
 */
int simple_led_off(void);

/**
 * Toggle LED state
 * @return 0 on success, negative error code on failure
 */
int simple_led_toggle(void);

/**
 * Check if LED driver is ready
 * @return true if ready, false otherwise
 */
bool simple_led_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* SIMPLE_LED_H */