/*
 * Simple stepper motor test - one revolution
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "stepper.h"

LOG_MODULE_REGISTER(stepper_test, CONFIG_LOG_DEFAULT_LEVEL);

int main(void)
{
    LOG_INF("Simple Stepper Test - One Revolution");

    // Initialize and enable steppers
    stepper_init();
    stepper_enable(STEPPER_BOTH);

    while (1) {
        LOG_INF("Starting one revolution at 360 deg/s (1 rev/s)");
        stepper_set_velocity(STEPPER_BOTH, 360.0f);
        k_sleep(K_SECONDS(1));  // Run for 1 second = 1 revolution

        LOG_INF("Stop");
        stepper_set_velocity(STEPPER_BOTH, 0.0f);
        k_sleep(K_SECONDS(2));  // Wait 2 seconds before next revolution
    }

    return 0;
}