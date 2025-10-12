/*
 * Single stepper motor test - basic movement patterns
 */

#include <zephyr/kernel.h>
#include "stepper.h"

int main(void)
{
    printk("Single Stepper Test - Left Motor Only\n");

    // Initialize steppers
    int ret = stepper_init();
    if (ret != 0) {
        printk("ERROR: Stepper initialization failed: %d\n", ret);
        printk("Check devicetree: stepper_left node with valid GPIO configuration\n");
        return -1;
    }
    
    printk("Stepper motor initialized successfully\n");
    
    // Enable left stepper only
    stepper_enable(STEPPER_LEFT);

    int cycle = 0;
    while (1) {
        cycle++;
        printk("\n=== Cycle %d ===\n", cycle);
        
        // Forward rotation - 1 revolution per second
        printk("Forward: 360°/s (1 rev/s)\n");
        stepper_set_velocity(STEPPER_LEFT, 360.0f);
        k_sleep(K_SECONDS(2));  // 2 revolutions forward

        // Stop
        printk("Stop\n");
        stepper_set_velocity(STEPPER_LEFT, 0.0f);
        k_sleep(K_SECONDS(1));

        // Reverse rotation
        printk("Reverse: -180°/s (0.5 rev/s)\n");
        stepper_set_velocity(STEPPER_LEFT, -180.0f);
        k_sleep(K_SECONDS(2));  // 1 revolution backward

        // Stop and pause
        printk("Stop - End of cycle\n");
        stepper_set_velocity(STEPPER_LEFT, 0.0f);
        k_sleep(K_SECONDS(2));
    }

    return 0;
}