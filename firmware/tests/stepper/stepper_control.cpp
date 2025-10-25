/*
 * Dual stepper motor test - left and right motors
 */

#include <zephyr/kernel.h>
#include "stepper.h"

int main(void)
{
    printk("Dual Stepper Motor Test\n");
    printk("=======================\n");

    // Initialize stepper system
    int ret = stepper_init();
    if (ret != 0) {
        printk("ERROR: Stepper initialization failed: %d\n", ret);
        return -1;
    }
    
    printk("Stepper system initialized successfully\n");
    
    // Enable both motors
    printk("Enabling left stepper motor...\n");
    stepper_enable(STEPPER_LEFT);
    
    printk("Enabling right stepper motor...\n");
    stepper_enable(STEPPER_RIGHT);
    
    k_sleep(K_SECONDS(1));

    int cycle = 0;
    while (1) {
        cycle++;
        printk("\n=== Test Cycle %d ===\n", cycle);
        
        // Test 1: Left motor forward
        printk("1. Left motor: 90° forward\n");
        stepper_set_velocity(STEPPER_LEFT, 180.0f);  // 180 deg/s
        k_sleep(K_MSEC(500));  // 500ms = 90 degrees
        stepper_set_velocity(STEPPER_LEFT, 0.0f);
        k_sleep(K_SECONDS(1));
        
        // Test 2: Right motor forward  
        printk("2. Right motor: 90° forward\n");
        stepper_set_velocity(STEPPER_RIGHT, 180.0f);  // 180 deg/s
        k_sleep(K_MSEC(500));  // 500ms = 90 degrees
        stepper_set_velocity(STEPPER_RIGHT, 0.0f);
        k_sleep(K_SECONDS(1));
        
        // Test 3: Both motors forward together
        printk("3. Both motors: 90° forward together\n");
        stepper_set_velocity(STEPPER_BOTH, 180.0f);  // Both at 180 deg/s
        k_sleep(K_MSEC(500));  // 500ms = 90 degrees
        stepper_set_velocity(STEPPER_BOTH, 0.0f);
        k_sleep(K_SECONDS(1));
        
        // Test 4: Opposite directions
        printk("4. Opposite directions: Left forward, Right backward\n");
        stepper_set_velocity(STEPPER_LEFT, 180.0f);   // Left forward
        stepper_set_velocity(STEPPER_RIGHT, -180.0f); // Right backward
        k_sleep(K_MSEC(500));  // 500ms = 90 degrees
        stepper_set_velocity(STEPPER_BOTH, 0.0f);     // Stop both
        k_sleep(K_SECONDS(1));
        
        // Test 5: Return to home (reverse previous moves)
        printk("5. Return to start position\n");
        stepper_set_velocity(STEPPER_LEFT, -180.0f);  // Left backward
        stepper_set_velocity(STEPPER_RIGHT, 180.0f);  // Right forward  
        k_sleep(K_MSEC(500));  // 500ms = 90 degrees
        stepper_set_velocity(STEPPER_BOTH, 0.0f);     // Stop both
        k_sleep(K_MSEC(500));
        
        stepper_set_velocity(STEPPER_BOTH, -180.0f);  // Both backward
        k_sleep(K_MSEC(500));  // 500ms = 90 degrees
        stepper_set_velocity(STEPPER_BOTH, 0.0f);     // Stop both
        k_sleep(K_MSEC(500));
        
        stepper_set_velocity(STEPPER_BOTH, -180.0f);  // Both backward  
        k_sleep(K_MSEC(500));  // 500ms = 90 degrees
        stepper_set_velocity(STEPPER_BOTH, 0.0f);     // Stop both
        
        printk("Cycle %d complete - both motors back to start\n", cycle);
        k_sleep(K_SECONDS(2));
        
        // Stop after a few cycles
        if (cycle >= 3) {
            printk("\nDual stepper test complete!\n");
            stepper_disable(STEPPER_BOTH);
            break;
        }
    }

    return 0;
}