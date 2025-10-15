/*
 * Simple servo test using alias-based initialization
 */

#include <zephyr/kernel.h>
#include "servo.h"

int main(void)
{
    printk("Servo Alias Test\n");

    // Initialize servos by alias
    const struct device *servo_eraser = servo_init_by_alias("servo0");
    const struct device *servo_marker = servo_init_by_alias("servo1");
    
    if (!servo_eraser) {
        printk("ERROR: Failed to initialize servo0 (eraser)\n");
        return -1;
    }
    
    if (!servo_marker) {
        printk("ERROR: Failed to initialize servo1 (marker)\n");
        return -1;
    }
    
    printk("Both servos initialized successfully!\n");

    int cycle = 0;
    while (1) {
        cycle++;
        printk("\n=== Cycle %d ===\n", cycle);
        
        // Test servo0 (eraser) - GPIO2
        printk("Moving servo0 (eraser) to 0°\n");
        servo_set_angle(servo_eraser, 0);
        k_sleep(K_MSEC(500));
        
        printk("Moving servo0 (eraser) to 90°\n");
        servo_set_angle(servo_eraser, 90);
        k_sleep(K_MSEC(500));
        
        printk("Moving servo0 (eraser) to 180°\n");
        servo_set_angle(servo_eraser, 180);
        k_sleep(K_MSEC(500));
        
        // Test servo1 (marker) - GPIO3
        printk("Moving servo1 (marker) to 0°\n");
        servo_set_angle(servo_marker, 0);
        k_sleep(K_MSEC(500));
        
        printk("Moving servo1 (marker) to 90°\n");
        servo_set_angle(servo_marker, 90);
        k_sleep(K_MSEC(500));
        
        printk("Moving servo1 (marker) to 180°\n");
        servo_set_angle(servo_marker, 180);
        k_sleep(K_MSEC(500));
        
        // Test using alias functions directly
        printk("Using alias functions:\n");
        servo_set_angle_by_alias("servo0", 45);
        servo_set_angle_by_alias("servo1", 135);
        k_sleep(K_MSEC(1000));
        
        printk("Cycle %d complete\n", cycle);
        k_sleep(K_SECONDS(2));
        
        // Stop after a few cycles
        if (cycle >= 3) {
            printk("Test complete - stopping\n");
            break;
        }
    }

    return 0;
}