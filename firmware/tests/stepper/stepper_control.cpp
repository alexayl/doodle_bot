/*
 * Simple Stepper Test - Alternating 90 Steps
 */

#include <zephyr/kernel.h>
#include "stepper.h"

int main(void)
{
    printk("=== STEPPER TEST - ALTERNATING 90 STEPS ===\n");

    // Initialize and enable both motors
    stepper_init();
    stepper_enable(STEPPER_LEFT);
    stepper_enable(STEPPER_RIGHT);
    k_sleep(K_SECONDS(1));

    while (1) {
        // Forward sequence
        printk("\n>>> LEFT MOTOR FORWARD 90 steps\n");
        stepper_set_velocity(STEPPER_LEFT, 90.0f);
        k_sleep(K_MSEC(1000));  // 90 degrees at 180°/s = 0.5s
        stepper_set_velocity(STEPPER_LEFT, 0.0f);
        k_sleep(K_MSEC(1000));
        
        printk(">>> RIGHT MOTOR FORWARD 90 steps\n");
        stepper_set_velocity(STEPPER_RIGHT, 90.0f);
        k_sleep(K_MSEC(1000));  // 90 degrees at 180°/s = 0.5s
        stepper_set_velocity(STEPPER_RIGHT, 0.0f);
        k_sleep(K_MSEC(1000));
        
        printk(">>> LEFT MOTOR FORWARD 90 steps (2nd time)\n");
        stepper_set_velocity(STEPPER_LEFT, 90.0f);
        k_sleep(K_MSEC(1000));
        stepper_set_velocity(STEPPER_LEFT, 0.0f);
        k_sleep(K_MSEC(1000));
        
        printk(">>> RIGHT MOTOR FORWARD 90 steps (2nd time)\n");
        stepper_set_velocity(STEPPER_RIGHT, 90.0f);
        k_sleep(K_MSEC(1000));
        stepper_set_velocity(STEPPER_RIGHT, 0.0f);
        k_sleep(K_MSEC(1000));

        // Backward sequence
        printk(">>> LEFT MOTOR BACKWARD 90 steps\n");
        stepper_set_velocity(STEPPER_LEFT, -90.0f);
        k_sleep(K_MSEC(1000));
        stepper_set_velocity(STEPPER_LEFT, 0.0f);
        k_sleep(K_MSEC(1000));
        
        printk(">>> RIGHT MOTOR BACKWARD 90 steps\n");
        stepper_set_velocity(STEPPER_RIGHT, -90.0f);
        k_sleep(K_MSEC(1000));
        stepper_set_velocity(STEPPER_RIGHT, 0.0f);
        k_sleep(K_MSEC(1000));
        
        printk(">>> LEFT MOTOR BACKWARD 90 steps (2nd time)\n");
        stepper_set_velocity(STEPPER_LEFT, -90.0f);
        k_sleep(K_MSEC(1000));
        stepper_set_velocity(STEPPER_LEFT, 0.0f);
        k_sleep(K_MSEC(1000));
        
        printk(">>> RIGHT MOTOR BACKWARD 90 steps (2nd time)\n");
        stepper_set_velocity(STEPPER_RIGHT, -90.0f);
        k_sleep(K_MSEC(1000));
        stepper_set_velocity(STEPPER_RIGHT, 0.0f);
        k_sleep(K_MSEC(1000));
        
        printk(">>> CYCLE COMPLETE - Both motors back to start\n");
        k_sleep(K_SECONDS(1));
    }

    return 0;
}