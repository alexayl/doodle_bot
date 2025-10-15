/*
 * Simple stepper motor test with GPIO pulse counting
 */

#include <zephyr/kernel.h>
#include "stepper.h"

int main(void)
{
    printk("Simple Stepper Test - GPIO Pulse Counting\n");

    // Initialize stepper
    int ret = stepper_init();
    if (ret != 0) {
        printk("ERROR: Stepper initialization failed: %d\n", ret);
        return -1;
    }
    
    printk("Stepper motor initialized successfully\n");
    stepper_enable(STEPPER_LEFT);

    int cycle = 0;
    while (1) {
        cycle++;
        printk("\n=== Cycle %d ===\n", cycle);
        
        // Reset counters before movement
        stepper_reset_counters();
        
        // 90° Forward
        printk("90° Forward (expecting 1600 GPIO pulses for 800 steps)...\n");
        stepper_start_counting(90.0f);  // Enable step counting
        stepper_set_velocity(STEPPER_LEFT, 90.0f);
        k_sleep(K_SECONDS(1));
        stepper_set_velocity(STEPPER_LEFT, 0.0f);
        stepper_stop_counting();  // Stop step counting
        
        uint32_t forward_pulses = stepper_get_gpio_pulse_count();
        uint32_t forward_steps = stepper_get_step_count();
        printk("Forward: %d GPIO pulses, %d steps counted\n", forward_pulses, forward_steps);
        
        k_sleep(K_SECONDS(1));

        // 90° Backward  
        printk("90° Backward (expecting another 1600 GPIO pulses)...\n");
        uint32_t before_backward = stepper_get_gpio_pulse_count();
        stepper_start_counting(-90.0f);  // Enable step counting for backward
        stepper_set_velocity(STEPPER_LEFT, -90.0f);
        k_sleep(K_SECONDS(1));
        stepper_set_velocity(STEPPER_LEFT, 0.0f);
        stepper_stop_counting();  // Stop step counting
        
        uint32_t total_pulses = stepper_get_gpio_pulse_count();
        uint32_t total_steps = stepper_get_total_steps();
        uint32_t backward_pulses = total_pulses - before_backward;
        
        printk("Backward: %d GPIO pulses, Total: %d pulses, %d steps\n", 
               backward_pulses, total_pulses, total_steps);
        
        printk("Cycle %d complete\n", cycle);
        k_sleep(K_SECONDS(2));
        
        // Stop after a few cycles to avoid running indefinitely
        if (cycle >= 6) {
            printk("Test complete - stopping\n");
            stepper_disable(STEPPER_LEFT);
            break;
        }
    }

    return 0;
}