/**
 * Simple PWM Buzzer Test - One Test Only
 */

#include <zephyr/kernel.h>
#include "pwm_buzzer.h"

int main(void)
{
    printk("PWM Buzzer Test\n");
    
    // Initialize buzzer
    pwm_buzzer_init();
    
    while (1) {
        // Simple toggle test with quiet volume
        printk("Toggle ON\n");
        pwm_buzzer_toggle();
        k_sleep(K_MSEC(500));
        
        printk("Toggle OFF\n");
        pwm_buzzer_toggle();
        k_sleep(K_MSEC(1000));
    }
    
    return 0;
}