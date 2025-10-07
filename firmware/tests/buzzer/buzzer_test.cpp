/*
 * Copyright (c) 2025 Doodle Bot Project
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Simple Buzzer Driver Test - 20ms Toggle
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "buzzer.h"

int main(void)
{
    printk("Buzzer Driver Test - 20ms Toggle\n");
    
    /* Initialize buzzer */
    int ret = buzzer_init();
    if (ret < 0) {
        printk("ERROR: Buzzer initialization failed: %d\n", ret);
        return ret;
    }
    
    printk("GPIO15 buzzer ready - starting 20ms toggle\n");
    
    while (1) {
        /* Toggle buzzer state */
        buzzer_toggle();
        /* Wait 500ms */
        k_sleep(K_MSEC(500));
    }
    
    return 0;
}