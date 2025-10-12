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
    buzzer_init();
    
    /* Check if buzzer is ready */
    if (!buzzer_is_ready()) {
        printk("ERROR: Buzzer driver not ready! Check devicetree configuration.\n");
        printk("Expected: buzzer0 alias pointing to valid GPIO device\n");
        return -1;
    }
    
    printk("Buzzer initialized successfully - starting toggle test\n");
    
    while (1) {
        /* Toggle buzzer state */
        buzzer_toggle();
        /* Wait 500ms */
        k_sleep(K_MSEC(500));
    }
    
    return 0;
}