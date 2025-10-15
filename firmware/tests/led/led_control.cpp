/*
 * Copyright (c) 2025 Doodle Bot Project
 * SPDX-License-Identifier: Apache-2.0
 * 
 * LED Driver Test with Abstraction Layer
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <simple_led.h>

/* LED device and configuration */
/* Using simple LED driver instead of Zephyr LED subsystem */

/**
 * LED Driver Abstraction Layer
 * Core firmware tells driver "ON" or "OFF"
 * The led_command_t enum and led_driver_set function 
 * are provided by simple_led.h
 */

int main(void)
{
    printk("LED Driver Test starting...\n");
    
    /* Initialize our simple LED driver */
    int ret = simple_led_init();
    if (ret != 0) {
        printk("ERROR: Simple LED driver initialization failed: %d\n", ret);
        printk("Check devicetree: led1 node with valid GPIO configuration\n");
        return -1;
    }
    
    printk("LED device initialized successfully\n");
    printk("GPIO11 LED driver ready\n");
    
    /* Core firmware loop - sends ON/OFF commands to LED driver */
    led_command_t command = LED_OFF;
    
    while (1) {
        /* Toggle command */
        command = (command == LED_OFF) ? LED_ON : LED_OFF;
        
        /* Send command to LED driver */
        led_driver_set(command);
        
        /* Wait 500ms */
        k_sleep(K_MSEC(500));
    }
    
    return 0;
}