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
#include <zephyr/drivers/led.h>

/* LED device and configuration */
#define LED_NODE DT_NODELABEL(led1)
#define LED_DEVICE DEVICE_DT_GET(DT_PARENT(LED_NODE))
#define LED_NUM DT_NODE_CHILD_IDX(LED_NODE)

/**
 * LED Driver Abstraction Layer
 * Core firmware tells driver "ON" or "OFF"
 */
typedef enum {
    LED_OFF = 0,
    LED_ON = 1
} led_command_t;

/**
 * LED Driver Function - receives ON/OFF commands from core firmware
 */
int led_driver_set(led_command_t command)
{
    uint8_t brightness = (command == LED_ON) ? 1 : 0;
    int ret = led_set_brightness(LED_DEVICE, LED_NUM, brightness);
    
    /* Only print when LED is turned ON */
    if (command == LED_ON && ret == 0) {
        printk("LED ON\n");
    }
    
    return ret;
}

int main(void)
{
    printk("LED Driver Test starting...\n");
    
    /* Check LED device is ready */
    if (!device_is_ready(LED_DEVICE)) {
        printk("ERROR: LED device not ready!\n");
        printk("Check devicetree: led1 node with valid GPIO configuration\n");
        return -1;
    }
    
    printk("LED device initialized successfully\n");
        return -1;
    }
    
    printk("GPIO14 LED driver ready\n");
    
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