/*
 * Copyright (c) 2025 Doodle Bot Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "buzzer.h"

LOG_MODULE_REGISTER(buzzer, CONFIG_LOG_DEFAULT_LEVEL);

/* Buzzer GPIO configuration from devicetree */
#define BUZZER_NODE DT_ALIAS(buzzer0)

#if !DT_NODE_EXISTS(BUZZER_NODE)
#error "Buzzer devicetree alias 'buzzer0' not found"
#endif

static const struct gpio_dt_spec buzzer_gpio = GPIO_DT_SPEC_GET(BUZZER_NODE, gpios);
static bool buzzer_state = false;
static bool buzzer_initialized = false;

int buzzer_init(void)
{
    if (buzzer_initialized) {
        return 0;
    }

    /* Check if GPIO device is ready */
    if (!device_is_ready(buzzer_gpio.port)) {
        LOG_ERR("Buzzer GPIO device not ready");
        return -ENODEV;
    }

    /* Configure GPIO pin as output, initially inactive */
    int ret = gpio_pin_configure_dt(&buzzer_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure buzzer GPIO pin: %d", ret);
        return ret;
    }

    buzzer_state = false;
    buzzer_initialized = true;
    
    LOG_INF("Buzzer driver initialized on GPIO %d", buzzer_gpio.pin);
    return 0;
}

int buzzer_on(void)
{
    if (!buzzer_initialized) {
        int ret = buzzer_init();
        if (ret < 0) {
            return ret;
        }
    }

    int ret = gpio_pin_set_dt(&buzzer_gpio, 1);
    if (ret < 0) {
        return ret;
    }

    buzzer_state = true;
    return 0;
}

int buzzer_off(void)
{
    if (!buzzer_initialized) {
        int ret = buzzer_init();
        if (ret < 0) {
            return ret;
        }
    }

    int ret = gpio_pin_set_dt(&buzzer_gpio, 0);
    if (ret < 0) {
        return ret;
    }

    buzzer_state = false;
    return 0;
}

int buzzer_toggle(void)
{
    return buzzer_state ? buzzer_off() : buzzer_on();
}