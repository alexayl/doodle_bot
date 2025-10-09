/**
 * Simple Zephyr GPIO buzzer driver
 * - Uses Zephyr GPIO API for basic on/off control
 * - Exposes simple buzzer control functions
 * - Minimal implementation for active buzzers
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include "buzzer.h"

LOG_MODULE_REGISTER(buzzer, CONFIG_LOG_DEFAULT_LEVEL);

/* Buzzer GPIO configuration from devicetree */
#define BUZZER_NODE DT_ALIAS(buzzer0)

#if !DT_NODE_EXISTS(BUZZER_NODE)
#error "Buzzer devicetree alias 'buzzer0' not found"
#endif

static const struct gpio_dt_spec buzzer_gpio = GPIO_DT_SPEC_GET(BUZZER_NODE, gpios);

void buzzer_init(void)
{
    if (!device_is_ready(buzzer_gpio.port)) {
        LOG_ERR("Buzzer GPIO device not ready");
        return;
    }

    int ret = gpio_pin_configure_dt(&buzzer_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure buzzer GPIO pin: %d", ret);
        return;
    }

    LOG_INF("Buzzer initialized on GPIO pin %d", buzzer_gpio.pin);
}

void buzzer_on(void)
{
    int ret = gpio_pin_set_dt(&buzzer_gpio, 1);
    if (ret < 0) {
        LOG_WRN("Failed to turn buzzer on: %d", ret);
    }
}

void buzzer_off(void)
{
    int ret = gpio_pin_set_dt(&buzzer_gpio, 0);
    if (ret < 0) {
        LOG_WRN("Failed to turn buzzer off: %d", ret);
    }
}

void buzzer_toggle(void)
{
    int ret = gpio_pin_toggle_dt(&buzzer_gpio);
    if (ret < 0) {
        LOG_WRN("Failed to toggle buzzer: %d", ret);
    }
}