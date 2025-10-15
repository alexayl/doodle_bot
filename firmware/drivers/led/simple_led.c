/**
 * Simple GPIO LED Driver Implementation
 * 
 * Direct GPIO control for LED on GPIO11
 * Uses device tree configuration from ESP32-S3 overlay
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <simple_led.h>

/* LED GPIO configuration from device tree */
#define LED_NODE DT_NODELABEL(led1)
#define LED_GPIO_SPEC GPIO_DT_SPEC_GET(LED_NODE, gpios)

static const struct gpio_dt_spec led_gpio = LED_GPIO_SPEC;
static bool led_state = false;
static bool driver_ready = false;

int simple_led_init(void)
{
    int ret;

    /* Check if GPIO device is ready */
    if (!gpio_is_ready_dt(&led_gpio)) {
        printk("ERROR: LED GPIO device not ready\n");
        return -ENODEV;
    }

    /* Configure GPIO pin as output */
    ret = gpio_pin_configure_dt(&led_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("ERROR: Failed to configure LED GPIO pin: %d\n", ret);
        return ret;
    }

    /* Ensure LED starts in OFF state */
    ret = gpio_pin_set_dt(&led_gpio, 0);
    if (ret < 0) {
        printk("ERROR: Failed to set initial LED state: %d\n", ret);
        return ret;
    }

    led_state = false;
    driver_ready = true;
    
    printk("Simple LED driver initialized on GPIO%d\n", led_gpio.pin);
    return 0;
}

int simple_led_on(void)
{
    int ret;

    if (!driver_ready) {
        return -ENODEV;
    }

    ret = gpio_pin_set_dt(&led_gpio, 1);
    if (ret == 0) {
        led_state = true;
    }
    
    return ret;
}

int simple_led_off(void)
{
    int ret;

    if (!driver_ready) {
        return -ENODEV;
    }

    ret = gpio_pin_set_dt(&led_gpio, 0);
    if (ret == 0) {
        led_state = false;
    }
    
    return ret;
}

int simple_led_toggle(void)
{
    if (!driver_ready) {
        return -ENODEV;
    }

    return led_state ? simple_led_off() : simple_led_on();
}

bool simple_led_is_ready(void)
{
    return driver_ready;
}

/**
 * LED Driver Function - receives ON/OFF commands from core firmware
 * Compatible with led_control test expectations
 */
int led_driver_set(led_command_t command)
{
    int ret;
    
    if (!driver_ready) {
        return -ENODEV;
    }
    
    if (command == LED_ON) {
        ret = simple_led_on();
        /* Only print when LED is turned ON, matching led_control test behavior */
        if (ret == 0) {
            printk("LED ON\n");
        }
    } else {
        ret = simple_led_off();
    }
    
    return ret;
}