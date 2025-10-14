/**
 * Simple stepper motor driver for 1.8° motors with A4988 driver
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <math.h>
#include "stepper.h"

LOG_MODULE_REGISTER(stepper, CONFIG_LOG_DEFAULT_LEVEL);

/* Stepper motor from devicetree */
#define STEPPER_NODE DT_NODELABEL(stepper_left)

#if !DT_NODE_EXISTS(STEPPER_NODE)
#error "Stepper devicetree node 'stepper_left' not found"
#endif

/* GPIO pins */
static const struct gpio_dt_spec step_pin = GPIO_DT_SPEC_GET(STEPPER_NODE, step_gpios);
static const struct gpio_dt_spec dir_pin = GPIO_DT_SPEC_GET(STEPPER_NODE, dir_gpios);
static const struct gpio_dt_spec en_pin = GPIO_DT_SPEC_GET(STEPPER_NODE, en_gpios);

/* Constants for 1.8° stepper with 16x microstepping */
#define STEPS_PER_REV 200
#define MICROSTEPS 16
#define TOTAL_STEPS_PER_REV (STEPS_PER_REV * MICROSTEPS)  // 3200
#define DEG_TO_STEPS (TOTAL_STEPS_PER_REV / 360.0f)       // 8.89 steps/deg

/* State */
static bool enabled = false;
static bool initialized = false;
static struct k_timer step_timer;

/* Step generation */
static void step_callback(struct k_timer *timer)
{
    if (enabled) {
        gpio_pin_toggle_dt(&step_pin);
    }
}

int stepper_init(void)
{
    if (initialized) {
        return 0;
    }

    /* Check GPIO ready */
    if (!device_is_ready(step_pin.port)) {
        return -ENODEV;
    }

    /* Configure pins */
    gpio_pin_configure_dt(&step_pin, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&dir_pin, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&en_pin, GPIO_OUTPUT_INACTIVE);  // Start disabled (LOW = disabled)

    /* Initialize timer */
    k_timer_init(&step_timer, step_callback, NULL);

    initialized = true;
    LOG_INF("Stepper initialized: 1.8° motor, 16x microstepping");

    return 0;
}

int stepper_enable(enum stepper_motor motor)
{
    if (!initialized) {
        stepper_init();
    }

    gpio_pin_set_dt(&en_pin, 1);  // HIGH = enabled (this A4988 board)
    enabled = true;
    


    return 0;
}

int stepper_disable(enum stepper_motor motor)
{
    k_timer_stop(&step_timer);
    gpio_pin_set_dt(&en_pin, 0);  // LOW = disabled
    enabled = false;

    return 0;
}

int stepper_set_velocity(enum stepper_motor motor, float velocity_deg_s)
{
    /* Convert deg/s to steps/s: velocity * 8.89 */
    uint32_t step_freq_hz = (uint32_t)(fabs(velocity_deg_s) * DEG_TO_STEPS);
    bool clockwise = (velocity_deg_s >= 0);

    /* Set direction */
    gpio_pin_set_dt(&dir_pin, clockwise ? 1 : 0);

    /* Start/stop stepping */
    if (step_freq_hz > 0 && enabled) {
        k_timeout_t period = K_USEC(500000 / step_freq_hz);
        k_timer_start(&step_timer, period, period);
    } else {
        k_timer_stop(&step_timer);
    }

    return 0;
}

int stepper_stop(enum stepper_motor motor)
{
    return stepper_set_velocity(motor, 0.0f);
}

bool stepper_is_enabled(enum stepper_motor motor)
{
    return enabled;
}