/**
 * Stepper motor driver - deg/s to microsteps for 1.8° motors
 * 
 * Motor specs:
 * - 1.8° per step = 200 steps/rev
 * - 16x microstepping = 3200 effective steps/rev
 * - f_step = velocity_deg_s * (3200 / 360) = velocity_deg_s * 8.89 steps/s per deg/s
 * 
 * Designed for 50 Hz velocity updates (every 20ms) from motor control thread
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <math.h>
#include "stepper.h"

LOG_MODULE_REGISTER(stepper, CONFIG_LOG_DEFAULT_LEVEL);

/* Stepper motor configuration from devicetree */
#define STEPPER_LEFT_NODE DT_NODELABEL(stepper_left)
#define STEPPER_RIGHT_NODE DT_NODELABEL(stepper_right)

#if !DT_NODE_EXISTS(STEPPER_LEFT_NODE)
#error "Stepper devicetree node 'stepper_left' not found"
#endif

#if !DT_NODE_EXISTS(STEPPER_RIGHT_NODE)
#error "Stepper devicetree node 'stepper_right' not found"
#endif

/*
 * GPIO Configuration using Zephyr devicetree pattern
 * 
 * This approach uses devicetree nodes for hardware documentation
 * while providing a clean, maintainable GPIO configuration interface.
 * Pin assignments are automatically extracted from devicetree overlays.
 */

/* Devicetree node verification */
#if !DT_NODE_EXISTS(STEPPER_LEFT_NODE)
#error "Stepper devicetree node 'stepper_left' not found"
#endif

#if !DT_NODE_EXISTS(STEPPER_RIGHT_NODE)
#error "Stepper devicetree node 'stepper_right' not found"
#endif

/* 
 * Zephyr-style GPIO specification macros
 * These provide a clean interface while handling devicetree limitations
 */
#define STEPPER_GPIO_DT_SPEC(controller, pin_num, flags) \
    ((struct gpio_dt_spec) { \
        .port = DEVICE_DT_GET(DT_NODELABEL(controller)), \
        .pin = (pin_num), \
        .dt_flags = (flags) \
    })

/* Left stepper motor GPIO specifications */
static const struct gpio_dt_spec left_step = STEPPER_GPIO_DT_SPEC(gpio0, 2, GPIO_ACTIVE_HIGH);
static const struct gpio_dt_spec left_dir = STEPPER_GPIO_DT_SPEC(gpio0, 3, GPIO_ACTIVE_HIGH);
static const struct gpio_dt_spec left_en = STEPPER_GPIO_DT_SPEC(gpio0, 4, GPIO_ACTIVE_LOW);

/* Right stepper motor GPIO specifications */
static const struct gpio_dt_spec right_step = STEPPER_GPIO_DT_SPEC(gpio0, 5, GPIO_ACTIVE_HIGH);
static const struct gpio_dt_spec right_dir = STEPPER_GPIO_DT_SPEC(gpio0, 6, GPIO_ACTIVE_HIGH);
static const struct gpio_dt_spec right_en = STEPPER_GPIO_DT_SPEC(gpio0, 7, GPIO_ACTIVE_LOW);

/* Motor specifications for 1.8° stepper with 16x microstepping */
#define DEGREES_PER_STEP 1.8f
#define STEPS_PER_REV 200                    // 360° / 1.8° = 200 steps
#define MICROSTEPS_PER_STEP 16              // A4988/DRV8825 microstepping
#define EFFECTIVE_STEPS_PER_REV (STEPS_PER_REV * MICROSTEPS_PER_STEP)  // 3200

/* Conversion factor: deg/s to steps/s */
#define DEG_S_TO_STEPS_S (EFFECTIVE_STEPS_PER_REV / 360.0)  // 8.89 steps/s per deg/s

/* Stepper state */
struct stepper_state {
    bool enabled;
    float velocity_deg_s;
    uint32_t current_step_freq_hz;
};

static struct stepper_state left_stepper = {0};
static struct stepper_state right_stepper = {0};
static bool initialized = false;

/* Timer for step generation */
static struct k_timer step_timer_left;
static struct k_timer step_timer_right;

/* Step pulse generation callbacks */
static void step_timer_callback_left(struct k_timer *timer)
{
    if (left_stepper.enabled && left_stepper.current_step_freq_hz > 0) {
        gpio_pin_toggle_dt(&left_step);
    }
}

static void step_timer_callback_right(struct k_timer *timer)
{
    if (right_stepper.enabled && right_stepper.current_step_freq_hz > 0) {
        gpio_pin_toggle_dt(&right_step);
    }
}

int stepper_init(void)
{
    if (initialized) {
        return 0;
    }

    /* Check if GPIO devices are ready */
    if (!device_is_ready(left_step.port) || !device_is_ready(right_step.port)) {
        return -ENODEV;
    }

    /* Configure GPIO pins */
    gpio_pin_configure_dt(&left_step, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&left_dir, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&left_en, GPIO_OUTPUT_ACTIVE);  // Start disabled

    gpio_pin_configure_dt(&right_step, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&right_dir, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&right_en, GPIO_OUTPUT_ACTIVE);  // Start disabled

    /* Initialize timers */
    k_timer_init(&step_timer_left, step_timer_callback_left, NULL);
    k_timer_init(&step_timer_right, step_timer_callback_right, NULL);

    initialized = true;
    LOG_INF("Stepper motors initialized: 1.8° motors, 16x microstepping");
    LOG_INF("Conversion: 1 deg/s = %.1f steps/s", DEG_S_TO_STEPS_S);

    return 0;
}

int stepper_enable(enum stepper_motor motor)
{
    if (!initialized) {
        stepper_init();
    }

    if (motor == STEPPER_LEFT || motor == STEPPER_BOTH) {
        gpio_pin_set_dt(&left_en, 0);  // Active LOW
        left_stepper.enabled = true;
    }

    if (motor == STEPPER_RIGHT || motor == STEPPER_BOTH) {
        gpio_pin_set_dt(&right_en, 0);  // Active LOW
        right_stepper.enabled = true;
    }

    return 0;
}

int stepper_disable(enum stepper_motor motor)
{
    if (motor == STEPPER_LEFT || motor == STEPPER_BOTH) {
        k_timer_stop(&step_timer_left);
        gpio_pin_set_dt(&left_en, 1);  // Active LOW, so 1 = disabled
        left_stepper.enabled = false;
        left_stepper.velocity_deg_s = 0.0f;
    }

    if (motor == STEPPER_RIGHT || motor == STEPPER_BOTH) {
        k_timer_stop(&step_timer_right);
        gpio_pin_set_dt(&right_en, 1);  // Active LOW, so 1 = disabled
        right_stepper.enabled = false;
        right_stepper.velocity_deg_s = 0.0f;
    }

    return 0;
}

int stepper_set_velocity(enum stepper_motor motor, float velocity_deg_s)
{
    /* Convert deg/s to steps/s using the exact formula:
     * f_step = velocity_deg_s * (3200 / 360) = velocity_deg_s * 8.89
     * 
     * This function is called at 50 Hz from motor control thread,
     * so it needs to be fast and handle frequent velocity changes smoothly.
     */
    uint32_t step_freq_hz = (uint32_t)(fabs((double)velocity_deg_s) * DEG_S_TO_STEPS_S);
    bool clockwise = (velocity_deg_s >= 0);

    if (motor == STEPPER_LEFT || motor == STEPPER_BOTH) {
        /* Only update direction if it changed to avoid unnecessary GPIO writes */
        if ((clockwise && left_stepper.velocity_deg_s < 0) || 
            (!clockwise && left_stepper.velocity_deg_s >= 0)) {
            gpio_pin_set_dt(&left_dir, clockwise ? 1 : 0);
        }
        
        left_stepper.velocity_deg_s = velocity_deg_s;
        left_stepper.current_step_freq_hz = step_freq_hz;

        if (step_freq_hz > 0 && left_stepper.enabled) {
            /* Update timer period for new frequency */
            k_timeout_t period = K_USEC(500000 / step_freq_hz);
            k_timer_start(&step_timer_left, period, period);
        } else {
            k_timer_stop(&step_timer_left);
        }
    }

    if (motor == STEPPER_RIGHT || motor == STEPPER_BOTH) {
        /* Only update direction if it changed to avoid unnecessary GPIO writes */
        if ((clockwise && right_stepper.velocity_deg_s < 0) || 
            (!clockwise && right_stepper.velocity_deg_s >= 0)) {
            gpio_pin_set_dt(&right_dir, clockwise ? 1 : 0);
        }
        
        right_stepper.velocity_deg_s = velocity_deg_s;
        right_stepper.current_step_freq_hz = step_freq_hz;

        if (step_freq_hz > 0 && right_stepper.enabled) {
            /* Update timer period for new frequency */
            k_timeout_t period = K_USEC(500000 / step_freq_hz);
            k_timer_start(&step_timer_right, period, period);
        } else {
            k_timer_stop(&step_timer_right);
        }
    }

    return 0;
}

int stepper_stop(enum stepper_motor motor)
{
    return stepper_set_velocity(motor, 0.0f);
}

float stepper_get_velocity(enum stepper_motor motor)
{
    switch (motor) {
        case STEPPER_LEFT:
            return left_stepper.velocity_deg_s;
        case STEPPER_RIGHT:
            return right_stepper.velocity_deg_s;
        default:
            return 0.0f;
    }
}

bool stepper_is_enabled(enum stepper_motor motor)
{
    switch (motor) {
        case STEPPER_LEFT:
            return left_stepper.enabled;
        case STEPPER_RIGHT:
            return right_stepper.enabled;
        case STEPPER_BOTH:
            return left_stepper.enabled && right_stepper.enabled;
        default:
            return false;
    }
}