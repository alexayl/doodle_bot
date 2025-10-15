/**
 * Simple stepper motor driver for 1.8° motors with A4988 driver
 * Uses PWM hardware for precise step pulse generation (similar to servo driver)
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
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
static const struct pwm_dt_spec step_pwm = PWM_DT_SPEC_GET(STEPPER_NODE);
static const struct gpio_dt_spec dir_pin = GPIO_DT_SPEC_GET(STEPPER_NODE, dir_gpios);
static const struct gpio_dt_spec en_pin = GPIO_DT_SPEC_GET(STEPPER_NODE, en_gpios);

/* Constants for 1.8° stepper with 16x microstepping */
#define STEPS_PER_REV 200
#define MICROSTEPS 16
#define TOTAL_STEPS_PER_REV (STEPS_PER_REV * MICROSTEPS)  // 3200

/* Back to theoretical values */
#define DEG_TO_STEPS (TOTAL_STEPS_PER_REV / 360.0f)  // 8.89 steps/deg

/* GPIO pulse counting for debugging */
static uint32_t gpio_pulse_count = 0;

/* State */
static bool enabled = false;
static bool initialized = false;

/* PWM cycle tracking for precise debugging */
static uint32_t expected_pwm_cycles = 0;
static uint32_t movement_start_time = 0;
static uint32_t current_step_freq_hz = 0;
static uint32_t movement_step_freq_hz = 0;  /* Frequency during movement for calculations */
static bool step_counting_enabled = false;

/* Legacy counters for compatibility */
static uint32_t total_step_count = 0;
static uint32_t current_movement_steps = 0;
static uint32_t expected_steps = 0;

int stepper_init(void)
{
    if (initialized) {
        return 0;
    }

    /* Check PWM device ready */
    if (!device_is_ready(step_pwm.dev)) {
        LOG_ERR("PWM device not ready");
        return -ENODEV;
    }

    /* Check GPIO ready */
    if (!device_is_ready(dir_pin.port) || !device_is_ready(en_pin.port)) {
        LOG_ERR("GPIO devices not ready");
        return -ENODEV;
    }

    /* Configure GPIO pins */
    gpio_pin_configure_dt(&dir_pin, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&en_pin, GPIO_OUTPUT_INACTIVE);  // Start disabled (LOW = disabled)

    initialized = true;
    LOG_INF("Stepper initialized: 1.8° motor, 16x microstepping, PWM-based");

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
    /* Stop PWM */
    pwm_set_dt(&step_pwm, 0, 0);
    current_step_freq_hz = 0;
    
    gpio_pin_set_dt(&en_pin, 0);  // LOW = disabled
    enabled = false;

    return 0;
}

int stepper_set_velocity(enum stepper_motor motor, float velocity_deg_s)
{
    /* Convert deg/s to steps/s: velocity * 8.89 */
    uint32_t step_freq_hz = (uint32_t)(fabsf(velocity_deg_s) * DEG_TO_STEPS);
    bool clockwise = (velocity_deg_s >= 0);

    /* Set direction */
    gpio_pin_set_dt(&dir_pin, clockwise ? 1 : 0);

    /* Start/stop stepping */
    if (step_freq_hz > 0 && enabled) {
        /* 
         * PWM generates step pulses directly:
         * - Period = 1/step_freq_hz seconds
         * - 50% duty cycle creates rising and falling edges for A4988
         */
        uint32_t period_ns = 1000000000 / step_freq_hz;  /* nanoseconds for high precision */
        uint32_t pulse_ns = period_ns / 2;                /* 50% duty cycle */
        
        current_step_freq_hz = step_freq_hz;
        
        /* Debug: print the calculated values */
        LOG_INF("PWM Stepper: %.1f°/s, Steps: %u Hz, Period: %u ns, Pulse: %u ns", 
                (double)velocity_deg_s, step_freq_hz, period_ns, pulse_ns);
        
        /* Set PWM with precise nanosecond timing */
        int ret = pwm_set_dt(&step_pwm, period_ns, pulse_ns);
        if (ret < 0) {
            LOG_ERR("Failed to set PWM: %d", ret);
            return ret;
        }
        
        /* Track movement for precise cycle counting */
        if (step_counting_enabled) {
            movement_start_time = k_uptime_get_32();
            movement_step_freq_hz = step_freq_hz;  /* Save frequency for calculations */
        }
    } else {
        /* Stop PWM */
        pwm_set_dt(&step_pwm, 0, 0);
        current_step_freq_hz = 0;
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

/* Precise PWM cycle counting functions */
void stepper_start_counting(float expected_degrees)
{
    expected_steps = (uint32_t)(fabsf(expected_degrees) * DEG_TO_STEPS);
    expected_pwm_cycles = expected_steps; /* 1 PWM cycle = 1 step for stepper */
    movement_start_time = k_uptime_get_32();
    step_counting_enabled = true;
    
    LOG_INF("PWM counting started - expecting %d steps (%d PWM cycles) for %.1f degrees", 
            expected_steps, expected_pwm_cycles, (double)expected_degrees);
}

void stepper_stop_counting(void)
{
    step_counting_enabled = false;
    
    if (movement_step_freq_hz > 0 && movement_start_time > 0) {
        /* Calculate actual PWM cycles generated based on elapsed time */
        uint32_t elapsed_ms = k_uptime_get_32() - movement_start_time;
        uint32_t actual_pwm_cycles = (uint32_t)((movement_step_freq_hz * (uint64_t)elapsed_ms) / 1000);
        
        /* Update legacy counters for compatibility */
        current_movement_steps = actual_pwm_cycles;
        total_step_count += actual_pwm_cycles;
        gpio_pulse_count = actual_pwm_cycles * 2; /* 2 edges per PWM cycle */
        
        LOG_INF("PWM counting stopped - %d ms elapsed, %d Hz freq, generated %d PWM cycles (expected %d)", 
                elapsed_ms, movement_step_freq_hz, actual_pwm_cycles, expected_pwm_cycles);
        
        if (actual_pwm_cycles != expected_pwm_cycles) {
            float actual_degrees = (float)actual_pwm_cycles / DEG_TO_STEPS;
            float expected_degrees = (float)expected_pwm_cycles / DEG_TO_STEPS;
            float accuracy_percent = (float)actual_pwm_cycles / expected_pwm_cycles * 100.0f;
            
            LOG_WRN("PWM cycle mismatch! Expected %.1f°, got %.1f° (%.2f%% accuracy)", 
                    (double)expected_degrees, (double)actual_degrees, (double)accuracy_percent);
        } else {
            LOG_INF("Perfect PWM accuracy: %d cycles generated as expected", actual_pwm_cycles);
        }
    } else {
        LOG_WRN("PWM counting stopped but no movement detected (freq=%d, start_time=%d)", 
                movement_step_freq_hz, movement_start_time);
    }
}

uint32_t stepper_get_step_count(void)
{
    return current_movement_steps;
}

uint32_t stepper_get_total_steps(void)
{
    return total_step_count;
}

void stepper_reset_counters(void)
{
    current_movement_steps = 0;
    total_step_count = 0;
    gpio_pulse_count = 0;
    expected_pwm_cycles = 0;
    movement_start_time = 0;
    movement_step_freq_hz = 0;
    LOG_INF("PWM counters reset");
}

uint32_t stepper_get_gpio_pulse_count(void)
{
    return gpio_pulse_count;
}