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

/* Per-motor direction inversion */
#define INVERT_LEFT_DIRECTION true   // Left motor is physically backwards
#define INVERT_RIGHT_DIRECTION false // Right motor is correct

/* Stepper motor devicetree nodes */
#define STEPPER_LEFT_NODE  DT_NODELABEL(stepper_left)
#define STEPPER_RIGHT_NODE DT_NODELABEL(stepper_right)

#if !DT_NODE_EXISTS(STEPPER_LEFT_NODE)
#error "Stepper devicetree node 'stepper_left' not found"
#endif

/* GPIO pins for left stepper (primary) */
static const struct pwm_dt_spec step_pwm = PWM_DT_SPEC_GET(STEPPER_LEFT_NODE);
static const struct gpio_dt_spec dir_pin = GPIO_DT_SPEC_GET(STEPPER_LEFT_NODE, dir_gpios);
static const struct gpio_dt_spec en_pin = GPIO_DT_SPEC_GET(STEPPER_LEFT_NODE, en_gpios);

/* Helper functions for motor aliasing */
static int stepper_get_motor_pins(enum stepper_motor motor, 
                                 const struct pwm_dt_spec **pwm,
                                 const struct gpio_dt_spec **dir,
                                 const struct gpio_dt_spec **en)
{
    static const struct pwm_dt_spec right_step_pwm = PWM_DT_SPEC_GET(STEPPER_RIGHT_NODE);
    static const struct gpio_dt_spec right_dir_pin = GPIO_DT_SPEC_GET(STEPPER_RIGHT_NODE, dir_gpios);
    static const struct gpio_dt_spec right_en_pin = GPIO_DT_SPEC_GET(STEPPER_RIGHT_NODE, en_gpios);
    
    switch (motor) {
        case STEPPER_LEFT:
            *pwm = &step_pwm;
            *dir = &dir_pin;
            *en = &en_pin;
            return 0;
            
        case STEPPER_RIGHT:
#if DT_NODE_EXISTS(STEPPER_RIGHT_NODE) && DT_NODE_HAS_STATUS(STEPPER_RIGHT_NODE, okay)
            *pwm = &right_step_pwm;
            *dir = &right_dir_pin;
            *en = &right_en_pin;
            return 0;
#else
            LOG_ERR("Right stepper not available in devicetree");
            return -ENODEV;
#endif
            
        default:
            LOG_ERR("Invalid stepper motor: %d", motor);
            return -EINVAL;
    }
}

/* Constants for 1.8° stepper with 16x microstepping */
#define STEPS_PER_REV 200
#define MICROSTEPS 16
#define TOTAL_STEPS_PER_REV (STEPS_PER_REV * MICROSTEPS)  // 3200

/* Back to theoretical values */
#define DEG_TO_STEPS (TOTAL_STEPS_PER_REV / 360.0f)  // 8.89 steps/deg

/* State tracking - simplified for now, can be expanded per-motor later */
static bool initialized = false;
static bool left_enabled = false;
static bool right_enabled = false;

/* GPIO pulse counting for debugging */
static uint32_t gpio_pulse_count = 0;

/* PWM cycle tracking for precise debugging */
static uint32_t expected_pwm_cycles = 0;
static uint32_t movement_start_time = 0;
static uint32_t movement_step_freq_hz = 0;  /* Frequency during movement for calculations */
static bool step_counting_enabled = false;
static uint32_t current_movement_steps = 0;
static uint32_t total_step_count = 0;
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

    /* Check GPIO ready for both steppers */
    if (!device_is_ready(dir_pin.port) || !device_is_ready(en_pin.port)) {
        LOG_ERR("Left stepper GPIO devices not ready");
        return -ENODEV;
    }

    /* Configure LEFT stepper GPIO pins */
    gpio_pin_configure_dt(&dir_pin, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&en_pin, GPIO_OUTPUT_INACTIVE);

#if DT_NODE_EXISTS(STEPPER_RIGHT_NODE) && DT_NODE_HAS_STATUS(STEPPER_RIGHT_NODE, okay)
    /* Configure RIGHT stepper GPIO pins */
    static const struct gpio_dt_spec right_dir_pin = GPIO_DT_SPEC_GET(STEPPER_RIGHT_NODE, dir_gpios);
    static const struct gpio_dt_spec right_en_pin = GPIO_DT_SPEC_GET(STEPPER_RIGHT_NODE, en_gpios);
    
    if (!device_is_ready(right_dir_pin.port) || !device_is_ready(right_en_pin.port)) {
        LOG_ERR("Right stepper GPIO devices not ready");
        return -ENODEV;
    }
    
    gpio_pin_configure_dt(&right_dir_pin, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&right_en_pin, GPIO_OUTPUT_INACTIVE);
    LOG_INF("Right stepper GPIO pins configured");
#endif  

    initialized = true;
    LOG_INF("Stepper initialized: 1.8° motor, 16x microstepping, PWM-based");

    return 0;
}

int stepper_enable(enum stepper_motor motor)
{
    if (!initialized) {
        stepper_init();
    }
    
    const struct pwm_dt_spec *pwm;
    const struct gpio_dt_spec *dir;
    const struct gpio_dt_spec *en;
    
    if (motor == STEPPER_BOTH) {
        /* Enable both motors */
        stepper_enable(STEPPER_LEFT);
#if DT_NODE_EXISTS(STEPPER_RIGHT_NODE) && DT_NODE_HAS_STATUS(STEPPER_RIGHT_NODE, okay)
        stepper_enable(STEPPER_RIGHT);
#endif
        return 0;
    }
    
    int ret = stepper_get_motor_pins(motor, &pwm, &dir, &en);
    if (ret < 0) {
        return ret;
    }

    gpio_pin_set_dt(en, 1);  // HIGH = enabled (this A4988 board)
    
    if (motor == STEPPER_LEFT) {
        left_enabled = true;
    } else if (motor == STEPPER_RIGHT) {
        right_enabled = true;
    }

    return 0;
}

int stepper_disable(enum stepper_motor motor)
{
    const struct pwm_dt_spec *pwm;
    const struct gpio_dt_spec *dir;
    const struct gpio_dt_spec *en;
    
    if (motor == STEPPER_BOTH) {
        /* Disable both motors */
        stepper_disable(STEPPER_LEFT);
#if DT_NODE_EXISTS(STEPPER_RIGHT_NODE) && DT_NODE_HAS_STATUS(STEPPER_RIGHT_NODE, okay)
        stepper_disable(STEPPER_RIGHT);
#endif
        return 0;
    }
    
    int ret = stepper_get_motor_pins(motor, &pwm, &dir, &en);
    if (ret < 0) {
        return ret;
    }
    
    /* Stop PWM */
    pwm_set_dt(pwm, 0, 0);
    
    gpio_pin_set_dt(en, 0);  // LOW = disabled
    
    if (motor == STEPPER_LEFT) {
        left_enabled = false;
    } else if (motor == STEPPER_RIGHT) {
        right_enabled = false;
    }

    return 0;
}

int stepper_set_velocity(enum stepper_motor motor, float velocity_deg_s)
{
    const struct pwm_dt_spec *pwm;
    const struct gpio_dt_spec *dir;
    const struct gpio_dt_spec *en;
    
    if (motor == STEPPER_BOTH) {
        /* Set velocity for both motors */
        stepper_set_velocity(STEPPER_LEFT, velocity_deg_s);
#if DT_NODE_EXISTS(STEPPER_RIGHT_NODE) && DT_NODE_HAS_STATUS(STEPPER_RIGHT_NODE, okay)
        stepper_set_velocity(STEPPER_RIGHT, velocity_deg_s);
#endif
        return 0;
    }
    
    int ret = stepper_get_motor_pins(motor, &pwm, &dir, &en);
    if (ret < 0) {
        return ret;
    }
    
    /* Check if motor is enabled */
    bool motor_enabled = (motor == STEPPER_LEFT) ? left_enabled : right_enabled;
    
    /* Convert deg/s to steps/s: velocity * 8.89 */
    uint32_t step_freq_hz = (uint32_t)(fabsf(velocity_deg_s) * DEG_TO_STEPS);
    bool clockwise = (velocity_deg_s >= 0);

    LOG_INF("Motor %d: velocity=%.1f°/s, step_freq=%u Hz, clockwise=%s", 
            motor, (double)velocity_deg_s, step_freq_hz, clockwise ? "yes" : "no");

    /* Only set direction when actually moving (not when stopping) */
    if (step_freq_hz > 0) {
        /* Set direction with per-motor inversion */
        bool invert_this_motor = (motor == STEPPER_LEFT) ? INVERT_LEFT_DIRECTION : INVERT_RIGHT_DIRECTION;
        bool dir_pin_value = invert_this_motor ? !clockwise : clockwise;
        gpio_pin_set_dt(dir, dir_pin_value ? 1 : 0);
        
        LOG_INF("Motor %d Direction set: %s (pin=%d, inverted=%s)", motor, 
                clockwise ? "CLOCKWISE" : "COUNTERCLOCKWISE", dir_pin_value ? 1 : 0,
                invert_this_motor ? "yes" : "no");
    }
    
    /* Small delay to ensure direction is set before stepping starts */
    k_usleep(10);  /* 10 microseconds delay */

    /* Start/stop stepping with automatic power management */
    if (step_freq_hz > 0) {
        /* Enable motor only when moving */
        if (!motor_enabled) {
            gpio_pin_set_dt(en, 1);  // Enable motor for movement
            if (motor == STEPPER_LEFT) {
                left_enabled = true;
            } else if (motor == STEPPER_RIGHT) {
                right_enabled = true;
            }
            LOG_INF("Motor %d enabled for movement", motor);
        }
        
        /* 
         * PWM generates step pulses directly:
         * - Period = 1/step_freq_hz seconds
         * - 50% duty cycle creates rising and falling edges for A4988
         */
        uint32_t period_ns = 1000000000 / step_freq_hz;  /* nanoseconds for high precision */
        uint32_t pulse_ns = period_ns / 2;                /* 50% duty cycle */
        
        /* Debug: print the calculated values */
        LOG_INF("PWM Stepper %d: %.1f°/s, Steps: %u Hz, Period: %u ns, Pulse: %u ns", 
                motor, (double)velocity_deg_s, step_freq_hz, period_ns, pulse_ns);
        
        /* Set PWM with precise nanosecond timing */
        ret = pwm_set_dt(pwm, period_ns, pulse_ns);
        if (ret < 0) {
            LOG_ERR("Failed to set PWM for motor %d: %d", motor, ret);
            return ret;
        }
        
        /* Track movement for precise cycle counting */
        if (step_counting_enabled) {
            movement_start_time = k_uptime_get_32();
            movement_step_freq_hz = step_freq_hz;  /* Save frequency for calculations */
        }
    } else {
        /* Stop PWM and disable motor to prevent heating */
        pwm_set_dt(pwm, 0, 0);
        
        /* Disable motor when stopped to prevent overheating */
        if (motor_enabled) {
            gpio_pin_set_dt(en, 0);  // Disable motor when stopped
            if (motor == STEPPER_LEFT) {
                left_enabled = false;
            } else if (motor == STEPPER_RIGHT) {
                right_enabled = false;
            }
            LOG_INF("Motor %d disabled to prevent heating", motor);
        }
    }

    return 0;
}

int stepper_stop(enum stepper_motor motor)
{
    return stepper_set_velocity(motor, 0.0f);
}

bool stepper_is_enabled(enum stepper_motor motor)
{
    if (motor == STEPPER_LEFT) {
        return left_enabled;
    } else if (motor == STEPPER_RIGHT) {
        return right_enabled;
    } else if (motor == STEPPER_BOTH) {
        return left_enabled && right_enabled;
    }
    return false;
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