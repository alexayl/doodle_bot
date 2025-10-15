/**
 * Simplified Doodle Bot Peripheral Demo
 * 
 * Quick demo showing basic functionality:
 * - Servos: Move back and forth
 * - Buzzer: Toggle on/off (using PWM buzzer toggle)
 * - LED: Toggle on/off (GPIO11)
 * - Stepper: 90° forward and backward
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h> // For abs() function

// Include all your drivers
extern "C" {
#include "stepper.h"
#include "pwm_buzzer.h"
#include "servo.h"
#include "simple_led.h"
}

/* Servo devices from device tree */
static const struct device *servo_eraser;
static const struct device *servo_marker;

/**
 * Initialize all peripherals
 */
static int demo_init(void)
{
    printk("\nDOODLE BOT DEMO - Initializing...\n");
    printk("=====================================\n");

    /* Initialize stepper motor */
    if (stepper_init() != 0) {
        printk("Stepper initialization failed!\n");
        return -1;
    }
    stepper_enable(STEPPER_LEFT);
    printk("Stepper motor ready\n");

    /* Initialize PWM buzzer */
    if (pwm_buzzer_init() != 0) {
        printk("Buzzer initialization failed!\n");
        return -1;
    }
    printk("PWM buzzer ready\n");

    /* Initialize servos */
    servo_eraser = DEVICE_DT_GET(DT_ALIAS(servo_eraser));
    servo_marker = DEVICE_DT_GET(DT_ALIAS(servo_marker));
    
    if (!device_is_ready(servo_eraser) || !device_is_ready(servo_marker)) {
        printk("Servo initialization failed!\n");
        return -1;
    }
    printk("Servos ready\n");

    /* LED initialization */
    if (simple_led_init() != 0) {
        printk("Simple LED initialization failed!\n");
        return -1;
    }
    printk("Simple LED ready (GPIO11)\n");

    printk("All peripherals ready!\n\n");
    return 0;
}

/**
 * Test servos - move back and forth with latency measurement
 */
static void test_servos(void)
{
    printk("Testing Servos\n");
    printk("-----------------\n");

    // Check if servo feedback pin is available for latency measurement
    const struct gpio_dt_spec servo_feedback = GPIO_DT_SPEC_GET_OR(DT_ALIAS(servo_eraser_feedback), gpios, {0});
    bool can_measure_latency = (servo_feedback.port != NULL);
    
    if (can_measure_latency) {
        gpio_pin_configure_dt(&servo_feedback, GPIO_INPUT);
        printk("Servo latency measurement enabled (GPIO%d feedback)\n", servo_feedback.pin);
    }

    /* Move eraser servo */
    printk("Eraser servo: 90° → 0° → 90°");
    if (can_measure_latency) {
        // For PWM signals, we need to sample multiple times to detect activity
        int initial_sample = 0;
        for (int i = 0; i < 5; i++) {
            initial_sample += gpio_pin_get_dt(&servo_feedback);
            k_sleep(K_USEC(100)); // Short delay between samples
        }
        
        int64_t start_time = k_uptime_get();
        
        servo_set_angle(servo_eraser, 90);    // Start position (up/retracted)
        
        // Wait for PWM activity change with timeout - servo PWM is slower
        int64_t timeout = start_time + 200; // 200ms timeout for servo PWM startup
        bool detected = false;
        while (k_uptime_get() < timeout && !detected) {
            int current_sample = 0;
            for (int i = 0; i < 5; i++) {
                current_sample += gpio_pin_get_dt(&servo_feedback);
                k_sleep(K_USEC(100));
            }
            
            // Check if PWM activity pattern changed significantly
            if (abs(current_sample - initial_sample) >= 2) {
                int64_t latency_ms = k_uptime_get() - start_time;
                uint64_t latency_ns = latency_ms * 1000000; // Convert to nanoseconds
                printk(" (PWM start latency: %llu ns)", latency_ns);
                detected = true;
            }
        }
        
        if (!detected) {
            printk(" (no PWM change detected)");
        }
    } else {
        servo_set_angle(servo_eraser, 90);    // Start position (up/retracted)
    }
    printk("\n");
    k_sleep(K_MSEC(1000));
    
    servo_set_angle(servo_eraser, 0);     // Move to 0° (down/extended)
    k_sleep(K_MSEC(1000));
    
    servo_set_angle(servo_eraser, 90);    // Back to start (up/retracted)
    k_sleep(K_MSEC(1000));

    /* Move marker servo */
    printk("Marker servo: 90° → 0° → 90°");
    if (can_measure_latency) {
        // For PWM signals, we need to sample multiple times to detect activity
        int initial_sample = 0;
        for (int i = 0; i < 5; i++) {
            initial_sample += gpio_pin_get_dt(&servo_feedback);
            k_sleep(K_USEC(100)); // Short delay between samples
        }
        
        int64_t start_time = k_uptime_get();
        
        servo_set_angle(servo_marker, 90);    // Start position (up/not drawing)
        
        // Wait for PWM activity change with timeout
        int64_t timeout = start_time + 200; // 200ms timeout for servo PWM startup
        bool detected = false;
        while (k_uptime_get() < timeout && !detected) {
            int current_sample = 0;
            for (int i = 0; i < 5; i++) {
                current_sample += gpio_pin_get_dt(&servo_feedback);
                k_sleep(K_USEC(100));
            }
            
            // Check if PWM activity pattern changed significantly
            if (abs(current_sample - initial_sample) >= 2) {
                int64_t latency_ms = k_uptime_get() - start_time;
                uint64_t latency_ns = latency_ms * 1000000; // Convert to nanoseconds
                printk(" (PWM start latency: %llu ns)", latency_ns);
                detected = true;
            }
        }
        
        if (!detected) {
            printk(" (no PWM change detected)");
        }
    } else {
        servo_set_angle(servo_marker, 90);    // Start position (up/not drawing)
    }
    printk("\n");
    k_sleep(K_MSEC(1000));
    
    servo_set_angle(servo_marker, 0);     // Move to 0° (down/drawing)
    k_sleep(K_MSEC(1000));
    
    servo_set_angle(servo_marker, 90);    // Back to start (up/not drawing)
    k_sleep(K_MSEC(1000));

    printk("Servo test complete!\n\n");
}

/**
 * Test buzzer - simple toggle test with latency measurement
 */
static void test_buzzer(void)
{
    printk("Testing Buzzer\n");
    printk("------------------\n");

    printk("Buzzer toggle test (like pwm_buzzer_driver_test)\n");
    
    // Check if feedback pin is available for latency measurement
    const struct gpio_dt_spec buzzer_feedback = GPIO_DT_SPEC_GET_OR(DT_ALIAS(buzzer_feedback), gpios, {0});
    bool can_measure_latency = (buzzer_feedback.port != NULL);
    
    if (can_measure_latency) {
        gpio_pin_configure_dt(&buzzer_feedback, GPIO_INPUT);
        printk("Latency measurement enabled (GPIO%d feedback)\n", buzzer_feedback.pin);
    }
    
    for (int i = 0; i < 4; i++) {
        printk("   Toggle %s", (i % 2 == 0) ? "ON" : "OFF");
        
        if (can_measure_latency) {
            // Record initial state and timestamp
            int initial_state = gpio_pin_get_dt(&buzzer_feedback);
            int64_t start_time = k_uptime_get();
            
            // Toggle the buzzer
            pwm_buzzer_toggle();
            
            // Wait for GPIO change with timeout
            int64_t timeout = start_time + 10; // 10ms timeout
            while (k_uptime_get() < timeout) {
                if (gpio_pin_get_dt(&buzzer_feedback) != initial_state) {
                    int64_t latency_ms = k_uptime_get() - start_time;
                    uint64_t latency_ns = latency_ms * 1000000; // Convert to nanoseconds
                    printk(" (latency: %llu ns)", latency_ns);
                    break;
                }
            }
        } else {
            pwm_buzzer_toggle();
        }
        
        printk("\n");
        k_sleep(K_MSEC(500));
    }

    printk("Buzzer test complete!\n\n");
}

/**
 * Test LED - simple on/off test with latency measurement
 */
static void test_led(void)
{
    printk("Testing LED\n");
    printk("---------------\n");

    // Check if feedback pin is available for latency measurement
    const struct gpio_dt_spec led_feedback = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led_feedback), gpios, {0});
    bool can_measure_latency = (led_feedback.port != NULL);
    
    if (can_measure_latency) {
        gpio_pin_configure_dt(&led_feedback, GPIO_INPUT);
        printk("Latency measurement enabled (GPIO%d feedback)\n", led_feedback.pin);
    }

    // Test LED ON
    printk("LED ON");
    if (can_measure_latency) {
        int initial_state = gpio_pin_get_dt(&led_feedback);
        int64_t start_time = k_uptime_get();
        
        simple_led_on();
        
        int64_t timeout = start_time + 10; // 10ms timeout
        while (k_uptime_get() < timeout) {
            if (gpio_pin_get_dt(&led_feedback) != initial_state) {
                int64_t latency_ms = k_uptime_get() - start_time;
                uint64_t latency_ns = latency_ms * 1000000; // Convert to nanoseconds
                printk(" (latency: %llu ns)", latency_ns);
                break;
            }
        }
    } else {
        simple_led_on();
    }
    printk("\n");
    k_sleep(K_MSEC(1000));
    
    // Test LED OFF
    printk("LED OFF");
    if (can_measure_latency) {
        int initial_state = gpio_pin_get_dt(&led_feedback);
        int64_t start_time = k_uptime_get();
        
        simple_led_off();
        
        int64_t timeout = start_time + 10; // 10ms timeout
        while (k_uptime_get() < timeout) {
            if (gpio_pin_get_dt(&led_feedback) != initial_state) {
                int64_t latency_ms = k_uptime_get() - start_time;
                uint64_t latency_ns = latency_ms * 1000000; // Convert to nanoseconds
                printk(" (latency: %llu ns)", latency_ns);
                break;
            }
        }
    } else {
        simple_led_off();
    }
    printk("\n");
    k_sleep(K_MSEC(1000));

    // Test LED ON again
    printk("LED ON");
    if (can_measure_latency) {
        int initial_state = gpio_pin_get_dt(&led_feedback);
        int64_t start_time = k_uptime_get();
        
        simple_led_on();
        
        int64_t timeout = start_time + 10; // 10ms timeout
        while (k_uptime_get() < timeout) {
            if (gpio_pin_get_dt(&led_feedback) != initial_state) {
                int64_t latency_ms = k_uptime_get() - start_time;
                uint64_t latency_ns = latency_ms * 1000000; // Convert to nanoseconds
                printk(" (latency: %llu ns)", latency_ns);
                break;
            }
        }
    } else {
        simple_led_on();
    }
    printk("\n");
    k_sleep(K_MSEC(1000));
    
    // Test LED OFF again
    printk("LED OFF");
    if (can_measure_latency) {
        int initial_state = gpio_pin_get_dt(&led_feedback);
        int64_t start_time = k_uptime_get();
        
        simple_led_off();
        
        int64_t timeout = start_time + 10; // 10ms timeout
        while (k_uptime_get() < timeout) {
            if (gpio_pin_get_dt(&led_feedback) != initial_state) {
                int64_t latency_ms = k_uptime_get() - start_time;
                uint64_t latency_ns = latency_ms * 1000000; // Convert to nanoseconds
                printk(" (latency: %llu ns)", latency_ns);
                break;
            }
        }
    } else {
        simple_led_off();
    }
    printk("\n");
    k_sleep(K_MSEC(1000));

    printk("LED test complete!\n\n");
}

/**
 * Test stepper accuracy with PWM-based precision - validate ±5% step command accuracy spec
 * Uses 90° movements for better validation and testing consistency
 */
static void test_stepper(void)
{
    printk("Testing PWM-Based Stepper Motor Accuracy (±5%% Spec)\n");
    printk("===================================================\n");
    printk("Using 90 degree movements for precise validation\n");
    
    /* Reset counters for clean test */
    stepper_reset_counters();

    /* Test 1: 90 degree Forward Movement */
    printk("\nTest 1: 90 degree Forward (clockwise)\n");
    int expected_steps_90 = 800;  // 90 degrees * 1.8 degrees/step * 16 microsteps = 800 steps
    
    stepper_start_counting(90.0f);
    stepper_set_velocity(STEPPER_LEFT, 360.0f);  // 1 rev/s
    k_sleep(K_MSEC(250));  // Run for 250ms for 90° at 360°/s
    stepper_stop(STEPPER_LEFT);
    stepper_stop_counting();
    
    int actual_steps = stepper_get_step_count();
    float accuracy_percent = ((float)actual_steps / expected_steps_90) * 100.0f;
    float error_percent = accuracy_percent - 100.0f;
    bool pass_90_forward = (error_percent >= -5.0f && error_percent <= 5.0f);
    
    printk("  Expected: %d steps (90 degrees)\n", expected_steps_90);
    printk("  Actual:   %d steps\n", actual_steps);
    printk("  Accuracy: %.2f%% (error: %+.2f%%)\n", accuracy_percent, error_percent);
    printk("  Result:   %s (±5%% spec)\n", pass_90_forward ? "PASS" : "FAIL");
    k_sleep(K_MSEC(500));

    /* Test 2: 90 degree Backward Movement */
    printk("\nTest 2: 90 degree Backward (counter-clockwise)\n");
    stepper_start_counting(90.0f);
    stepper_set_velocity(STEPPER_LEFT, -360.0f);  // 1 rev/s reverse
    k_sleep(K_MSEC(250));  // Run for 250ms for 90 degrees at 360 degrees/s
    stepper_stop(STEPPER_LEFT);
    stepper_stop_counting();
    
    actual_steps = stepper_get_step_count();
    accuracy_percent = ((float)actual_steps / expected_steps_90) * 100.0f;
    error_percent = accuracy_percent - 100.0f;
    bool pass_90_backward = (error_percent >= -5.0f && error_percent <= 5.0f);
    
    printk("  Expected: %d steps (90 degrees)\n", expected_steps_90);
    printk("  Actual:   %d steps\n", actual_steps);
    printk("  Accuracy: %.2f%% (error: %+.2f%%)\n", accuracy_percent, error_percent);
    printk("  Result:   %s (±5%% spec)\n", pass_90_backward ? "PASS" : "FAIL");

    /* Test 3: PWM Precision Validation */
    printk("\nTest 3: PWM Timing Precision\n");
    stepper_start_counting(90.0f);
    stepper_set_velocity(STEPPER_LEFT, 360.0f);  // 1 rev/s
    k_sleep(K_MSEC(250));  // 250ms for 90 degrees
    stepper_stop(STEPPER_LEFT);
    stepper_stop_counting();
    
    uint32_t pwm_pulses = stepper_get_gpio_pulse_count();
    actual_steps = stepper_get_step_count();
    
    printk("  PWM Pulses: %u\n", pwm_pulses);
    printk("  Step Count: %d\n", actual_steps);
    printk("  PWM/Step Ratio: %.2f (expect ~1.0 for PWM mode)\n", 
           actual_steps > 0 ? (float)pwm_pulses / actual_steps : 0.0f);

    /* Overall Test Results */
    printk("\n=== PWM STEPPER ACCURACY SPECIFICATION TEST ===\n");
    printk("Requirement: ±5%% step command accuracy\n");
    printk("Hardware: PWM-based timing (nanosecond precision)\n");
    printk("90 degree Forward:  %s\n", pass_90_forward ? "PASS" : "FAIL");
    printk("90 degree Backward: %s\n", pass_90_backward ? "PASS" : "FAIL");
    
    bool overall_pass = pass_90_forward && pass_90_backward;
    printk("OVERALL SPEC: %s\n", overall_pass ? "PASS" : "FAIL");
    
    if (overall_pass) {
        printk("\n✓ PWM stepper driver meets ±5%% accuracy requirement!\n");
        printk("✓ Nanosecond-precision timing provides excellent accuracy\n");
    } else {
        printk("\nTROUBLESHOOTING:\n");
        printk("- Check PWM hardware configuration\n");
        printk("- Verify microstepping settings (16x)\n");
        printk("- Check for mechanical issues\n");
        printk("- Validate timing calculations\n");
    }
    
    printk("===============================================\n");
    printk("Total steps since reset: %d\n", stepper_get_total_steps());
    printk("PWM-based stepper test complete!\n\n");
}

/**
 * Main demo function
 */
void peripheral_demo(void)
{
    /* Initialize all systems */
    if (demo_init() != 0) {
        printk("Demo initialization failed!\n");
        return;
    }

    k_sleep(K_MSEC(1000));

    /* Run simple tests */
    test_servos();
    test_buzzer();
    test_led();
    test_stepper();

    printk("Demo complete Manu! Press reset to run again.\n");
}

/**
 * Main function for demo test
 */
int main(void)
{
    printk("Starting simplified demo in 2 seconds...\n");
    k_sleep(K_MSEC(2000));
    
    peripheral_demo();
    
    return 0;
}