/**
 * Simplified Doodle Bot Peripheral Demo
 * 
 * Quick demo showing basic functionality:
 * - Servos: Move back and forth
 * - Buzzer: Toggle on/off (using PWM buzzer toggle)
 * - LED: Toggle on/off (GPIO11)
 * - Stepper: 360° forward and backward
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

// Include all your drivers
#include "stepper.h"
#include "pwm_buzzer.h"
#include "servo.h"
#include "simple_led.h"

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
 * Test servos - move back and forth
 */
static void test_servos(void)
{
    printk("Testing Servos\n");
    printk("-----------------\n");

    /* Move eraser servo */
    printk("Eraser servo: 180° → 0° → 180°\n");
    servo_set_angle(servo_eraser, 180);   // Start position
    k_sleep(K_MSEC(1000));
    
    servo_set_angle(servo_eraser, 0);    // Move to 0°
    k_sleep(K_MSEC(1000));
    
    servo_set_angle(servo_eraser, 180);   // Back to start
    k_sleep(K_MSEC(1000));

    /* Move marker servo */
    printk("Marker servo: 180° → 0° → 180°\n");
    servo_set_angle(servo_marker, 180);   // Start position
    k_sleep(K_MSEC(1000));
    
    servo_set_angle(servo_marker, 0);    // Move to 0°
    k_sleep(K_MSEC(1000));
    
    servo_set_angle(servo_marker, 180);   // Back to start
    k_sleep(K_MSEC(1000));

    printk("Servo test complete!\n\n");
}

/**
 * Test buzzer - simple toggle test like the working buzzer test
 */
static void test_buzzer(void)
{
    printk("Testing Buzzer\n");
    printk("------------------\n");

    printk("Buzzer toggle test (like pwm_buzzer_driver_test)\n");
    
    for (int i = 0; i < 6; i++) {
        printk("   Toggle %s\n", (i % 2 == 0) ? "ON" : "OFF");
        pwm_buzzer_toggle();
        k_sleep(K_MSEC(500));
    }

    printk("Buzzer test complete!\n\n");
}

/**
 * Test LED - simple on/off/toggle test
 */
static void test_led(void)
{
    printk("Testing LED\n");
    printk("---------------\n");

    printk("LED ON\n");
    simple_led_on();
    k_sleep(K_MSEC(1000));
    
    printk("LED OFF\n");
    simple_led_off();
    k_sleep(K_MSEC(1000));
    

    printk("LED test complete!\n\n");
}

/**
 * Test stepper - 360° forward and backward
 */
static void test_stepper(void)
{
    printk("Testing Stepper Motor\n");
    printk("-------------------------\n");

    printk("360° Forward (clockwise)\n");
    stepper_set_velocity(STEPPER_LEFT, 360.0f);  // 1 rev/s
    k_sleep(K_MSEC(1000));  // Run for 1 second = 360°
    stepper_stop(STEPPER_LEFT);
    k_sleep(K_MSEC(500));

    printk("360° Backward (counter-clockwise)\n");
    stepper_set_velocity(STEPPER_LEFT, -360.0f);  // 1 rev/s reverse
    k_sleep(K_MSEC(1000));  // Run for 1 second = 360°
    stepper_stop(STEPPER_LEFT);
    k_sleep(K_MSEC(500));

    printk("Stepper test complete!\n\n");
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