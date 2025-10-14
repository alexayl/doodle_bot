/*
 * Simple PWM test for P0.12 - Direct PWM output without servo driver
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(pwm_test, CONFIG_LOG_DEFAULT_LEVEL);

/* LED configuration */
#define LED_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

/* PWM configuration for P0.12 */
static const struct pwm_dt_spec pwm_servo = PWM_DT_SPEC_GET(DT_NODELABEL(servo_marker));

int main(void)
{
    LOG_INF("=== DIRECT PWM TEST - P0.12 ===");

    /* Initialize LED */
    if (gpio_is_ready_dt(&led)) {
        gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
        gpio_pin_set_dt(&led, 1);
        LOG_INF("LED2 turned ON - program running!");
    }

    /* Check PWM device */
    if (!pwm_is_ready_dt(&pwm_servo)) {
        LOG_ERR("PWM device not ready for P0.12");
        while(1) { k_sleep(K_SECONDS(1)); }
    }

    LOG_INF("PWM device ready - testing P0.12 with different pulse widths");

    uint32_t pulse_widths[] = {500, 1000, 1500, 2000, 2500}; // microseconds
    int num_pulses = sizeof(pulse_widths) / sizeof(pulse_widths[0]);
    
    bool led_state = false;
    int pulse_index = 0;

    while (1) {
        uint32_t pulse_us = pulse_widths[pulse_index];
        
        LOG_INF("Setting PWM pulse to %d microseconds on P0.12", pulse_us);
        
        int ret = pwm_set_pulse_dt(&pwm_servo, PWM_USEC(pulse_us));
        if (ret < 0) {
            LOG_ERR("Failed to set PWM pulse: %d", ret);
        }

        /* Toggle LED to show activity */
        led_state = !led_state;
        gpio_pin_set_dt(&led, led_state);

        /* Move to next pulse width */
        pulse_index = (pulse_index + 1) % num_pulses;
        
        k_sleep(K_SECONDS(2));
    }

    return 0;
}