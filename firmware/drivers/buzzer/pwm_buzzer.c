/**
 * Super Simple PWM Buzzer Driver
 * Just toggle buzzer on/off with fixed settings
 */

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>
#include "pwm_buzzer.h"

// Hardware setup
#define PWM_DEV_NODE DT_NODELABEL(ledc0)
#define PWM_DEV DEVICE_DT_GET(PWM_DEV_NODE)
#define PWM_CHANNEL 2  // GPIO12

// Fixed buzzer settings - percentage based volume
#define BUZZER_FREQUENCY_HZ 400
#define BUZZER_VOLUME_PERCENT 10  // adjust this to change loudness
#define BUZZER_PERIOD_NS (1000000000U / BUZZER_FREQUENCY_HZ)  // 2.5ms period
#define BUZZER_DUTY_NS (BUZZER_PERIOD_NS * BUZZER_VOLUME_PERCENT / 100)

static bool buzzer_active = false;

int pwm_buzzer_init(void)
{
    return device_is_ready(PWM_DEV) ? 0 : -1;
}

int pwm_buzzer_on(void)
{
    pwm_set(PWM_DEV, PWM_CHANNEL, BUZZER_PERIOD_NS, BUZZER_DUTY_NS, 0);
    buzzer_active = true;
    return 0;
}

int pwm_buzzer_off(void)
{
    // Keep same frequency but set duty cycle to 0 (truly silent)
    pwm_set(PWM_DEV, PWM_CHANNEL, BUZZER_PERIOD_NS, 0, 0);
    buzzer_active = false;
    return 0;
}

int pwm_buzzer_toggle(void)
{
    return buzzer_active ? pwm_buzzer_off() : pwm_buzzer_on();
}