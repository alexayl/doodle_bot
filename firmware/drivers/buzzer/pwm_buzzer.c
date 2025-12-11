/**
 * PWM Buzzer Driver
 * Supports on/off, toggle, and beep with configurable frequency/volume
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>
#include "pwm_buzzer.h"

// Hardware setup - platform specific
#if DT_NODE_EXISTS(DT_NODELABEL(ledc0))
    // ESP32 platform
    #define PWM_DEV_NODE DT_NODELABEL(ledc0)
    #define PWM_CHANNEL 2  // GPIO12
#elif DT_NODE_EXISTS(DT_NODELABEL(pwm0))
    // nRF platform
    #define PWM_DEV_NODE DT_NODELABEL(pwm0)
    #define PWM_CHANNEL 3  // Channel 3 for buzzer on nRF
#else
    #error "No compatible PWM controller found"
#endif

#define PWM_DEV DEVICE_DT_GET(PWM_DEV_NODE)

// Default buzzer settings
#define DEFAULT_FREQUENCY_HZ 400
#define DEFAULT_VOLUME_PERCENT 10

static bool buzzer_active = false;
static bool buzzer_ready = false;
static uint16_t current_frequency = DEFAULT_FREQUENCY_HZ;
static uint8_t current_volume = DEFAULT_VOLUME_PERCENT;

int pwm_buzzer_init(void)
{
    buzzer_ready = device_is_ready(PWM_DEV);
    return buzzer_ready ? 0 : -1;
}

bool pwm_buzzer_is_ready(void)
{
    return buzzer_ready;
}

int pwm_buzzer_on(void)
{
    if (!buzzer_ready) return -1;
    
    uint32_t period_ns = 1000000000U / current_frequency;
    uint32_t duty_ns = period_ns * current_volume / 100;
    
    pwm_set(PWM_DEV, PWM_CHANNEL, period_ns, duty_ns, 0);
    buzzer_active = true;
    return 0;
}

int pwm_buzzer_off(void)
{
    if (!buzzer_ready) return -1;
    
    uint32_t period_ns = 1000000000U / current_frequency;
    pwm_set(PWM_DEV, PWM_CHANNEL, period_ns, 0, 0);
    buzzer_active = false;
    return 0;
}

int pwm_buzzer_toggle(void)
{
    return buzzer_active ? pwm_buzzer_off() : pwm_buzzer_on();
}

int pwm_buzzer_beep(uint16_t frequency_hz, uint8_t volume_percent, uint32_t duration_ms)
{
    if (!buzzer_ready) return -1;
    
    uint32_t period_ns = 1000000000U / frequency_hz;
    uint32_t duty_ns = period_ns * volume_percent / 100;
    
    pwm_set(PWM_DEV, PWM_CHANNEL, period_ns, duty_ns, 0);
    k_sleep(K_MSEC(duration_ms));
    pwm_set(PWM_DEV, PWM_CHANNEL, period_ns, 0, 0);
    
    return 0;
}

bool pwm_buzzer_is_active(void)
{
    return buzzer_active;
}

uint16_t pwm_buzzer_get_frequency(void)
{
    return current_frequency;
}

uint8_t pwm_buzzer_get_volume(void)
{
    return current_volume;
}

int pwm_buzzer_set_frequency(uint16_t frequency_hz)
{
    current_frequency = frequency_hz;
    return 0;
}

int pwm_buzzer_set_volume(uint8_t volume_percent)
{
    current_volume = volume_percent > 100 ? 100 : volume_percent;
    return 0;
}

int pwm_buzzer_update(void)
{
    if (buzzer_active) {
        return pwm_buzzer_on();
    }
    return 0;
}