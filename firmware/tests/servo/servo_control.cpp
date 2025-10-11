#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>

#define SERVO_PWM_NODE     DT_ALIAS(servo0)
#define SERVO_PWM_CTLR     DT_PWMS_CTLR(SERVO_PWM_NODE)
#define SERVO_PWM_CHANNEL  DT_PWMS_CHANNEL(SERVO_PWM_NODE)
#define SERVO_PWM_PERIOD   DT_PWMS_PERIOD(SERVO_PWM_NODE)

int main(void)
{
    printk("=== Hardware PWM Servo Test ===\n");
    
    const struct device *pwm_dev = DEVICE_DT_GET(SERVO_PWM_CTLR);
    
    if (!device_is_ready(pwm_dev)) {
        printk("Error: PWM device not ready\n");
        return -1;
    }
    
    printk("PWM device ready: %s\n", pwm_dev->name);
    printk("Channel: %d, Period: %d ns\n", SERVO_PWM_CHANNEL, SERVO_PWM_PERIOD);
    
    uint32_t pulse_width = 1500000;
    
    int ret = pwm_set(pwm_dev, SERVO_PWM_CHANNEL, SERVO_PWM_PERIOD, pulse_width, 0);
    if (ret < 0) {
        printk("Error: pwm_set failed with error %d\n", ret);
        return ret;
    }
    
    printk("PWM set successfully - servo should be at center position\n");
    
    while (1) {
        printk("Setting 0 degrees (1ms pulse)\n");
        ret = pwm_set(pwm_dev, SERVO_PWM_CHANNEL, SERVO_PWM_PERIOD, 1000000, 0);
        if (ret < 0) printk("Error: %d\n", ret);
        k_sleep(K_SECONDS(1));
        
        printk("Setting 180 degrees (2ms pulse)\n");
        ret = pwm_set(pwm_dev, SERVO_PWM_CHANNEL, SERVO_PWM_PERIOD, 2000000, 0);
        if (ret < 0) printk("Error: %d\n", ret);
        k_sleep(K_SECONDS(1));
        
        printk("Setting 90 degrees (1.5ms pulse)\n");
        ret = pwm_set(pwm_dev, SERVO_PWM_CHANNEL, SERVO_PWM_PERIOD, 1500000, 0);
        if (ret < 0) printk("Error: %d\n", ret);
        k_sleep(K_SECONDS(1));
    }
    
    return 0;
}
