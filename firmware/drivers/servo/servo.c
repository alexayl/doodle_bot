/**
 * Simple Zephyr servo driver
 * - Uses Zephyr PWM API
 * - Exposes a simple servo_set_angle(dev, angle) function
 * - Uses instance-based device definition
 */

#define DT_DRV_COMPAT doodle_servo

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(servo, CONFIG_SERVO_LOG_LEVEL);

#define SERVO_MIN_PULSE 500U   /* 0.5ms for 0° */
#define SERVO_MAX_PULSE 2500U  /* 2.5ms for 180° */

struct servo_config {
    struct pwm_dt_spec pwm;
};

struct servo_data {
    /* Runtime data if needed */
};


int servo_set_angle(const struct device *dev, uint8_t angle)
{
    const struct servo_config *config = dev->config;
    
    if (!device_is_ready(config->pwm.dev)) {
        LOG_ERR("PWM device not ready");
        return -ENODEV;
    }
    
    if (angle > 180) {
        angle = 180;
    }

    uint32_t pulse = SERVO_MIN_PULSE + ((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle) / 180U;
    uint32_t period = 20000U; /* 20ms period for servo */
    printk("IN THE DRIVER SETTING TEH ANGE\n");
    return pwm_set_dt(&config->pwm, PWM_USEC(period), PWM_USEC(pulse));
}

int servo_set_angle_by_alias(const char *alias, uint8_t angle)
{
    const struct device *dev = NULL;
    
    if (strcmp(alias, "servoe") == 0 || strcmp(alias, "servo-eraser") == 0) {
        dev = DEVICE_DT_GET(DT_NODELABEL(servo_eraser));
    } else if (strcmp(alias, "servom") == 0 || strcmp(alias, "servo-marker") == 0) {
        dev = DEVICE_DT_GET(DT_NODELABEL(servo_marker));
    }
    
    if (!dev) {
        LOG_ERR("Servo device with alias '%s' not found", alias);
        return -ENODEV;
    }
    return servo_set_angle(dev, angle);
}

const struct device *servo_init_by_alias(const char *alias)
{
    const struct device *dev = NULL;
    
    if (strcmp(alias, "servoe") == 0 || strcmp(alias, "servo-eraser") == 0) {
        dev = DEVICE_DT_GET(DT_NODELABEL(servo_eraser));
    } else if (strcmp(alias, "servom") == 0 || strcmp(alias, "servo-marker") == 0) {
        dev = DEVICE_DT_GET(DT_NODELABEL(servo_marker));
    }
    
    if (!dev) {
        LOG_ERR("Servo device with alias '%s' not found", alias);
        return NULL;
    }
    
    if (!device_is_ready(dev)) {
        LOG_ERR("Servo device '%s' not ready", dev->name);
        return NULL;
    }
    
    LOG_INF("Servo initialized by alias '%s' -> device %s", alias, dev->name);
    return dev;
}

static int servo_init(const struct device *dev)
{
    const struct servo_config *config = dev->config;
    
    if (!device_is_ready(config->pwm.dev)) {
        LOG_ERR("PWM device not ready");
        return -ENODEV;
    }

    LOG_INF("Servo initialized on PWM device %s", config->pwm.dev->name);
    return 0;
}

#define SERVO_DEFINE(inst) \
    static const struct servo_config servo_config_##inst = { \
        .pwm = PWM_DT_SPEC_INST_GET(inst), \
    }; \
    static struct servo_data servo_data_##inst; \
    DEVICE_DT_INST_DEFINE(inst, servo_init, NULL, &servo_data_##inst, &servo_config_##inst, POST_KERNEL, CONFIG_SERVO_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(SERVO_DEFINE)
