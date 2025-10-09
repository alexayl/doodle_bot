/*
 * Simple servo motor test - both eraser and marker servos
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "servo.h"

LOG_MODULE_REGISTER(servo_test, CONFIG_LOG_DEFAULT_LEVEL);

/* Forward declaration for servo driver API */
extern "C" int servo_set_angle(const struct device *dev, uint8_t angle);

extern "C" {

/* Servo device pointers */
static const struct device *servo_eraser_dev = DEVICE_DT_GET(DT_NODELABEL(servo_eraser));
static const struct device *servo_marker_dev = DEVICE_DT_GET(DT_NODELABEL(servo_marker));

} /* extern "C" */

int main(void)
{
    LOG_INF("Simple Servo Test - Eraser and Marker Servos");

    if (!device_is_ready(servo_eraser_dev)) {
        LOG_ERR("ERROR: Servo eraser device is not ready!");
        LOG_ERR("Check devicetree: servo_eraser node with valid PWM configuration");
        return -1;
    }

    if (!device_is_ready(servo_marker_dev)) {
        LOG_ERR("ERROR: Servo marker device is not ready!");
        LOG_ERR("Check devicetree: servo_marker node with valid PWM configuration");
        return -1;
    }

    LOG_INF("Both servo devices initialized successfully");

    while (1) {
        LOG_INF("Moving both servos to 45 degrees");
        servo_set_angle(servo_eraser_dev, 45);
        servo_set_angle(servo_marker_dev, 45);
        k_sleep(K_SECONDS(2));

        LOG_INF("Moving both servos to 135 degrees");
        servo_set_angle(servo_eraser_dev, 135);
        servo_set_angle(servo_marker_dev, 135);
        k_sleep(K_SECONDS(2));

        LOG_INF("Moving both servos to center (90 degrees)");
        servo_set_angle(servo_eraser_dev, 90);
        servo_set_angle(servo_marker_dev, 90);
        k_sleep(K_SECONDS(2));
    }

    return 0;
}