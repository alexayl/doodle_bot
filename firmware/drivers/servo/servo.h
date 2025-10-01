#pragma once

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public API for the servo driver */
int servo_set_angle(const struct device *dev, uint8_t angle);

/* Get device by DT nodelabel and set angle */
int servo_set_angle_by_name(const char *name, uint8_t angle);

/* Helper macro to get servo device by DT nodelabel */
#define SERVO_DT_GET(node_id) DEVICE_DT_GET(node_id)

#ifdef __cplusplus
}
#endif
