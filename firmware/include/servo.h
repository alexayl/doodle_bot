#pragma once

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public API for the servo driver */
int servo_set_angle(const struct device *dev, uint8_t angle);

/* Get device by DT alias and set angle */
int servo_set_angle_by_alias(const char *alias, uint8_t angle);

/* Helper macro to get servo device by DT alias */
#define SERVO_DT_GET_BY_ALIAS(alias) DEVICE_DT_GET(DT_ALIAS(alias))

#ifdef __cplusplus
}
#endif
