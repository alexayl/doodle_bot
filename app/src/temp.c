/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>

#define BLINK_PERIOD_MS_STEP 100U
#define BLINK_PERIOD_MS_MAX  1000U

int main(void)
{
	int ret;
	unsigned int period_ms = BLINK_PERIOD_MS_MAX;
	const struct device *sensor;
	struct sensor_value last_val = { 0 }, val;

	sensor = DEVICE_DT_GET(DT_NODELABEL(example_sensor));
	if (!device_is_ready(sensor)) {
		printk("Sensor not ready");
		return 0;
	}

	printk("Use the sensor to change LED blinking period\n");

	while (1) {
		ret = sensor_sample_fetch(sensor);
		if (ret < 0) {
			printk("Could not fetch sample (%d)", ret);
			return 0;
		}

		ret = sensor_channel_get(sensor, SENSOR_CHAN_PROX, &val);
		if (ret < 0) {
			printk("Could not get sample (%d)", ret);
			return 0;
		}

		last_val = val;

		k_sleep(K_MSEC(100));
	}

	return 0;
}

