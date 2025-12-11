/*
 * SPDX-License-Identifier: Apache-2.0
 * A4988 Stepper Motor Driver using Zephyr's step_dir framework
 */

#define DT_DRV_COMPAT doodle_a4988_stepper

#include <zephyr/kernel.h>
#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/gpio.h>
#include "step_dir/step_dir_stepper_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(a4988, CONFIG_STEPPER_LOG_LEVEL);

struct a4988_config {
	struct step_dir_stepper_common_config common;
	struct gpio_dt_spec en_pin;
};

struct a4988_data {
	struct step_dir_stepper_common_data common;
	enum stepper_micro_step_resolution micro_step_res;
};

STEP_DIR_STEPPER_STRUCT_CHECK(struct a4988_config, struct a4988_data);

static int a4988_stepper_enable(const struct device *dev)
{
	const struct a4988_config *cfg = dev->config;
	if (!cfg->en_pin.port) return -ENOTSUP;
	return gpio_pin_set_dt(&cfg->en_pin, 1);  /* Active LOW */
}

static int a4988_stepper_disable(const struct device *dev)
{
	const struct a4988_config *cfg = dev->config;
	if (!cfg->en_pin.port) return -ENOTSUP;
	return gpio_pin_set_dt(&cfg->en_pin, 0);
}

static int a4988_set_micro_step_res(const struct device *dev,
				    enum stepper_micro_step_resolution res)
{
	struct a4988_data *data = dev->data;
	if (res > STEPPER_MICRO_STEP_16) return -ENOTSUP;
	data->micro_step_res = res;
	return 0;
}

static int a4988_get_micro_step_res(const struct device *dev,
				    enum stepper_micro_step_resolution *res)
{
	*res = ((struct a4988_data *)dev->data)->micro_step_res;
	return 0;
}

static int a4988_init(const struct device *dev)
{
	const struct a4988_config *cfg = dev->config;
	int ret;

	if (cfg->en_pin.port) {
		if (!gpio_is_ready_dt(&cfg->en_pin)) return -ENODEV;
		ret = gpio_pin_configure_dt(&cfg->en_pin, GPIO_OUTPUT_INACTIVE);
		if (ret) return ret;
	}

	ret = step_dir_stepper_common_init(dev);
	if (ret) return ret;

	gpio_pin_set_dt(&cfg->common.step_pin, 0);
	LOG_INF("%s initialized", dev->name);
	return 0;
}

static DEVICE_API(stepper, a4988_api) = {
	.enable = a4988_stepper_enable,
	.disable = a4988_stepper_disable,
	.move_by = step_dir_stepper_common_move_by,
	.move_to = step_dir_stepper_common_move_to,
	.is_moving = step_dir_stepper_common_is_moving,
	.set_reference_position = step_dir_stepper_common_set_reference_position,
	.get_actual_position = step_dir_stepper_common_get_actual_position,
	.set_microstep_interval = step_dir_stepper_common_set_microstep_interval,
	.run = step_dir_stepper_common_run,
	.stop = step_dir_stepper_common_stop,
	.set_micro_step_res = a4988_set_micro_step_res,
	.get_micro_step_res = a4988_get_micro_step_res,
	.set_event_callback = step_dir_stepper_common_set_event_callback,
};

#define A4988_DEVICE(inst)                                                     \
	static const struct a4988_config a4988_cfg_##inst = {                  \
		.common = STEP_DIR_STEPPER_DT_INST_COMMON_CONFIG_INIT(inst),   \
		.en_pin = GPIO_DT_SPEC_INST_GET_OR(inst, en_gpios, {0}),       \
	};                                                                     \
	static struct a4988_data a4988_data_##inst = {                         \
		.common = STEP_DIR_STEPPER_DT_INST_COMMON_DATA_INIT(inst),     \
		.micro_step_res = DT_INST_PROP(inst, micro_step_res),          \
	};                                                                     \
	DEVICE_DT_INST_DEFINE(inst, a4988_init, NULL, &a4988_data_##inst,      \
			      &a4988_cfg_##inst, POST_KERNEL,                  \
			      CONFIG_STEPPER_INIT_PRIORITY, &a4988_api);

DT_INST_FOREACH_STATUS_OKAY(A4988_DEVICE)
