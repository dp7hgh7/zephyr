/*
 * Copyright (c) 2022 Caspar Friedrich <c.s.w.friedrich@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "w1_ds248x_common.h"

#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <drivers/w1.h>
#include <logging/log.h>
#include <zephyr.h>

#define DT_DRV_COMPAT maxim_ds2484

LOG_MODULE_REGISTER(ds2484, CONFIG_W1_LOG_LEVEL);

struct ds2484_config {
	struct w1_controller_config w1_config;
	const struct i2c_dt_spec i2c_spec;
	const struct gpio_dt_spec slpz_spec;
	bool apu;
};

struct ds2484_data {
	struct w1_controller_data w1_data;
	uint8_t reg_device_config;
};

static inline const struct ds2484_config *get_config(const struct device *dev)
{
	return dev->config;
}

static inline struct ds2484_data *get_data(const struct device *dev)
{
	return dev->data;
}

static int ds2484_reset_bus(const struct device *dev)
{
	return ds248x_reset_bus(&get_config(dev)->i2c_spec);
}

static int ds2484_read_bit(const struct device *dev)
{
	return ds248x_read_bit(&get_config(dev)->i2c_spec);
}

static int ds2484_write_bit(const struct device *dev, bool bit)
{
	return ds248x_write_bit(&get_config(dev)->i2c_spec, bit);
}

static int ds2484_read_byte(const struct device *dev)
{
	return ds248x_read_byte(&get_config(dev)->i2c_spec);
}

static int ds2484_write_byte(const struct device *dev, uint8_t byte)
{
	return ds248x_write_byte(&get_config(dev)->i2c_spec, byte);
}

static int ds2484_configure(const struct device *dev, enum w1_driver_settings_type type,
			    uint32_t value)
{
	switch (type) {
	case W1_SETTING_SPEED:
		WRITE_BIT(get_data(dev)->reg_device_config, DEVICE_1WS_pos, value);
		break;
	case W1_SETTING_STRONG_PULLUP:
		WRITE_BIT(get_data(dev)->reg_device_config, DEVICE_SPU_pos, value);
		break;
	default:
		return -EINVAL;
	}

	return ds248x_write_config(&get_config(dev)->i2c_spec, get_data(dev)->reg_device_config);
}

#ifdef CONFIG_PM_DEVICE
static int ds2484_pm_control(const struct device *dev, enum pm_device_action action)
{
	int ret;

	uint8_t reg;

	if (!device_is_ready(get_config(dev)->slpz_spec.port)) {
		return -EINVAL;
	}

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		ret = gpio_pin_set_dt(&get_config(dev)->slpz_spec, 0);
		if (ret) {
			return ret;
		}
		break;
	case PM_DEVICE_ACTION_SUSPEND:
	case PM_DEVICE_ACTION_FORCE_SUSPEND:
		ret = gpio_pin_set_dt(&get_config(dev)->slpz_spec, 1);
		if (ret) {
			return ret;
		}
		break;
	case PM_DEVICE_ACTION_TURN_OFF:
	case PM_DEVICE_ACTION_LOW_POWER:
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static int ds2484_init(const struct device *dev)
{
	int ret;

	if (!device_is_ready(get_config(dev)->slpz_spec.port)) {
		LOG_ERR("Port (SLPZ) not ready");
		return -EINVAL;
	}

	ret = gpio_pin_configure_dt(&get_config(dev)->slpz_spec, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		LOG_ERR("Pin configuration (SLPZ) failed: %d", ret);
		return ret;
	}

	ret = ds248x_reset_device(&get_config(dev)->i2c_spec);
	if (ret) {
		LOG_ERR("Device reset failed: %d", ret);
		return ret;
	}

	WRITE_BIT(get_data(dev)->reg_device_config, DEVICE_APU_pos, get_config(dev)->apu);

	ret = ds248x_write_config(&get_config(dev)->i2c_spec, get_data(dev)->reg_device_config);
	if (ret) {
		LOG_ERR("Device config update failed: %d", ret);
		return ret;
	}

	return 0;
}

static const struct w1_driver_api ds2484_driver_api = {
	.reset_bus = ds2484_reset_bus,
	.read_bit = ds2484_read_bit,
	.write_bit = ds2484_write_bit,
	.read_byte = ds2484_read_byte,
	.write_byte = ds2484_write_byte,
	.configure = ds2484_configure,
};

#define DS2484_INIT(n)                                                                             \
	static const struct ds2484_config inst_##n##_config = {                                    \
		.w1_config.client_count = W1_PERIPHERALS_COUNT(DT_DRV_INST(n)),                    \
		.i2c_spec = I2C_DT_SPEC_INST_GET(n),                                               \
		.slpz_spec = GPIO_DT_SPEC_INST_GET(n, slpz_gpios),                                 \
		.apu = DT_INST_PROP(n, active_pullup),                                             \
	};                                                                                         \
	static struct ds2484_data inst_##n##_data;                                                 \
	DEVICE_DT_INST_DEFINE(n, ds2484_init, ds2484_pm_control, &inst_##n##_data,                 \
			      &inst_##n##_config, POST_KERNEL, CONFIG_W1_INIT_PRIORITY,            \
			      &ds2484_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DS2484_INIT)
